/*

Copyright (c) 2010, Stephan Weiss, ASL, ETH Zurich, Switzerland
You can contact the author at <stephan dot weiss at ieee dot org>

All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
* Redistributions of source code must retain the above copyright
notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright
notice, this list of conditions and the following disclaimer in the
documentation and/or other materials provided with the distribution.
* Neither the name of ETHZ-ASL nor the
names of its contributors may be used to endorse or promote products
derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL ETHZ-ASL BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef VISIONPOSE_MEASUREMENTS_H
#define VISIONPOSE_MEASUREMENTS_H

#include <ros/ros.h>
#include <ssf_core/measurement.h>
#include "visionpose_sensor.h"

#include <cmath> // for M_PI
#include <vector>

#include <chrono>
#include <thread>

class VisionPoseMeasurements : public ssf_core::Measurements
{
public:
	VisionPoseMeasurements() // hm: the first constructor to call, since "main.cpp"
	{
		addHandler(new VisionPoseSensorHandler(this));
		// hm: addHandler is defined in "measurement.h" in ssf_core
		// data structure is std::vector<MeasurementHandler*>
		// "handlers" is not really used, only for keeping track of the new object, and to be destroyed in the desctructor

		// hm: VisionPoseSensorHandler(*) is defined in "visionpose_sensor.h/cpp", this initialise parameters including
		// measurement_world_sensor and use_fixed_covariance
		// subscribe() is called as well
		//  - "visionpose_measurement" topic
		//  - "meas_noise1" "meas_noise2" reconfigurable

		// VisionPoseSensorHandler(*) is responsible for handling measurementCallback
		// - it also inherit and public SSF_Core() instance: ssf_core_. this is how SSF_core is engaged

		ros::NodeHandle pnh("~");
                pnh.param("use_imu_internal_q", use_imu_internal_q, false);


		pnh.param("init/p_ci/x", p_ci_[0], 0.0);
		pnh.param("init/p_ci/y", p_ci_[1], 0.0);
		pnh.param("init/p_ci/z", p_ci_[2], 0.0);

		pnh.param("init/q_ci/w", q_ci_.w(), 1.0);
		pnh.param("init/q_ci/x", q_ci_.x(), 0.0);
		pnh.param("init/q_ci/y", q_ci_.y(), 0.0);
		pnh.param("init/q_ci/z", q_ci_.z(), 0.0);
		q_ci_.normalize();

		pnh.param("init/q_wv/w", q_wv_.w(), 1.0);
		pnh.param("init/q_wv/x", q_wv_.x(), 0.0);
		pnh.param("init/q_wv/y", q_wv_.y(), 0.0);
		pnh.param("init/q_wv/z", q_wv_.z(), 0.0);
		q_wv_.normalize();

		Eigen::Quaternion<double> q_sw_;
                pnh.getParam("init/q_sw/w", q_sw_.w());
                pnh.getParam("init/q_sw/x", q_sw_.x());
                pnh.getParam("init/q_sw/y", q_sw_.y());
                pnh.getParam("init/q_sw/z", q_sw_.z());
		q_sw_.normalize();

		R_sw = q_sw_.toRotationMatrix();

		// the sensor's world frame of Ellipse is NED, we are using ENU, therefore conversion is needed.
		
		//R_sw << 0 , 1 , 0,
		//1 , 0 , 0,
		//0 , 0 , -1;

		// For Visensor, imu's global frame is NWU, we are still using ENU	

		// R_sw << 0 , 1 , 0,
        //         -1 , 0 , 0,
        //         0 , 0 , 1;

		

		pnh.param("scale_init", scale_, 1.0);


		ROS_INFO_STREAM("p_ci_: (x,y,z): ["  << p_ci_.x() << ", " << p_ci_.y() << ", " << p_ci_.z()  << "]");
		ROS_INFO_STREAM("q_ci_: (w,x,y,z): [" << q_ci_.w() << ", " << q_ci_.x() << ", " << q_ci_.y() << ", " << q_ci_.z()  << "]");
		ROS_INFO_STREAM("q_wv_: (w,x,y,z): [" << q_wv_.w() << ", " << q_wv_.x() << ", " << q_wv_.y() << ", " << q_wv_.z()  << "]");
		ROS_INFO_STREAM("R_sw: " << std::endl << R_sw);
		ROS_INFO_STREAM("scale_: " << scale_);
	}

	bool initStateZero(struct ssf_core::ImuInputsCache& imuEstimateMean)
	{

		Eigen::Matrix3d R_wi,R_iw; 
		if (use_imu_internal_q == false)
		{
			//////////////////////////////////
			/// q_iw estimation
			//////////////////////////////////

			Eigen::Vector3d normalvec_m;
			// if there is no magnetometer input, set it to x direction
			if (imuEstimateMean.m_m_.isZero())
			{
				ROS_WARN("There is no magnetometer readings, assume North is in +Y-axis");
				normalvec_m << 0, 1, 0;
			}else{ // there is readings
				normalvec_m = imuEstimateMean.m_m_.normalized().array();
			}

			// std::cout << "normalvec_m " << normalvec_m << std::endl;

			Eigen::Vector3d normalvec_down;
			if (imuEstimateMean.a_m_.norm() > 11 || imuEstimateMean.a_m_.norm() < 8)
			{
				ROS_WARN_STREAM("The gravity readings might be out of range: " << imuEstimateMean.a_m_.norm() );
				exit(1);
			}
			
			//// IMPORTANT: gravity direction is straight upwards!
			

			

			normalvec_down = - imuEstimateMean.a_m_.normalized().array(); // negative direction of gravity acceleration

			//std::cout << "normalvec_g " << normalvec_g << std::endl;
			
			Eigen::Hyperplane<double,3> horizontal_plane = Eigen::Hyperplane<double,3>(normalvec_down, Eigen::VectorXd::Zero(3));

			// Calculation projection of magnetic field onto the horizontal plane
			Eigen::Vector3d normalvec_north = horizontal_plane.projection(normalvec_m);

			if (normalvec_north.norm() / normalvec_m.norm() < 0.5 )
			{
				ROS_WARN_STREAM("Magnetic North is too small in the horizontal plane: " << normalvec_north.norm() / normalvec_m.norm());
				exit(1);
			}
			normalvec_north.normalize();
			//std::cout << "horizontal_plane= " << horizontal_plane.coeffs() << std::endl;

			Eigen::Vector3d normalvec_east = normalvec_down.cross(normalvec_north).normalized();

			
			// rotation matrix R_wi represents the world frame (NED) coordinate in IMU-Frame
			std::cout << "normalvec_north: " << normalvec_north.transpose() << std::endl;
			std::cout << "normalvec_east: " << normalvec_east.transpose() << std::endl;
			std::cout << "normalvec_down: " << normalvec_down.transpose() << std::endl;

			Eigen::Matrix3d R_si, R_is;
			R_si << normalvec_north, normalvec_east, normalvec_down;

			R_wi = R_si * R_sw.transpose();

			R_is = R_si.transpose();
			R_iw = R_sw * R_is;

			
			std::cout << "R_iw = " << std::endl << R_iw << std::endl;

			Eigen::Quaternion<double> q_iw(R_iw);
			q_iw_ = q_iw;
		}
		else{ // use imu internal q
			ROS_WARN("USING IMU INTERNAL QUATERNION ESTIMATION.");
			q_iw_ = R_sw * imuEstimateMean.q_m_;
			R_iw = q_iw_.toRotationMatrix();
			R_wi = R_iw.transpose();
		}
		
		// Initialise world frame in local frame
		q_wv_ = q_iw_.inverse();

		g_ << 0, 0, (R_iw*imuEstimateMean.a_m_)(2) ; // gravity is in z-axis, using ENU coordinate
		std::cout << "g_ = " << std::endl << g_.transpose() << ", a_m_ norm = " << imuEstimateMean.a_m_.norm() << std::endl;

		//////////////////////////////////
		/// Bias Estimation
		//////////////////////////////////

		Eigen::Vector3d g_imu = imuEstimateMean.a_m_.array();

		b_a_ = R_wi * (R_iw * g_imu - g_);	// use estimated bias as the initial state
		// b_a_ << 0, 0, 0;

		b_w_ = imuEstimateMean.w_m_.array(); // use estimated bias as the initial state
		// b_w_ << 0, 0, 0;

		return init();
		// global_start is not yet set
	}

	// should be only called after initStateZero(), otherwise iw and ci calibration are not set
	// void boardcastInitialCalibration()
	// {
	// 	if (!ssf_core_.getGlobalStart().isZero())
	// 	{
	// 		ROS_ERROR("boardcastInitialCalibration() should not be called after a global start.");
	// 		return;
	// 	}
	// 	auto timestamp = ros::Time::now();
	// 	ssf_core_.broadcast_iw_transformation(0, timestamp);
	// 	ssf_core_.broadcast_ci_transformation(0, timestamp);
	// }

	void initialiseIMU(struct ssf_core::ImuInputsCache& imuEstimateMean)
	{	
		ROS_INFO("Waiting for IMU inputs...");

		struct ssf_core::ImuInputsCache* imuCache ;
		int imuCache_size;

		struct AverageVariance{
			struct ssf_core::ImuInputsCache avg;
			struct ssf_core::ImuInputsCache var;
		};
		while (ros::ok())
		{
			std::vector<AverageVariance> result_vec;
			if (ssf_core_.getImuInputsCache(imuCache,imuCache_size) )
			{
				for (int k = 0; k < imuCache_size; k = k + 8)
				{
					struct AverageVariance result = {};
					int i;
					// calculate mean
					for (i = 0; i < 8 && i + k < imuCache_size; i++)
					{
						result.avg.a_m_ += imuCache[i+k].a_m_;
						result.avg.w_m_ += imuCache[i+k].w_m_;
						result.avg.m_m_ += imuCache[i+k].m_m_;
						result.avg.q_m_.coeffs() += imuCache[i+k].q_m_.coeffs();
					}
					result.avg.a_m_ /= i;
					result.avg.w_m_ /= i;
					result.avg.m_m_ /= i;
					result.avg.q_m_.coeffs() /= i;

					result_vec.push_back(result);
				}

				std::cout << "=============IMU Variance Statitics==============" << std::endl;

				struct AverageVariance max_result = {};

				// calculate overall mean
				for (auto result : result_vec)
				{

					max_result.avg.a_m_ += result.avg.a_m_;
					max_result.avg.w_m_ += result.avg.w_m_;
					max_result.avg.m_m_ += result.avg.m_m_;
					max_result.avg.q_m_.coeffs() += result.avg.q_m_.coeffs();

				}
				max_result.avg.a_m_ /= result_vec.size();
				max_result.avg.w_m_ /= result_vec.size();
				max_result.avg.m_m_ /= result_vec.size();
				max_result.avg.q_m_.coeffs() /= result_vec.size();

				// calculate overall variance
				for (auto result : result_vec)
				{
					max_result.var.a_m_ = max_result.var.a_m_.array() + (result.avg.a_m_ - max_result.avg.a_m_).array().pow(2.0);
					max_result.var.w_m_ = max_result.var.w_m_.array() + (result.avg.w_m_ - max_result.avg.w_m_).array().pow(2.0);
					max_result.var.m_m_ = max_result.var.m_m_.array() + (result.avg.m_m_ - max_result.avg.m_m_).array().pow(2.0);
					max_result.var.q_m_.coeffs() = max_result.var.q_m_.coeffs().array() + (result.avg.q_m_.coeffs() - max_result.avg.q_m_.coeffs()).array().pow(2.0);
				}
				max_result.var.a_m_ /= result_vec.size();
				max_result.var.w_m_ /= result_vec.size();
				max_result.var.m_m_ /= result_vec.size();
				max_result.var.q_m_.coeffs() /= result_vec.size();


				std::cout << "avg.a_m_" << max_result.avg.a_m_.transpose();
				std::cout << "\tvar.a_m_" << max_result.var.a_m_.transpose();
				std::cout << std::endl;
				std::cout << "avg.w_m_" << max_result.avg.w_m_.transpose();
				std::cout << "\tvar.w_m_" << max_result.var.w_m_.transpose();
				std::cout << std::endl;
				std::cout << "avg.m_m_" << max_result.avg.m_m_.transpose();
				std::cout << "\tvar.m_m_" << max_result.var.m_m_.transpose();
				std::cout << std::endl;
				std::cout << "avg.q_m_" << max_result.avg.q_m_.coeffs().transpose();
				std::cout << "\tvar.q_m_" << max_result.var.q_m_.coeffs().transpose();
				std::cout << std::endl;

				// can be 0.8, 0.05, 0.002
				if (max_result.var.a_m_.norm() < 0.1 && max_result.var.w_m_.norm() < 0.002 && max_result.var.m_m_.norm() < 0.05 ) // variance smaller than 0.05 m/s^2
				{
					// CHECK FOR GRAVITY ESTIMATE

					if ( fabs( max_result.avg.a_m_.norm() - 9.7760) < 0.5 )
					{
						imuEstimateMean = max_result.avg;
								
						w_m_ = imuEstimateMean.w_m_;
						a_m_ = imuEstimateMean.a_m_;
						m_m_ = imuEstimateMean.m_m_;
						q_m_ = imuEstimateMean.q_m_;
						break;
					}

					ROS_ERROR_STREAM("IMU Gravity estimate deviate too much from reference: 9.7760, retrying: " << max_result.avg.a_m_.norm());
				}
					
			}
			ros::Duration(0.25).sleep();
			ros::spinOnce();
		}

		std::cout << "=============IMU Variance PASS==============" << std::endl;
		
	}

	ros::Time setGlobalStart()
	{
		ros::Time last_imu_time = ssf_core_.getLastImuInputsTime();
		if (last_imu_time.isZero())
			ROS_ERROR("setGlobalStart(): failed to start since IMU data is not yet available");
		else
			ssf_core_.setGlobalStart(last_imu_time);

		return last_imu_time;
	}

private:

	Eigen::Matrix3d R_sw; // transform between sensor's global frame and ekf's global frame

	Eigen::Matrix<double, 3, 1> p_iw_, v_iw_;

	// Calibration
	Eigen::Matrix<double, 3, 1> p_ci_; ///< initial distance camera-IMU
	Eigen::Quaternion<double> q_ci_;   ///< initial rotation camera-IMU
	Eigen::Quaternion<double> q_wv_;   ///< initial rotation wolrd-vision

	// Initial Estimation
	Eigen::Quaternion<double> q_iw_;
	Eigen::Matrix<double, 3, 1> b_w_, b_a_; /// bias gyroscopes, bias accelerometer
	Eigen::Matrix<double, 3, 1> g_;

	// Initial Readings (Mean)
	Eigen::Matrix<double, 3, 1> w_m_, a_m_, m_m_;
	Eigen::Quaternion<double> q_m_;

	double scale_;
        bool use_imu_internal_q;

	ssf_core::SSF_Core::ErrorStateCov P_;

	bool init()
	{
		std::cout << "calculated q_iw_ = " << q_iw_.coeffs().transpose() << std::endl; // x,y,z,w

		auto q_iw_m_ = Eigen::Quaternion<double>(R_sw)*q_m_;
		std::cout << "compared to measured R_sw * q_m_" << q_iw_m_.coeffs().transpose() << std::endl;


		double dev = 180.0/M_PI*std::acos( 2 * std::pow(q_iw_m_.coeffs().dot(q_iw_.coeffs()),2.0) - 1.0 );

		ROS_INFO_STREAM("Deviation Angle = " << dev << "degree");
		if ( std::abs(dev) > 45 ){ // check consistency with IMU's internal state, 10%
			ROS_INFO_STREAM("Deviation Angle too big, try again.");
			return false;
		} 

		if (q_iw_.norm() == 0) // eigen library convention(x,y,z,w)
		{
			ROS_ERROR("q_iw_ is not initialised / estimated properly.");
			return false;
		}

		std::cout << "b_a_ = " << std::endl << b_a_.transpose() << std::endl;
		std::cout << "b_w_ = " << std::endl << b_w_.transpose() << std::endl;

		if (a_m_.isZero() )
		{
			ROS_ERROR("a_m_ is not initialised / estimated properly.");
			return false;
		}

		// initialise world frame to be the first frame of imu frame
		p_iw_.setZero();
		v_iw_.setZero(); // robot velocity (IMU centered)
		
		
		
		/////////////////////////////////////////////
		//// Constructing initial State Variance
		/////////////////////////////////////////////
		P_.setZero(); // error state covariance; if zero, a default initialization in ssf_core is used
		double init_np = 0;
		double init_nv = 0.0;
		double init_nq = 0.05;
		double init_nbw = 0.005;
		double init_nba = 0.1;
		double init_L = 0.0001;
		double init_qwv = 0;
		double init_qci = 0.001;
		double init_pci = 0.01;
		Eigen::Vector3d initvar_p(Eigen::VectorXd::Ones(3)			*	init_np*init_np); // initial position is known
		Eigen::Vector3d initvar_v(Eigen::VectorXd::Ones(3)			*	init_nv*init_nv); // variance of 1 m/s
		Eigen::Vector3d initvar_q_err(Eigen::VectorXd::Ones(3)		*	init_nq*init_nq); // TODO: HOW TO ESTIMATE THIS?
		Eigen::Vector3d initvar_b_w(Eigen::VectorXd::Ones(3)		*	init_nbw*init_nbw); 
		Eigen::Vector3d initvar_b_a(Eigen::VectorXd::Ones(3)		*	init_nba*init_nba);
		double initvar_L = 												init_L*init_L;
		Eigen::Vector3d initvar_q_wv_err(Eigen::VectorXd::Ones(3)	* 	init_qwv*init_qwv);
		Eigen::Vector3d initvar_q_ci_err(Eigen::VectorXd::Ones(3)	*	init_qci*init_qci);
		Eigen::Vector3d initvar_p_ci_(Eigen::VectorXd::Ones(3)		*	init_pci*init_pci);

		Eigen::VectorXd P_diagonal(P_.rows());
		P_diagonal << initvar_p , initvar_v , initvar_q_err , initvar_b_w , initvar_b_a , initvar_L	, initvar_q_wv_err , initvar_q_ci_err , initvar_p_ci_;

		P_ = P_diagonal.asDiagonal();

		std::cout << "P diagonal = " << P_diagonal.transpose() << std::endl;


		ssf_core_.initialize(p_iw_, v_iw_, q_iw_, b_w_, b_a_, scale_, q_wv_, P_, w_m_, a_m_, m_m_, g_, q_ci_, p_ci_);

		ROS_INFO_STREAM("===================filter state[0] initialized to================= \n"
				<< "\tposition: [" << p_iw_[0] << ", " << p_iw_[1] << ", " << p_iw_[2] << "]" << std::endl
				<< "\tscale:" << scale_ << std::endl
				<< "\tattitude (w,x,y,z): [" << q_iw_.w() << ", " << q_iw_.x() << ", " << q_iw_.y() << ", " << q_iw_.z() << "]" << std::endl);
		return true;
	}
};

#endif /* VICONPOSE_MEASUREMENTS_H */
