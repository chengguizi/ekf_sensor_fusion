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

		pnh.param("scale_init", scale_, 1.0);


		ROS_INFO_STREAM("p_ci_: (x,y,z): ["  << p_ci_.x() << ", " << p_ci_.y() << ", " << p_ci_.z()  << "]");
		ROS_INFO_STREAM("q_ci_: (w,x,y,z): [" << q_ci_.w() << ", " << q_ci_.x() << ", " << q_ci_.y() << ", " << q_ci_.z()  << "]");
		ROS_INFO_STREAM("q_wv_: (w,x,y,z): [" << q_wv_.w() << ", " << q_wv_.x() << ", " << q_wv_.y() << ", " << q_wv_.z()  << "]");
		ROS_INFO_STREAM("scale_: " << scale_);
	}

	void initStateZero(struct ssf_core::ImuInputsCache& imuEstimateMean)
	{

		//////////////////////////////////
		/// q_iw estimation
		//////////////////////////////////

		Eigen::Vector3d normalvec_m;
		// if there is no magnetometer input, set it to x direction
		if (imuEstimateMean.m_m_.isZero())
		{
			ROS_WARN("There is no magnetometer readings, assume North is in Z-axis");
			normalvec_m << 0, 0, 1;
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
		g_ << 0, 0, - imuEstimateMean.a_m_.norm() ; // gravity is in z-axis, using NED coordinate

		std::cout << "g_ = " << std::endl << g_.transpose() << std::endl;

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

		Eigen::Matrix3d R_wi,R_iw; 
		// rotation matrix R_wi represents the world frame (NED) coordinate in IMU-Frame
		std::cout << "normalvec_north: " << normalvec_north.transpose() << std::endl;
		std::cout << "normalvec_east: " << normalvec_east.transpose() << std::endl;
		std::cout << "normalvec_down: " << normalvec_down.transpose() << std::endl;

		R_wi << normalvec_north, normalvec_east, normalvec_down;
		R_iw = R_wi.transpose();
		std::cout << "R_iw = " << std::endl << R_iw << std::endl;

		Eigen::Quaternion<double> q_iw(R_iw);
		q_iw_ = q_iw;

		//////////////////////////////////
		/// Bias Estimation
		//////////////////////////////////

		Eigen::Vector3d g_imu = imuEstimateMean.a_m_.array();

		b_a_ = R_wi * (R_iw * g_imu - g_);	
		//b_a_ << 0, 0, 0;

		b_w_ = imuEstimateMean.w_m_.array();
		//b_w_ << 0, 0, 0;

		init();
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
					}
					result.avg.a_m_ /= i;
					result.avg.w_m_ /= i;
					result.avg.m_m_ /= i;

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

				}
				max_result.avg.a_m_ /= result_vec.size();
				max_result.avg.w_m_ /= result_vec.size();
				max_result.avg.m_m_ /= result_vec.size();

				// calculate overall variance
				for (auto result : result_vec)
				{
					max_result.var.a_m_ = max_result.var.a_m_.array() + (result.avg.a_m_ - max_result.avg.a_m_).array().pow(2.0);
					max_result.var.w_m_ = max_result.var.w_m_.array() + (result.avg.w_m_ - max_result.avg.w_m_).array().pow(2.0);
					max_result.var.m_m_ = max_result.var.m_m_.array() + (result.avg.m_m_ - max_result.avg.m_m_).array().pow(2.0);
				}
				max_result.var.a_m_ /= result_vec.size();
				max_result.var.w_m_ /= result_vec.size();
				max_result.var.m_m_ /= result_vec.size();


				std::cout << "avg.a_m_" << max_result.avg.a_m_.transpose();
				std::cout << "\tvar.a_m_" << max_result.var.a_m_.transpose();
				std::cout << std::endl;
				std::cout << "avg.w_m_" << max_result.avg.w_m_.transpose();
				std::cout << "\tvar.w_m_" << max_result.var.w_m_.transpose();
				std::cout << std::endl;

				if (max_result.var.a_m_.norm() < 0.05 && max_result.var.w_m_.norm() < 0.001 ) // variance smaller than 0.05 m/s^2
				{
					// CHECK FOR GRAVITY ESTIMATE

					if ( fabs( max_result.avg.a_m_.norm() - 9.7760) > 0.2 )
					{
						ROS_ERROR_STREAM("IMU Gravity estimate deviate too much from reference: 9.7760, retrying...");
						continue;
					}
					
					imuEstimateMean = max_result.avg;
								
					w_m_ = imuEstimateMean.w_m_;
					a_m_ = imuEstimateMean.a_m_;
					m_m_ = imuEstimateMean.m_m_;
					break;
				}
					
			}
			ros::Duration(1).sleep();
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

	double scale_;

	ssf_core::SSF_Core::ErrorStateCov P_;

	void init()
	{
		std::cout << "q_iw_ = " << std::endl << q_iw_.coeffs() << std::endl; // x,y,z,w
		if (q_iw_.norm() == 0) // eigen library convention(x,y,z,w)
		{
			ROS_ERROR("q_iw_ is not initialised / estimated properly.");
			exit(-1);
			return;
		}

		std::cout << "b_a_ = " << std::endl << b_a_.transpose() << std::endl;
		std::cout << "b_w_ = " << std::endl << b_w_.transpose() << std::endl;

		if (a_m_.isZero() )
		{
			ROS_ERROR("a_m_ is not initialised / estimated properly.");
			exit(-1);
			return;
		}

		// initialise world frame to be the first frame of imu frame
		p_iw_.setZero();
		v_iw_.setZero(); // robot velocity (IMU centered)
		
		
		
		/////////////////////////////////////////////
		//// Constructing initial State Variance
		/////////////////////////////////////////////
		P_.setZero(); // error state covariance; if zero, a default initialization in ssf_core is used
		Eigen::Vector3d initvar_p(Eigen::VectorXd::Zero(3)); // initial position is known
		Eigen::Vector3d initvar_v(Eigen::VectorXd::Ones(3)*1); // variance of 1 m/s
		Eigen::Vector3d initvar_q_err(Eigen::VectorXd::Ones(3)*pow(0.1,2.0) ); // TODO: HOW TO ESTIMATE THIS?
		Eigen::Vector3d initvar_b_w(Eigen::VectorXd::Ones(3)*1*pow(M_PI/6.0,2.0)); 
		Eigen::Vector3d initvar_b_a(Eigen::VectorXd::Ones(3)*1*pow(0.1,2.0));
		double initvar_L = 0;
		Eigen::Vector3d initvar_q_wv_err(Eigen::VectorXd::Zero(3));
		Eigen::Vector3d initvar_q_ci_err(Eigen::VectorXd::Zero(3));
		Eigen::Vector3d initvar_p_ci_(Eigen::VectorXd::Zero(3));

		Eigen::VectorXd P_diagonal(P_.rows());
		P_diagonal << initvar_p , initvar_v , initvar_q_err , initvar_b_w , initvar_b_a , initvar_L	, initvar_q_wv_err , initvar_q_ci_err , initvar_p_ci_;

		P_ = P_diagonal.asDiagonal();

		std::cout << "P diagonal = " << P_diagonal.transpose() << std::endl;


		ssf_core_.initialize(p_iw_, v_iw_, q_iw_, b_w_, b_a_, scale_, q_wv_, P_, w_m_, a_m_, m_m_, g_, q_ci_, p_ci_);

		ROS_INFO_STREAM("===================filter state[0] initialized to================= \n"
				<< "\tposition: [" << p_iw_[0] << ", " << p_iw_[1] << ", " << p_iw_[2] << "]" << std::endl
				<< "\tscale:" << scale_ << std::endl
				<< "\tattitude (w,x,y,z): [" << q_iw_.w() << ", " << q_iw_.x() << ", " << q_iw_.y() << ", " << q_iw_.z() << "]" << std::endl);
	}
};

#endif /* VICONPOSE_MEASUREMENTS_H */
