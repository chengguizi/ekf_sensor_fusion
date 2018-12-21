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

#include "visionpose_sensor.h"
#include <ssf_core/eigen_utils.h>
// for adding gaussian noise
#include <iostream>
#include <iterator>
#include <random>

#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Geometry>

//#define DEBUG_ON

#define N_MEAS 6 /// one artificial constraints, six measurements

int noise_iterator = 1;

VisionPoseSensorHandler::VisionPoseSensorHandler(ssf_core::Measurements* meas) :    // parent class pointer points child class
	MeasurementHandler(meas), lastMeasurementTime_(ros::Time(0))
{
	// read some parameters
	ros::NodeHandle pnh("~");
	ros::NodeHandle nh;
	pnh.param("use_fixed_covariance", use_fixed_covariance_, true);
	pnh.param("velocity_measurement", velocity_measurement_, true);

	pnh.param("max_state_measurement_variance_ratio", max_state_measurement_variance_ratio_, 30);

	pnh.param("sigma_distance_scale", sigma_distance_scale, 10.0);


	// Obtain transformation between sensor global frame and ekf global frame (ENU)
	Eigen::Quaternion<double> q_sw_;
        pnh.getParam("init/q_sw/w", q_sw_.w());
        pnh.getParam("init/q_sw/x", q_sw_.x());
        pnh.getParam("init/q_sw/y", q_sw_.y());
        pnh.getParam("init/q_sw/z", q_sw_.z());

        ROS_INFO_STREAM(" q_sw_ " << q_sw_.w() << ", " << q_sw_.vec().transpose());

	q_sw_.normalize();

	R_sw = q_sw_.toRotationMatrix();

	ROS_WARN_COND(use_fixed_covariance_, "using fixed covariance");
	ROS_WARN_COND(!use_fixed_covariance_, "using covariance from sensor");

	ROS_WARN_COND(velocity_measurement_, "using VO as velocity sensor");
	ROS_WARN_COND(!velocity_measurement_, "using VO as delta pose sensor");

	subscribe();

}

void VisionPoseSensorHandler::subscribe()
{
	// has_measurement = false;
	ros::NodeHandle nh("~");
	subMeasurement_ = nh.subscribe("visionpose_measurement", 10, &VisionPoseSensorHandler::measurementCallback, this);

	measurements->ssf_core_.registerCallback(&VisionPoseSensorHandler::noiseConfig, this);

	nh.param("meas_noise1", n_zv_, 0.1);	// default position noise is for ethzasl_ptam
	nh.param("meas_noise2", n_zq_, 0.17);	  // default attitude noise is for ethzasl_ptam

}

void VisionPoseSensorHandler::noiseConfig(ssf_core::SSF_CoreConfig& config, uint32_t level)
{
	//	if(level & ssf_core::SSF_Core_MISC)
	//	{
	this->n_zv_ = config.meas_noise1;
	this->n_zq_ = config.meas_noise2;
	//	}
}

// void VisionPoseSensorHandler::measurementCallback(const geometry_msgs::PoseStampedConstPtr & msg)
 void VisionPoseSensorHandler::measurementCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr poseMsg)
{



	if (lastMeasurementTime_.isZero())
		ROS_INFO_STREAM("measurementCallback(): First Measurement Received at " << poseMsg->header.stamp);

	lastMeasurementTime_ = poseMsg->header.stamp;

	auto global_start = measurements->ssf_core_.getGlobalStart();

	if ( global_start.isZero() )
	{
		ROS_WARN_THROTTLE(1,"Measurement received but global_start is not yet set.");
		return;
	}
	
	if ( global_start > lastMeasurementTime_)
	{
		ROS_WARN_THROTTLE(1,"Measurement arrives before global start time.");
		return;
	}
	
	ros::Time time_old = poseMsg->header.stamp;
	int _seq = poseMsg->header.seq;
	Eigen::Matrix<double, N_MEAS, N_STATE> H_old;
	Eigen::Matrix<double, N_MEAS, 1> r_old;
	Eigen::Matrix<double, N_MEAS, N_MEAS> R;


	z_v_ = Eigen::Matrix<double,3,1>(poseMsg->pose.pose.position.x, poseMsg->pose.pose.position.y, poseMsg->pose.pose.position.z); 

	R.setZero();
	if (use_fixed_covariance_)
		R(0,0) = R(1,1) = R(2,2) = n_zv_;
	else
	{
		Eigen::Matrix<double, 6, 6> eigen_cov(poseMsg->pose.covariance.data());
		R.block<3, 3> (0, 0) = eigen_cov.block<3, 3>(0, 0);
	}
		

	// Preset noise level for q measurement from IMU estimate
	R(3,3) = R(4,4) = R(5,5) = n_zq_;

	//////////////////////////////////////////////////////////////////
	//////// Start mutex
	/////////////////////////////////////////////////////////////////

	measurements->ssf_core_.mutexLock();

	std::cout << "Measurement Callback for frame at " << time_old << std::endl;

	// find closest predicted state in time which fits the measurement time
	ssf_core::State* state_old_ptr = nullptr;
	unsigned char idx;

	// A LOOP TO TRY UNTIL VO IS NOT TOO EARLY
	{
		ssf_core::ClosestStateStatus ret = ssf_core::TOO_EARLY;

		while(ret == ssf_core::TOO_EARLY && ros::ok()){
			ret = measurements->ssf_core_.getClosestState(state_old_ptr, time_old,0.0, idx);

			if (ret == ssf_core::TOO_EARLY){
				measurements->ssf_core_.mutexUnlock();
				ROS_INFO("Wait 200ms as VO is too fast");
				ros::Duration(0.2).sleep();
				measurements->ssf_core_.mutexLock();
			}
				
		}

		if (ret != ssf_core::FOUND){
		ROS_WARN("finding Closest State not possible, reject measurement");
		measurements->ssf_core_.mutexUnlock();
		return;
	}

	}
	

	ssf_core::State state_old = *state_old_ptr;
	ros::Time buffer_time;
	buffer_time.fromSec(state_old.time_);
	std::cout << std::endl << std::endl <<
		_seq << "th measurement frame found state buffer at time " << buffer_time << " at index " << (int)idx << std::endl;

	z_q_ = R_sw * state_old.q_m_; // use IMU's internal q estimate as the FAKE measurement

	double P_v_avg = state_old.P_(3,3) + state_old.P_(4,4) + state_old.P_(5,5) / 3.0;
	double P_q_avg = state_old.P_(6,6) + state_old.P_(7,7) + state_old.P_(8,8) / 3.0;

	ROS_INFO_STREAM_THROTTLE(15,"P_v_avg=" << P_v_avg << ", R=" << R(0,0));
	ROS_INFO_STREAM_THROTTLE(15,"P_q_avg=" << P_q_avg << ", R=" << R(3,3));

	// Make sure the variance are not differ by too much
	if (P_v_avg > R(0,0) * max_state_measurement_variance_ratio_)
	{
		ROS_WARN_STREAM("R resets to " << P_v_avg / 30 << " from " << R(0,0) );
		R(0,0) =  R(1,1) =  R(2,2) = P_v_avg / 30;
	}

	


	ROS_INFO_STREAM_THROTTLE(10, "state_old " << state_old );

	std::cout << "state_old " << state_old << std::endl;


	H_old.setZero();
	auto _identity3 = Eigen::Matrix<double, 3, 3>::Identity();
	// position:

	if (!velocity_measurement_)
	{
		//////////////////////////////////
		// setting zero
		state_old_ptr->v_ << 0,0,0;
		/////////////////////////
		H_old.block<3, 3> (0, 0) = _identity3 * state_old.L_; // p
		H_old.block<3, 1> (0, 15) =  state_old.p_; // L
	}else{
		Eigen::Matrix<double, 3, 3> R_ci = state_old.q_ci_.toRotationMatrix();
		Eigen::Matrix<double, 3, 3> R_iw = state_old.q_.toRotationMatrix();
		Eigen::Matrix<double, 3, 3> v_skew = skew( R_iw.transpose() * state_old.v_);

		H_old.block<3, 3> (0, 3) = R_ci.transpose() * R_iw.transpose() * state_old.L_; //  partial v_iw
		H_old.block<3, 3> (0, 6) = R_ci.transpose() * v_skew * state_old.L_; // partial theta_iw
		H_old.block<3, 1> (0, 15) = R_ci.transpose() * R_iw.transpose() * state_old.v_; // partial lambda
		H_old.block<3, 3> (0, 19) = skew(R_ci.transpose() * R_iw.transpose()* state_old.v_) * state_old.L_;
	}
	

	// attitude
	H_old.block<3, 3> (3, 6) = _identity3;  // q

	// position
	if (!velocity_measurement_)
	{
		r_old.block<3, 1> (0, 0) = z_v_ - state_old.p_ * state_old.L_;
	}
	else
	{
		Eigen::Matrix<double, 3, 3> R_ci = state_old.q_ci_.toRotationMatrix();
		Eigen::Matrix<double, 3, 3> R_iw = state_old.q_.toRotationMatrix();
		r_old.block<3, 1> (0, 0) = z_v_ - R_ci.transpose() * R_iw.transpose() * state_old.v_* state_old.L_;
	}
	
	// attitude
	Eigen::Quaternion<double> q_err;
	q_err = state_old.q_.conjugate() * z_q_;
	q_err.normalize();


	r_old.block<3, 1> (3, 0) = q_err.vec() / q_err.w()  * 2; // may not need  q_err.w()

	if (velocity_measurement_)
		std::cout << "z_v_ = " << std::endl << z_v_.transpose() << std::endl;
	else	
		std::cout << "z_p_ = " << std::endl << z_v_.transpose() << std::endl;

	std::cout << "z_q_ = " << std::endl << z_q_.w() << ", " << z_q_.vec().transpose() << std::endl;
	std::cout << "r_old = " << std::endl << r_old.transpose() << std::endl;
	
	// call update step in core class

	bool do_update = true;

	if (R(0,0) >= 999)
	{
		ROS_WARN("Big Variance Detected");
		do_update = false;
	}

	// check for velocity measurement outliers, using variance

	double velocity_err_distance = r_old.block<3, 1>(0, 0).norm();
	double sigma_distance = sigma_distance_scale * ( R(0,0) + state_old.P_.diagonal().block<3,1>(3,0).norm() );

	ROS_INFO_STREAM_THROTTLE(2, "vel error = " << velocity_err_distance << ", " << sigma_distance_scale << "sigma distance = " << sigma_distance);
	if (velocity_err_distance > sigma_distance)
	{
		ROS_WARN_STREAM("Big Velocity Difference Detected: " << velocity_err_distance << ", compared to sigma-distance: " << sigma_distance );
		do_update = false;
	}

	if ( (state_old.v_ - state_old_ptr->v_).norm() > 3 )
	{
		ROS_WARN("BAD Velocity (v_), skipping update");
		do_update = false;
	}


	if (do_update)
	{
		bool result = measurements->ssf_core_.applyMeasurement(idx, H_old, r_old, R, poseMsg->header);
		if (!result)
			ROS_WARN("Apply Measurement failed (imu not initialised properly?)");
		
	}
	else{
		ROS_WARN("Apply Measurement SKIPED");
	}

	
	

	// broadcast the calibration as well as posterior pose estimate, to external VO.
	measurements->ssf_core_.broadcast_ci_transformation(idx,time_old,true);
	measurements->ssf_core_.broadcast_iw_transformation(idx,time_old,true);

	//ROS_DEBUG_STREAM("Processed Measurement and broacased ci & iw transforms " << poseMsg->header.seq);

	///////////////////////////////////////////////////////////////
	//////// mutext unlock
	///////////////////////////////////////////////////////////////

	measurements->ssf_core_.mutexUnlock();
}
