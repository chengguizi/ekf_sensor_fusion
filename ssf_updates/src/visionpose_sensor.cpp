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
	pnh.param("measurement_world_sensor", measurement_world_sensor_, true);
	pnh.param("use_fixed_covariance", use_fixed_covariance_, true);

	timer_mag_measure = nh.createTimer(ros::Duration(1), &VisionPoseSensorHandler::magTimerCallback, this);

	ROS_INFO_COND(measurement_world_sensor_, "interpreting measurement as sensor w.r.t. world");
	ROS_INFO_COND(!measurement_world_sensor_, "interpreting measurement as world w.r.t. sensor (e.g. ethzasl_ptam)");

	ROS_INFO_COND(use_fixed_covariance_, "using fixed covariance");
	ROS_INFO_COND(!use_fixed_covariance_, "using covariance from sensor");

	subscribe();

}

void VisionPoseSensorHandler::subscribe()
{
	// has_measurement = false;
	ros::NodeHandle nh("~");
	subMeasurement_ = nh.subscribe("visionpose_measurement", 10, &VisionPoseSensorHandler::measurementCallback, this);

	measurements->ssf_core_.registerCallback(&VisionPoseSensorHandler::noiseConfig, this);

	nh.param("meas_noise1", n_zp_, 0.1);	// default position noise is for ethzasl_ptam
	nh.param("meas_noise2", n_zq_, 0.17);	  // default attitude noise is for ethzasl_ptam

}

void VisionPoseSensorHandler::noiseConfig(ssf_core::SSF_CoreConfig& config, uint32_t level)
{
	//	if(level & ssf_core::SSF_Core_MISC)
	//	{
	this->n_zp_ = config.meas_noise1;
	this->n_zq_ = config.meas_noise2;
	//	}
}

void VisionPoseSensorHandler::magTimerCallback(const ros::TimerEvent& te){

	return; // disable this for now
	auto global_start = measurements->ssf_core_.getGlobalStart();
	if ( global_start == ros::Time(0) )
	{
		ROS_WARN_THROTTLE(1,"Fake Measurement received but global_start is not yet set.");
		return;
	}
	
	if ( global_start > measurements->ssf_core_.getLastImuInputsTime())
	{
		ROS_WARN_STREAM_THROTTLE(1,"Fake Measurement arrives before global start time." << global_start << ", " << lastMeasurementTime_);
		return;
	}

	

	measurements->ssf_core_.mutexLock();
	ROS_WARN("Updating Fake Heading Measurement using IMU Quaternion");






	measurements->ssf_core_.mutexUnlock();
	
}

// void VisionPoseSensorHandler::measurementCallback(const geometry_msgs::PoseStampedConstPtr & msg)
 void VisionPoseSensorHandler::measurementCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr poseMsg)
{

	// if (poseMsg->header.stamp.isZero())
	// {
	// 	ROS_INFO("VO Disabled");
		
	// 	unsigned char idx;
	// 	ssf_core::State state_now = measurements->ssf_core_.getCurrentState(idx);

	// 	ros::Time now;
	// 	now.fromSec(state_now.time_);
	// 	measurements->ssf_core_.broadcast_ci_transformation(idx,now,true);
	// 	measurements->ssf_core_.broadcast_iw_transformation(idx,now,true);
	// 	return;
	// }

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

	//  ROS_INFO_STREAM("translation: [x, y, z] = " << msg->transform.translation.x << ", " << msg->transform.translation.y << ", "  << msg->transform.translation.z << std::endl);
	//  if (msg->header.seq%5!=0)
	//    return;
	// init variables

	// const double mean = 0.0;
	// const double stddev = 1.0;
	// std::default_random_engine generator;
	// std::normal_distribution<double> dist(mean, stddev * 0.002 * noise_iterator);
	// noise_iterator ++;
	
	ros::Time time_old = poseMsg->header.stamp;
	int _seq = poseMsg->header.seq;
	Eigen::Matrix<double, N_MEAS, N_STATE> H_old;
	Eigen::Matrix<double, N_MEAS, 1> r_old;
	Eigen::Matrix<double, N_MEAS, N_MEAS> R;

	// get measurements
	// z_p_ = Eigen::Matrix<double,3,1>(msg->transform.translation.x + dist(generator), msg->transform.translation.y + dist(generator), msg->transform.translation.z+ dist(generator)); /* read sensor information from msg: z_p_ is usually a position */
	// z_q_ = Eigen::Quaternion<double>(msg->transform.rotation.w, msg->transform.rotation.x, msg->transform.rotation.y, msg->transform.rotation.z); /* read sensor information from msg: z_q_ is usually an attitude */
	// z_p_ = Eigen::Matrix<double,3,1>(msg->transform.translation.x, msg->transform.translation.y, msg->transform.translation.z; 
	
	// for visensor pose
	z_p_ = Eigen::Matrix<double,3,1>(poseMsg->pose.pose.position.x, poseMsg->pose.pose.position.y, poseMsg->pose.pose.position.z); 
	// z_q_ = Eigen::Quaternion<double>(poseMsg->pose.pose.orientation.w, poseMsg->pose.pose.orientation.x, 
	// 		poseMsg->pose.pose.orientation.y, poseMsg->pose.pose.orientation.z);


	

	bool isVelocity = poseMsg->header.frame_id == "camera_frame";

	// if (isVelocity)
	// 	ROS_INFO("frame: camera_frame -> isVelocity = true");
	// else
	// 	ROS_INFO_STREAM("frame: " << poseMsg->header.frame_id);

	// take covariance from sensor
	// fill the measurement covariance matrix R with the covariance values provided in msg
	R.setZero();
	Eigen::Matrix<double, 6, 6> eigen_cov(poseMsg->pose.covariance.data());
	
	R.block<3, 3> (0, 0) = eigen_cov.block<3, 3>(0, 0);


	// Preset noise level for q measurement from IMU estimate

	R(3,3) = R(4,4) = R(5,5) = n_zq_;

	//////////////////////////////////////////////////////////////////
	//////// Start mutex
	/////////////////////////////////////////////////////////////////

	measurements->ssf_core_.mutexLock();

	// ROS_INFO_STREAM("Measurement Callback for frame at " << time_old);

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
	
	

	
	// int num_state = measurements->ssf_core_.getNumberofState();

	
	
	// if (state_old_ptr == nullptr)
	// {
	// 	ROS_WARN_STREAM("measurement callback early abort: no past state with the timestamp found: " 
	// 			<< std::fixed << time_old.toSec() << std::endl);
	// 	ROS_WARN_STREAM("number of states: " << num_state);
	// 	exit(-1); // CRITICAL ERROR
	// 	return; // // early abort // //
	// }

	ssf_core::State state_old = *state_old_ptr;
	// auto diff = state_old.time_ - time_old.toSec();
	ros::Time buffer_time;
	buffer_time.fromSec(state_old.time_);
	std::cout << std::endl << std::endl <<
		_seq << "th measurement frame found state buffer at time " << buffer_time << " at index " << (int)idx << std::endl;

	z_q_ = state_old.q_m_; // use IMU's internal q estimate as the FAKE measurement
	Eigen::Matrix3d R_sw;
	R_sw << 0 , 1 , 0,
                1 , 0 , 0,
                0 , 0 , -1;

	z_q_ = R_sw * z_q_;

	{
		double P_v_avg = state_old.P_(3,3) + state_old.P_(4,4) + state_old.P_(5,5) / 3.0;
		double P_q_avg = state_old.P_(6,6) + state_old.P_(7,7) + state_old.P_(8,8) / 3.0;

		ROS_INFO_STREAM_THROTTLE(2,"P_v_avg=" << P_v_avg << ", R=" << R(0,0));
		ROS_INFO_STREAM_THROTTLE(2,"P_q_avg=" << P_q_avg << ", R=" << R(3,3));

		// Make sure the variance are not differ by too much
		if (P_v_avg > R(0,0) * 30)
		{
			ROS_WARN_STREAM("R resets to " << P_v_avg / 30 << " from " << R(0,0) );
			R(0,0) =  R(1,1) =  R(2,2) = P_v_avg / 30;
		}
	}
	
	
	// ROS_INFO_STREAM( "\nz_p_ = " << z_p_.transpose() << std::endl
	// 	<< "z_q_ " << z_q_.w() << ", " << z_q_.vec().transpose()
	// );

	// debug
	// {
	// 	Eigen::Matrix<double, 3, 3> R_ci = state_old.q_ci_.toRotationMatrix();
	// 	Eigen::Matrix<double, 3, 3> R_iw = state_old.q_.toRotationMatrix();
	// 	Eigen::Vector3d world_z_p_ = R_iw*R_ci*z_p_ / state_old.L_ ;

	// 	ROS_INFO_STREAM( "rotated z_p_ = " << world_z_p_.transpose());
	// }
	



	// ROS_WARN_STREAM( "\nR" << std::endl << R.diagonal().transpose() );

	ROS_INFO_STREAM_THROTTLE(2, "state_old " << state_old );

	// ROS_INFO_STREAM( "P_old " << state_old.P_.diagonal().transpose() );



	// double theta_dev = 180.0/M_PI*std::acos( 2 * std::pow(z_q_.coeffs().dot(state_old.q_.coeffs()),2.0) - 1.0 );
	// ROS_WARN_STREAM( "theta_dev between measurement and state q_ (deg): " << theta_dev);

	/////////////////////////////////////
	/////// RESET VELOCITY
	// state_old_ptr->v_.setZero();


	// get rotation matrices

	// hm: for quaternion, internal storage is [x,y,z,w]; conjugate reverse xyz signs, reversing the rotation effectively
	// by conjugating, the rotation matrix is effectively transposed
	// Eigen::Matrix<double, 3, 3> C_wv = state_old.q_wv_.conjugate().toRotationMatrix(); // hm: C_q{vw}, rotation from world-frame to vision frame
	// Eigen::Matrix<double, 3, 3> C_q = state_old.q_.conjugate().toRotationMatrix(); // hm: C_q{wi}, rotation from inertial-fram to world-frame
	// Eigen::Matrix<double, 3, 3> C_ci = state_old.q_ci_.conjugate().toRotationMatrix(); // hm: C_q{ic}, rotation from camera-frame to inertial-frame

// preprocess for elements in H matrix
	//Eigen::Matrix<double, 3, 1> vecold;
	//vecold = (state_old.p_ + C_q.transpose() * state_old.p_ci_) * state_old.L_;
	//Eigen::Matrix<double, 3, 3> skewold = skew(vecold);

	// Eigen::Matrix<double, 3, 3> pci_sk = skew(state_old.p_ci_);

	// construct H matrix using H-blockx :-)
	H_old.setZero();
	auto _identity3 = Eigen::Matrix<double, 3, 3>::Identity();
	// position:

	if (!isVelocity)
	{
		//////////////////////////////////
		// setting zero
		state_old_ptr->v_ << 0,0,0;
		/////////////////////////
		H_old.block<3, 3> (0, 0) = _identity3 * state_old.L_; // p
		// H_old.block<3, 3> (0, 6) = - C_q.transpose() * pci_sk * state_old.L_; // q
		H_old.block<3, 1> (0, 15) =  state_old.p_; // C_q.transpose() * state_old.p_ci_ + state_old.p_; // L
		// H_old.block<3, 3> (0, 16) = -C_wv.transpose() * skewold; // q_wv
		// H_old.block<3, 3> (0, 22) =  C_q.transpose() * state_old.L_; //p_ci
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
	// Eigen::Matrix3d R_ci = state_old.q_ci_.toRotationMatrix();
	// Eigen::Matrix3d R_iw = state_old.q_.toRotationMatrix();
	H_old.block<3, 3> (3, 6) = _identity3; //R_ci.transpose(); //???? _identity3; //C_ci; // q
	// H_old.block<3, 3> (3, 16) = R_iw.transpose(); //R_ci.transpose() *      q_wv
	// H_old.block<3, 3> (3, 19) = _identity3 ; //q_ci
	// H_old(6, 18) = 1.0; // fix vision world yaw drift because unobservable otherwise (see PhD Thesis)

	// construct residuals
	r_old.setZero();
	// position
	if (!isVelocity)
	{
		r_old.block<3, 1> (0, 0) = z_p_ - state_old.p_ * state_old.L_;
	}
	else
	{
		Eigen::Matrix<double, 3, 3> R_ci = state_old.q_ci_.toRotationMatrix();
		Eigen::Matrix<double, 3, 3> R_iw = state_old.q_.toRotationMatrix();
		r_old.block<3, 1> (0, 0) = z_p_ - R_ci.transpose() * R_iw.transpose() * state_old.v_* state_old.L_;

		// if (r_old.block<3, 1> (0, 0).norm() > 0.5)
		// 	ROS_WARN_STREAM( "BIG VELOCITY CHANGE: " << (r_old.block<3, 1>(0, 0).norm()) );
		
		// ROS_INFO_STREAM("v_err between z_p_ - v_ in world (IMU z_q_): " <<  (z_q_.toRotationMatrix() * z_p_ / state_old.L_ - state_old.v_).transpose()  );
		// ROS_INFO_STREAM("v_err between z_p_ - v_ in world (state q_): " <<  (state_old.q_.toRotationMatrix() * z_p_ / state_old.L_ - state_old.v_).transpose()  );
	}

	
	
	// attitude
	Eigen::Quaternion<double> q_err;
	q_err = state_old.q_.conjugate() * z_q_;
	q_err.normalize();

	// ROS_INFO_STREAM("q_err between q_ and z_q_: " << q_err.w() << ", " << q_err.vec().transpose());


	r_old.block<3, 1> (3, 0) = q_err.vec() / q_err.w()  * 2; // may not need  q_err.w()

	// RESET YAW TO ZERO


	// if (q_err.w() > 0.0)
	// 	r_old.block<3, 1> (3, 0) = q_err.vec() * 2;
	// else
	// 	r_old.block<3, 1> (3, 0) = - q_err.vec() * 2;

	// vision world yaw drift
	// q_err = state_old.q_wv_;
	// r_old(6, 0) = -2 * (q_err.w() * q_err.z() + q_err.x() * q_err.y()) / (1 - 2 * (q_err.y() * q_err.y() + q_err.z() * q_err.z()));

	
	// ROS_INFO_STREAM( "H_old" << std::endl << H_old );
	std::cout << "r_old = " << std::endl << r_old.transpose() << std::endl;

	//ROS_INFO_STREAM( "P_old" << std::endl << state_old.P_ );
	
	// call update step in core class

	bool do_update = true;

	if (R(0,0) >= 999)
	{
		ROS_WARN("Big Variance Detected");
		do_update = false;
	}

	// check v difference
	if (r_old.block<3, 1>(0, 0).norm() > 10.0)
	{
		ROS_WARN_STREAM("Big Velocity Difference Detected: " << (r_old.block<3, 1>(0, 0).norm()) );
		do_update = false;
		exit(1);
	}

	
	// check q difference
	// if (fabs(q_err.w() - 1) > 0.2 && fabs(q_err.w() + 1) > 0.2)
	// {
	// 	ROS_FATAL_STREAM("\nq_err is not close to identity: " << q_err.w() << ", " << q_err.vec().transpose());
	// 	ROS_FATAL_STREAM("Measurement variance: " << R(0,0));
	// 	do_update = false;
	// 	exit(1);
	// }

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

		// double check corrected angle

		// double theta_dev_after = 180.0/M_PI*std::acos( 2 * std::pow(z_q_.coeffs().dot(state_old_ptr->q_.coeffs()),2.0) - 1.0 );
		// ROS_WARN_STREAM( "after correction theta_dev (deg): " << theta_dev_after);
		// ROS_WARN_STREAM( "theta after: " << state_old_ptr->q_.w() << ", " << state_old_ptr->q_.vec().transpose());

		// if (fabs(theta_dev_after) > 4  && theta_dev_after / theta_dev > 5)
		// {
		// 	ROS_WARN("BAD Variance of theta (q)");
		// 	exit(1);
		// }
		
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
