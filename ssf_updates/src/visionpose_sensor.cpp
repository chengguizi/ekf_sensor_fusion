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

#define N_MEAS 7 /// one artificial constraints, six measurements

int noise_iterator = 1;

VisionPoseSensorHandler::VisionPoseSensorHandler(ssf_core::Measurements* meas) :    // parent class pointer points child class
	MeasurementHandler(meas), lastMeasurementTime_(ros::Time(0))
{
	// read some parameters
	ros::NodeHandle pnh("~");
	pnh.param("measurement_world_sensor", measurement_world_sensor_, true);
	pnh.param("use_fixed_covariance", use_fixed_covariance_, true);

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
	subMeasurement_ = nh.subscribe("visionpose_measurement", 1, &VisionPoseSensorHandler::measurementCallback, this);

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

// void VisionPoseSensorHandler::measurementCallback(const geometry_msgs::PoseStampedConstPtr & msg)
 void VisionPoseSensorHandler::measurementCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr poseMsg)
{

	if (lastMeasurementTime_.isZero())
		ROS_INFO_STREAM("measurementCallback(): First Measuremetn Received at " << poseMsg->header.stamp);

	lastMeasurementTime_ = poseMsg->header.stamp;

	auto global_start = measurements->ssf_core_.getGlobalStart();

	if ( global_start == ros::Time(0) )
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
	z_q_ = Eigen::Quaternion<double>(poseMsg->pose.pose.orientation.w, poseMsg->pose.pose.orientation.x, 
			poseMsg->pose.pose.orientation.y, poseMsg->pose.pose.orientation.z);

	// for magnetometer readings


	// take covariance from sensor
	// fill the measurement covariance matrix R with the covariance values provided in msg
	R.setZero();
	Eigen::Matrix<double, 6, 6> eigen_cov(poseMsg->pose.covariance.data());
	
	R.block<6, 6> (0, 0) = eigen_cov;
	R(6,6) = 1e-6;
	
	
	
	//  alternatively take fix covariance from reconfigure GUI

	/// yx: if no covariance info in sensor msg, then set use_fixed_covariance to true
	// if (use_fixed_covariance_)
	// {
	// 	const double s_zp = n_zp_ * n_zp_;
	// 	const double s_zq = n_zq_ * n_zq_;

	// 	/// yx: small value 1e-6 for artificial constraints
	
	// 	R = (Eigen::Matrix<double, N_MEAS, 1>() << s_zp, s_zp, s_zp, s_zq, s_zq, s_zq, 1e-6).finished().asDiagonal();
	// 	// fill the measurement covariance matrix R using the standard deviations n_zp_ (usually for positions) and n_zq_ (usually for attitudes)
	// }

	// if (!measurement_world_sensor_) // transform the readings and covariance to the sensor reference frame
	// {
	// 	Eigen::Matrix<double, 3, 3> C_zq = z_q_.toRotationMatrix();
	// 	z_q_ = z_q_.conjugate();
	// 	z_p_ = -C_zq.transpose() * z_p_;

	// 	Eigen::Matrix<double, 6, 6> C_cov(Eigen::Matrix<double, 6, 6>::Zero());
	// 	C_cov.block<3, 3> (0, 0) = C_zq;
	// 	C_cov.block<3, 3> (3, 3) = C_zq;

	// 	R.block<6, 6> (0, 0) = C_cov.transpose() * R.block<6, 6> (0, 0) * C_cov;
	// }

	// feedback for init case
	// measurements->p_vc_ = z_p_;	// these values are used as current sensor readings for state initialization
	// measurements->q_cv_ = z_q_;	// these values are used as current sensor readings for state initialization

	// if (!has_measurement)
	// 	{
	// 		ROS_INFO("First Measurement Topic Received!");
	// 		has_measurement = true;
	// 	}

	// find closest predicted state in time which fits the measurement time
	ssf_core::State state_old;
	unsigned char idx = measurements->ssf_core_.getClosestState(&state_old, time_old);
	int num_state = measurements->ssf_core_.getNumberofState();
	
	if (state_old.time_ == -1)
	{
		ROS_WARN_STREAM("measurement callback early abort: no past state with the timestamp found: " 
				<< std::fixed << time_old.toSec() << std::endl);
		ROS_WARN_STREAM("number of states: " << num_state);
		exit(-1); // CRITICAL ERROR
		return; // // early abort // //
	}else{
		auto diff = state_old.time_ - time_old.toSec();
		std::cout << _seq << "th measurement frame found state buffer that is " << diff << " seconds from measurement at index " << (int)idx << std::endl;
	}

	ROS_INFO_STREAM( "\nz_p_" << std::endl << z_p_);
	ROS_INFO_STREAM( "\nz_q_" << std::endl << z_q_.w() << ", " << z_q_.vec());
	ROS_INFO_STREAM( "\nR" << std::endl << R );

	ROS_INFO_STREAM( state_old );

	// get rotation matrices

	// hm: for quaternion, internal storage is [x,y,z,w]; conjugate reverse xyz signs, reversing the rotation effectively
	// by conjugating, the rotation matrix is effectively transposed
	//Eigen::Matrix<double, 3, 3> C_wv = state_old.q_wv_.conjugate().toRotationMatrix(); // hm: C_q{vw}, rotation from world-frame to vision frame
	// Eigen::Matrix<double, 3, 3> C_q = state_old.q_.conjugate().toRotationMatrix(); // hm: C_q{wi}, rotation from inertial-fram to world-frame
	// Eigen::Matrix<double, 3, 3> C_ci = state_old.q_ci_.conjugate().toRotationMatrix(); // hm: C_q{ic}, rotation from camera-frame to inertial-frame


#ifdef DEBUG_ON
	std::cout<< "q_wv_" <<std::endl << state_old.q_wv_.coeffs() << std::endl;
	std::cout<< "q_wv_.conjugate()" <<std::endl << state_old.q_wv_.conjugate().coeffs() << std::endl;
	std::cout<< "C_wv" <<std::endl << C_wv << std::endl;
	std::cout<<std::endl;
#endif


// preprocess for elements in H matrix
	//Eigen::Matrix<double, 3, 1> vecold;
	//vecold = (state_old.p_ + C_q.transpose() * state_old.p_ci_) * state_old.L_;
	//Eigen::Matrix<double, 3, 3> skewold = skew(vecold);

	// Eigen::Matrix<double, 3, 3> pci_sk = skew(state_old.p_ci_);

	// construct H matrix using H-blockx :-)
	H_old.setZero();
	auto _identity3 = Eigen::Matrix<double, 3, 3>::Identity();
	// position:
	H_old.block<3, 3> (0, 0) = _identity3 * state_old.L_; // p
	// H_old.block<3, 3> (0, 6) = - C_q.transpose() * pci_sk * state_old.L_; // q
	H_old.block<3, 1> (0, 15) =  state_old.p_; // C_q.transpose() * state_old.p_ci_ + state_old.p_; // L
	// H_old.block<3, 3> (0, 16) = -C_wv.transpose() * skewold; // q_wv
	// H_old.block<3, 3> (0, 22) =  C_q.transpose() * state_old.L_; //p_ci

	// attitude
	H_old.block<3, 3> (3, 6) = _identity3; //C_ci; // q
	// H_old.block<3, 3> (3, 16) = C_ci * C_q; // q_wv
	// H_old.block<3, 3> (3, 19) = _identity3 ; //q_ci
	// H_old(6, 18) = 1.0; // fix vision world yaw drift because unobservable otherwise (see PhD Thesis)

	// construct residuals
	r_old.setZero();
	// position
	r_old.block<3, 1> (0, 0) = z_p_ - state_old.p_ * state_old.L_;
	// attitude
	Eigen::Quaternion<double> q_err;
	q_err = state_old.q_.conjugate() * z_q_;
	q_err.normalize();

	if (fabs(q_err.w() - 1) > 0.1 && fabs(q_err.w() + 1) > 0.1) // should be a quaternion close to identity
	{
		ROS_FATAL_STREAM("\nq_err is not close to identity: " << q_err.w() << ", " << q_err.vec().transpose());
		exit(1);
	}

	if (q_err.w() > 0)
		r_old.block<3, 1> (3, 0) = q_err.vec() * 2;
	else
		r_old.block<3, 1> (3, 0) = - q_err.vec() * 2;
	// vision world yaw drift
	// q_err = state_old.q_wv_;
	// r_old(6, 0) = -2 * (q_err.w() * q_err.z() + q_err.x() * q_err.y()) / (1 - 2 * (q_err.y() * q_err.y() + q_err.z() * q_err.z()));

	
	ROS_DEBUG_STREAM_THROTTLE(0.5, "H_old" << std::endl << H_old );
	ROS_INFO_STREAM_THROTTLE(0.5, "r_old" << std::endl << r_old );
	
	// call update step in core class
	bool result = measurements->ssf_core_.applyMeasurement(idx, H_old, r_old, R, poseMsg->header);
	if (!result)
		ROS_WARN("Apply Measurement failed (imu not initialised properly?)");

	// broadcast the calibration as well as posterior pose estimate, to external VO.
	measurements->ssf_core_.broadcast_ci_transformation(idx,time_old);
	measurements->ssf_core_.broadcast_iw_transformation(idx,time_old);

	ROS_DEBUG_STREAM("Processed Measurement and broacased ci & iw transforms " << poseMsg->header.seq);
}
