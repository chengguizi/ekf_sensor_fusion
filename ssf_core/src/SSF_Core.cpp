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

#include <ssf_core/SSF_Core.h>
#include "calcQ.h"
#include <ssf_core/eigen_utils.h>

#include <cassert>

namespace ssf_core
{
SSF_Core::SSF_Core() : imu_received_(0), mag_received_(0)
	, approximate_sync_(ApproximatePolicy(N_STATE_BUFFER),subImu_,subMag_) , global_start_(0) , lastImuInputsTime_(ros::Time(0)), isImuCacheReady(false)
{
	/// ros stuff
	ros::NodeHandle nh_local("~");

	nh_local.param("pose_of_camera_not_imu",_is_pose_of_camera_not_imu, false);

	ROS_WARN_STREAM("Output is set to pose of " << ( _is_pose_of_camera_not_imu ? "CAMERA" : "IMU"));

	pubState_ = nh_local.advertise<sensor_fusion_comm::DoubleArrayStamped> ("state_out", 3);
	//pubCorrect_ = nh.advertise<sensor_fusion_comm::ExtEkf> ("correction", 1);
	pubPose_ = nh_local.advertise<geometry_msgs::PoseWithCovarianceStamped> ("pose", 3);
	pubPose_local_ = nh_local.advertise<geometry_msgs::PoseWithCovarianceStamped> ("pose_local", 3);
	pubPoseCorrected_ = nh_local.advertise<geometry_msgs::PoseWithCovarianceStamped> ("pose_corrected", 3);
	pubIntPose_ = nh_local.advertise<geometry_msgs::PoseWithCovarianceStamped> ("pose_integrated", 3);
	//pubPoseCrtl_ = nh.advertise<sensor_fusion_comm::ExtState> ("ext_state", 1);

	msgState_.data.resize(nFullState_ + N_STATE, 0);

	subImu_.subscribe(nh_local,"imu_state_input", 20);
	subMag_.subscribe(nh_local,"mag_state_input", 20);

	subImu_.registerCallback(boost::bind(SSF_Core::increment, &imu_received_));
	subMag_.registerCallback(boost::bind(SSF_Core::increment, &mag_received_));
	check_synced_timer_ = nh_local.createWallTimer(ros::WallDuration(5.0), boost::bind(&SSF_Core::checkInputsSynchronized, this));

	// exact_sync_.registerCallback(boost::bind(&SSF_Core::imuCallback, this, _1, _2));
	approximate_sync_.registerCallback(boost::bind(&SSF_Core::imuCallback, this, _1, _2));

	qvw_inittimer_ = 1;

	//register dyn config list
	registerCallback(&SSF_Core::DynConfig, this);

	// set call back for reconfigure server should come last, so DynConfig is called for initialisation
	reconfServer_ = new ReconfigureServer(ros::NodeHandle("~"));
	ReconfigureServer::CallbackType f = boost::bind(&SSF_Core::Config, this, _1, _2);
	reconfServer_->setCallback(f);
}

SSF_Core::~SSF_Core()
{
	delete reconfServer_;
}

void SSF_Core::initialize(const Eigen::Matrix<double, 3, 1> & p, const Eigen::Matrix<double, 3, 1> & v,
													const Eigen::Quaternion<double> & q, const Eigen::Matrix<double, 3, 1> & b_w,
													const Eigen::Matrix<double, 3, 1> & b_a, const double & L,
													const Eigen::Quaternion<double> & q_wv, const Eigen::Matrix<double, N_STATE, N_STATE> & P,
													const Eigen::Matrix<double, 3, 1> & w_m, const Eigen::Matrix<double, 3, 1> & a_m,
													const Eigen::Matrix<double, 3, 1> & m_m, const Eigen::Matrix<double, 3, 1> & g, 
													const Eigen::Quaternion<double> & q_ci, const Eigen::Matrix<double, 3, 1> & p_ci)
{

	// init state buffer
	for (int i = 0; i < N_STATE_BUFFER; i++)
	{
		StateBuffer_[i].reset();
	}

	idx_state_ = 0;
	idx_P_ = 0;
	idx_time_ = 0;

	State & state = StateBuffer_[idx_state_];
	state.p_ = p;
	state.v_ = v;
	state.q_ = q;
	initial_q_ = q;
	state.b_w_ = b_w;
	state.b_a_ = b_a;
	state.L_ = L;
	state.q_wv_ = q_wv;
	state.q_ci_ = q_ci;
	state.p_ci_ = p_ci;
	 
	state.q_int_ = q; //state.q_wv_;
	state.p_int_ = p; // this is the pure imu integration, without update
	state.v_int_ = v;

	msgIntPose_.header.seq = 0;
	msgIntPose_.header.stamp = ros::Time(0);
	state.w_m_ = w_m;
	state.a_m_ = a_m;
	state.m_m_ = m_m;

	// constants
	g_ = g;

	global_start_ = ros::Time(0);

	StateBuffer_[idx_P_].P_ = P;

	

	// buffer for vision failure check
	qvw_inittimer_ = 1;
	qbuff_ = Eigen::Matrix<double, nBuff_, 4>::Constant(0);


	ROS_INFO_STREAM("State[" << (int)idx_state_ << "] initialised!");

	// increase state pointers
	idx_state_++;
	idx_P_++;
}

void SSF_Core::imuCallback(const sensor_msgs::ImuConstPtr & msg, const sensor_msgs::MagneticFieldConstPtr & msg_mag)
// void SSF_Core::imuCallback(const ssf_core::visensor_imuConstPtr & msg)
{
	all_received_++;
	static int imuCacheIdx = 0;

	struct ImuInputsCache* imuInputsCache_ptr;

	imuInputsCache_ptr = &imuInputsCache[imuCacheIdx%imuInputsCache_size];
	imuInputsCache_ptr->seq = imuCacheIdx;
	imuInputsCache_ptr->a_m_ << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
	imuInputsCache_ptr->w_m_ << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
	imuInputsCache_ptr->m_m_ << msg_mag->magnetic_field.x, msg_mag->magnetic_field.y, msg_mag->magnetic_field.z;
	imuInputsCache_ptr->q_m_ = Eigen::Quaternion<double>(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z); 

	// make sure the q from IMU measurement is valid
	assert( fabs(imuInputsCache_ptr->q_m_.norm() - 1.0) < 1e-2 );
	imuInputsCache_ptr->q_m_.normalize();

	if (imuCacheIdx == imuInputsCache_size - 1)
		isImuCacheReady = true;

	imuCacheIdx++;

	if (imuInputsCache_ptr->a_m_.norm() > 80)
	{
		ROS_ERROR_STREAM("IMU acceleration input too large: " << imuInputsCache_ptr->a_m_.norm() );
		exit(1);
	}
		

	if (imuInputsCache_ptr->w_m_.norm() > 30)
	{
		ROS_ERROR_STREAM("IMU angular velocity input too large: " << imuInputsCache_ptr->w_m_.norm() );
		exit(1);
	}
		

	////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////

	if (lastImuInputsTime_.isZero())
		ROS_INFO("imuCallback(): First IMU inputs received!");
	
	// keep track of the last input time
	lastImuInputsTime_ = msg->header.stamp;

	if (global_start_.isZero()) // enter calibration mode, not ekf mode yet
	{
		StateBuffer_[0].time_ = msg->header.stamp.toSec();
		ROS_WARN_THROTTLE(1,"IMU data received but global_start_ is yet to be initialised, setting initial timestamp to most recent IMU readings");
		return; // // early abort // //
	}else if (global_start_ > msg->header.stamp)
	{
		ROS_WARN_THROTTLE(1,"IMU data arrives before global start time.");
		return;
	}

	////////////////////////////////////////////////////////////
	///// Mutex start
	////////////////////////////////////////////////////////////

	mutexLock();

	// construct new input state
	StateBuffer_[idx_state_].time_ = msg->header.stamp.toSec();

	// std::cout << "msg->header.stamp = " << msg->header.stamp.toNSec() << ", state = " << (unsigned int)idx_state_ << std::endl;

	// get inputs
	StateBuffer_[idx_state_].a_m_ << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
	StateBuffer_[idx_state_].w_m_ << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
	StateBuffer_[idx_state_].m_m_ << msg_mag->magnetic_field.x, msg_mag->magnetic_field.y, msg_mag->magnetic_field.z;
	StateBuffer_[idx_state_].q_m_ = Eigen::Quaternion<double>(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z); 
	StateBuffer_[idx_state_].q_m_.normalize();
	// DEBUG
	// StateBuffer_[idx_state_].a_m_ = StateBuffer_[(unsigned char)(idx_state_ - 1)].a_m_;
	// StateBuffer_[idx_state_].w_m_ = StateBuffer_[(unsigned char)(idx_state_ - 1)].w_m_;
	// StateBuffer_[idx_state_].m_m_ = StateBuffer_[(unsigned char)(idx_state_ - 1)].m_m_;
	//std::cout << "imuCallback()" << all_received_ << std::endl;

	// remove acc spikes (TODO: find a cleaner way to do this)
	static Eigen::Matrix<double, 3, 1> last_am = Eigen::Matrix<double, 3, 1>(0, 0, 0);
	if (StateBuffer_[idx_state_].a_m_.norm() > 80)
	{
		ROS_ERROR_STREAM("IMU acceleration too large" << StateBuffer_[idx_state_].a_m_.norm() );
		exit(-1);
		StateBuffer_[idx_state_].a_m_ = last_am;
	}
	else
		last_am = StateBuffer_[idx_state_].a_m_;

	static Eigen::Matrix<double, 3, 1> last_wm = Eigen::Matrix<double, 3, 1>(0, 0, 0);
	if (StateBuffer_[idx_state_].w_m_.norm() > 30)
	{
		ROS_ERROR_STREAM("IMU angular velocity too large" << StateBuffer_[idx_state_].w_m_.norm() );
		exit(-1);
		StateBuffer_[idx_state_].w_m_ = last_am;
	}
	else
		last_wm = StateBuffer_[idx_state_].w_m_;


	if (std::abs(StateBuffer_[idx_state_].time_ - StateBuffer_[(unsigned char)(idx_state_ - 1)].time_) > 0.1)
	{
		ROS_ERROR_STREAM("large time-gap detected, resetting previous state to current state time: "
		 << (long long)(StateBuffer_[idx_state_].time_ * 1e9) << ", " << 
		 (long long)(StateBuffer_[(unsigned char)(idx_state_ - 1)].time_ * 1e9) << ", state = " << (unsigned int)idx_state_ << ", abs = " << std::abs(StateBuffer_[idx_state_].time_ - StateBuffer_[(unsigned char)(idx_state_ - 1)].time_) << ", normal = " << StateBuffer_[idx_state_].time_ - StateBuffer_[(unsigned char)(idx_state_ - 1)].time_);
		StateBuffer_[(unsigned char)(idx_state_ - 1)].time_ = StateBuffer_[(idx_state_)].time_ - 1.0e-4; // double has 15 digits of significant figure
		// exit(-1);
	}

	// std::cout  << "dt = " << StateBuffer_[idx_state_].time_ - StateBuffer_[(unsigned char)(idx_state_ - 1)].time_ << std::endl;

	propagateState(StateBuffer_[idx_state_].time_ - StateBuffer_[(unsigned char)(idx_state_ - 1)].time_); 
	// StateBuffer_[idx_state_] = StateBuffer_[(unsigned char)(idx_state_ - 1)];
	// idx_state_++;

	predictProcessCovariance(StateBuffer_[idx_P_].time_ - StateBuffer_[(unsigned char)(idx_P_ - 1)].time_);
	// StateBuffer_[idx_P_].P_ = StateBuffer_[(unsigned char)(idx_P_ - 1)].P_;
	// idx_P_++;
	// HM : from here, both idx_state_ and idx_P_ INCREMENT!
	
	assert(checkForNumeric((double*)(&StateBuffer_[(unsigned char)(idx_state_ - 1)].p_[0]), 3, "prediction p"));

	//predictionMade_ = true;

	msgPose_.header.stamp = msg->header.stamp;
	msgPose_.header.seq = msg->header.seq;

	State &updated_state = StateBuffer_[(unsigned char)(idx_state_ - 1)];

	if (_is_pose_of_camera_not_imu)
	{
		updated_state.toPoseMsg_camera(msgPose_);
		updated_state.toPoseMsg_camera(msgPose_local_, true);
	}
	else
	{
		updated_state.toPoseMsg_imu(msgPose_);
		updated_state.toPoseMsg_imu(msgPose_local_, true);
	}
		

	pubPose_.publish(msgPose_);
	pubPose_local_.publish(msgPose_local_);

	// publish state out
	msgState_.header.stamp = ros::Time().fromSec(updated_state.time_);
	msgState_.header.seq = updated_state.seq_;
	msgState_.delay_measurement = 0; //(msgState_.header.stamp - msg_header.stamp).toSec() ;
	updated_state.toStateMsg(msgState_);
	pubState_.publish(msgState_);

	// publish transforms to help initialising VO
	// broadcast_ci_transformation((unsigned char)(idx_state_ - 1),msgPose_.header.stamp);
	// broadcast_iw_transformation((unsigned char)(idx_state_ - 1),msgPose_.header.stamp);

	// std::cout << updated_state << std::endl;

	// double theta_dev = 180.0/M_PI*std::acos( 2 * std::pow(initial_q_.coeffs().dot(updated_state.q_.coeffs()),2.0) - 1.0 );
	// ROS_INFO_STREAM_THROTTLE(0.5, "angle deviation from the initial q_ (deg): " << theta_dev );
	// 	//  << "P diagonal(): " << std::endl << updated_state.P_.diagonal().transpose() << std::endl);

	// ROS_INFO_STREAM_THROTTLE(0.5, std::endl << "predict v: " << StateBuffer_[(unsigned char)(idx_state_ - 1)].v_.transpose() 
		// << std::endl << "predict p" << StateBuffer_[(unsigned char)(idx_state_ - 1)].p_.transpose() );
	// msgPoseCtrl_.header = msgPose_.header;
	// StateBuffer_[(unsigned char)(idx_state_ - 1)].toExtStateMsg(msgPoseCtrl_);
	//pubPoseCrtl_.publish(msgPoseCtrl_);


	//////////////////////////////////////////////////////////////
	//////// mutex end
	//////////////////////////////////////////////////////////////
	mutexUnlock();
	
}

Eigen::Matrix<double, 4, 4> compute_delta_q(const Eigen::Matrix<double, 3, 1> &ew, const Eigen::Matrix<double, 3, 1> &ewold, double dt){

	typedef const Eigen::Matrix<double, 4, 4> ConstMatrix4;
	// typedef const Eigen::Matrix<double, 3, 1> ConstVector3;
	typedef Eigen::Matrix<double, 4, 4> Matrix4;

	ConstMatrix4 Omega = omegaMatJPL(ew);
	ConstMatrix4 OmegaOld = omegaMatJPL(ewold);

	Eigen::Matrix<double, 3, 1>  ew_avg = (ew + ewold) / 2.0;

	//std::cout<< "ew_avg:" << ew_avg.transpose() << std::endl; 

	// if (ew_avg.norm() < 0.01)
	// 	ew_avg.setZero();
	// else if (ew_avg.norm() < 0.03)
	// 	ew_avg = (ew_avg.norm() - 0.01)/0.03 * ew_avg;


	Matrix4 OmegaMean = omegaMatJPL(ew_avg);

	// zero order quaternion integration
	//	cur_state.q_ = (Eigen::Matrix<double,4,4>::Identity() + 0.5*Omega*dt)*StateBuffer_[(unsigned char)(idx_state_-1)].q_.coeffs();

	// first order quaternion integration, this is kind of costly and may not add a lot to the quality of propagation...
	int div = 1;
	Matrix4 MatExp;
	MatExp.setIdentity();
	OmegaMean *= 0.5 * dt;
	for (int i = 1; i < 5; i++)
	{
		div *= i;
		MatExp = MatExp + OmegaMean / div;
		OmegaMean *= OmegaMean;
	}

	// first oder quat integration matrix
	ConstMatrix4 quat_int = MatExp + 1.0 / 48.0 * (Omega * OmegaOld - OmegaOld * Omega) * dt * dt;

	return quat_int;
} 


void SSF_Core::propagateState(const double dt)
{
	// typedef const Eigen::Matrix<double, 4, 4> ConstMatrix4;
	typedef const Eigen::Matrix<double, 3, 1> ConstVector3;
	// typedef Eigen::Matrix<double, 4, 4> Matrix4;

	// get references to current and previous state
	State & cur_state = StateBuffer_[idx_state_];
	State & prev_state = StateBuffer_[(unsigned char)(idx_state_ - 1)];

	// zero props:
	cur_state.b_w_ = prev_state.b_w_;
	cur_state.b_a_ = prev_state.b_a_;
	cur_state.L_ = prev_state.L_;
	cur_state.q_wv_ = prev_state.q_wv_;
	cur_state.q_ci_ = prev_state.q_ci_;
	cur_state.p_ci_ = prev_state.p_ci_;

//  Eigen::Quaternion<double> dq;
	Eigen::Matrix<double, 3, 1> dv, dv_int, dv_without_g, dv_without_g_int;
	ConstVector3 ew = cur_state.w_m_ - cur_state.b_w_;
	ConstVector3 ewold = prev_state.w_m_ - prev_state.b_w_;
	ConstVector3 ea = cur_state.a_m_ - cur_state.b_a_; // estimated acceleration of current state
	ConstVector3 eaold = prev_state.a_m_ - prev_state.b_a_; // estimated acceleration of previous state


	auto quat_int = compute_delta_q(ew,ewold,dt);

	auto quat_int_int_ = compute_delta_q(cur_state.w_m_, prev_state.w_m_ , dt);

	// first oder quaternion integration
	cur_state.q_.coeffs() = quat_int * prev_state.q_.coeffs();
	cur_state.q_.normalize();

	// OVERRIDE USING IMU'S INTERNAL ATTITUDE INFOMATION!
	// cur_state.q_ = cur_state.q_m_;

	// first oder quaternion integration
	cur_state.q_int_.coeffs() = quat_int_int_ * prev_state.q_int_.coeffs(); // quat_int_int_
	cur_state.q_int_.normalize();
	
	// DEBUG
	// cur_state.q_ = prev_state.q_; 

	// hm: this part shows that C(q_) is a passive transformation from imu to world frame
	dv = (cur_state.q_.toRotationMatrix() * ea + prev_state.q_.toRotationMatrix() * eaold) / 2.0;

	//dv_int = (cur_state.q_int_.toRotationMatrix() * ea + prev_state.q_int_.toRotationMatrix() * eaold) / 2.0;
	dv_int = (cur_state.q_int_.toRotationMatrix() * cur_state.a_m_ + prev_state.q_int_.toRotationMatrix() * prev_state.a_m_) / 2.0;
	
	
	dv_without_g = dv - g_;
	dv_without_g_int = dv_int - g_;

	// for stationary situration, reset acceleration to zero
	// if (  fabs ( dv_without_g.norm() ) < 0.3 && fabs (dv.norm() - g_.norm()) < 0.1 )
	// {
	// 	ROS_WARN_STREAM_THROTTLE(0.25, "Resetting Speed to 90%");
	// 	// dv_without_g.setZero();
	// 	prev_state.v_ = prev_state.v_*( 1 - dt*0.1 );
	// }
		

	// ROS_INFO_STREAM_THROTTLE(1, "\ndv-g based on current: " << (cur_state.q_.toRotationMatrix() * ea  - g_).transpose() << std::endl
	// 	<< "dv-g based on avg (with zero correction): " << dv_without_g.transpose() << std::endl
	//  	<< "v change:" << (dv_without_g * dt).transpose() );

	cur_state.v_ = prev_state.v_ + dv_without_g * dt; // dv is world coordinate accerlation
	cur_state.v_int_ = prev_state.v_int_ + dv_without_g_int * dt;

	// // TO PREVENT DRIFT, TRY MAKING THINGS SMALLER
	// cur_state.v_ = cur_state.v_*(1.0 - dt*1e-1);

	cur_state.p_ = prev_state.p_ + ((cur_state.v_ + prev_state.v_) / 2.0 * dt);
	cur_state.p_int_ = prev_state.p_int_ + ((cur_state.v_int_ + prev_state.v_int_) / 2.0 * dt);


	///// PUBLISH PURE INTEGRATED STATE FOR DEBUG

	ros::Time state_time;
	state_time.fromSec(cur_state.time_);

	if (state_time > msgIntPose_.header.stamp){ // publish new stuff
		msgIntPose_.header.stamp = state_time;
		cur_state.toIntPoseMsg(msgIntPose_);
		pubIntPose_.publish(msgIntPose_);
	}
	


	idx_state_++;  // hm: unsigned char, so will automatically become a ring buffer
}

	
void SSF_Core::predictProcessCovariance(const double dt)
{

	typedef const Eigen::Matrix<double, 3, 3> ConstMatrix3;
	typedef const Eigen::Matrix<double, 3, 1> ConstVector3;
	typedef Eigen::Vector3d Vector3;

	// noises
	ConstVector3 nav = Vector3::Constant(config_.noise_acc /* / sqrt(dt) */);
	ConstVector3 nbav = Vector3::Constant(config_.noise_accbias /* * sqrt(dt) */);

	ConstVector3 nwv = Vector3::Constant(config_.noise_gyr /* / sqrt(dt) */);
	ConstVector3 nbwv = Vector3::Constant(config_.noise_gyrbias /* * sqrt(dt) */);

	ConstVector3 nqwvv = Eigen::Vector3d::Constant(config_.noise_qwv);
	ConstVector3 nqciv = Eigen::Vector3d::Constant(config_.noise_qci);
	ConstVector3 npicv = Eigen::Vector3d::Constant(config_.noise_pic);

	State & cur_state = StateBuffer_[idx_P_];
	State & prev_state = StateBuffer_[(unsigned char)(idx_P_ - 1)];

	// bias corrected IMU readings
	ConstVector3 ew = cur_state.w_m_ - cur_state.b_w_;  // ew: expectation of w, no bias
	ConstVector3 ewold = prev_state.w_m_ - prev_state.b_w_;
	ConstVector3 ew_avg = (ew + ewold) / 2.0;


	ConstVector3 ea = cur_state.a_m_ - cur_state.b_a_; // hm: why not minus away the gravity?
	ConstVector3 eaold = prev_state.a_m_ - prev_state.b_a_; // estimated acceleration of previous state
	ConstVector3 ea_avg = (cur_state.q_.toRotationMatrix() * ea + prev_state.q_.toRotationMatrix() * eaold) / 2.0;
	// HM: FIXED SMALL Z VARIANCE ISSUE ==> WRONG??
	ConstMatrix3 a_sk = skew(ea); // skew(ea_avg - g_); //skew(ea); 
	ConstMatrix3 w_sk = skew(ew_avg); // skew(ew);
	ConstMatrix3 eye3 = Eigen::Matrix<double, 3, 3>::Identity();

	// ConstMatrix3 C_eq = StateBuffer_[idx_P_].q_.toRotationMatrix();
	ConstMatrix3 C_eq = (cur_state.q_.toRotationMatrix() + prev_state.q_.toRotationMatrix()) / 2.0;

	const double dt_p2_2 = dt * dt * 0.5; // dt^2 / 2
	const double dt_p3_6 = dt_p2_2 * dt / 3.0; // dt^3 / 6
	const double dt_p4_24 = dt_p3_6 * dt * 0.25; // dt^4 / 24
	const double dt_p5_120 = dt_p4_24 * dt * 0.2; // dt^5 / 120

	ConstMatrix3 Ca3 = C_eq * a_sk;
	ConstMatrix3 A = Ca3 * (-dt_p2_2 * eye3 + dt_p3_6 * w_sk - dt_p4_24 * w_sk * w_sk);
	ConstMatrix3 B = Ca3 * (dt_p3_6 * eye3 - dt_p4_24 * w_sk + dt_p5_120 * w_sk * w_sk);
	ConstMatrix3 D = -A;
	ConstMatrix3 E = eye3 - dt * w_sk + dt_p2_2 * w_sk * w_sk;
	ConstMatrix3 F = -dt * eye3 + dt_p2_2 * w_sk - dt_p3_6 * (w_sk * w_sk);
	ConstMatrix3 C = Ca3 * F;

	// std::cout << "C_eq: " << std::endl << C_eq  << std::endl;
	// std::cout << "a_sk: " << std::endl <<  a_sk << std::endl;
	// std::cout << "Ca3: " << std::endl << C_eq * a_sk << std::endl;
	// std::cout << "A: " << std::endl << A << std::endl;
	// std::cout << "B: " << std::endl << B << std::endl;
	// std::cout << "D: " << std::endl << D << std::endl;
	// std::cout << "E: " << std::endl << E << std::endl;
	// std::cout << "F: " << std::endl << F << std::endl;
	// std::cout << "C: " << std::endl << C << std::endl;
	// std::cout << "dt * eye3: " << std::endl << dt * eye3 << std::endl;
	// std::cout << "-C_eq * dt_p2_2: " << std::endl << -C_eq * dt_p2_2 << std::endl;
	// std::cout << "-C_eq * dt: " << std::endl << -C_eq * dt << std::endl;


	// discrete error state propagation Matrix Fd according to:
	// Stephan Weiss and Roland Siegwart.
	// Real-Time Metric State Estimation for Modular Vision-Inertial Systems.
	// IEEE International Conference on Robotics and Automation. Shanghai, China, 2011
	Fd_.setIdentity();
	Fd_.block<3, 3> (0, 3) = dt * eye3;
	Fd_.block<3, 3> (0, 6) = A;
	Fd_.block<3, 3> (0, 9) = B;
	Fd_.block<3, 3> (0, 12) = -C_eq * dt_p2_2;

	Fd_.block<3, 3> (3, 6) = C;
	Fd_.block<3, 3> (3, 9) = D;
	Fd_.block<3, 3> (3, 12) = -C_eq * dt;

	Fd_.block<3, 3> (6, 6) = E;
	Fd_.block<3, 3> (6, 9) = F;

	Qd_.setZero(); // hm: reset to zero
	calc_Q(dt, StateBuffer_[idx_P_].q_, ew, ea, nav, nbav, nwv, nbwv, config_.noise_scale, nqwvv, nqciv, npicv, Qd_);

	// ROS_INFO_STREAM("Qd_.diagonal() with dt = " << dt << std::endl << Qd_.diagonal().transpose() );

	// Qd_ = Qd_.diagonal().eval().asDiagonal();

	// ROS_INFO_STREAM("Qd_ with dt = " << dt << std::endl << Qd_);

	StateBuffer_[idx_P_].P_ = Fd_ * StateBuffer_[(unsigned char)(idx_P_ - 1)].P_ * Fd_.transpose() + Qd_;

	// hm: make it symmetrical
	// StateBuffer_[idx_P_].P_  = 0.5 * ( StateBuffer_[idx_P_].P_  + StateBuffer_[idx_P_].P_.transpose());

	idx_P_++;
}


// bool SSF_Core::getStateAtIdx(State* timestate, unsigned char idx)
// {
// 	// if (!predictionMade_)
// 	// {
// 	// 	timestate->time_ = -1;
// 	// 	return false;
// 	// }

// 	*timestate = StateBuffer_[idx];

// 	return true;
// }

ClosestStateStatus SSF_Core::getClosestState(State*& timestate, ros::Time tstamp, double delay, unsigned char &idx)
{  
	// if (!predictionMade_)
	// {
	// 	std::cout  << "getClosestState(): predictionMade_ is FALSE" << std::endl;
	// 	timestate->time_ = -1;
	// 	return false;
	// }

	idx = (unsigned char)(idx_state_ - 1);
	double timedist = 1e100;
	double timenow = tstamp.toSec() - delay - config_.delay; // delay is zero by default


	// vo shouldn't be ahead of imu inputs
	if(StateBuffer_[idx].time_ < timenow){
		ROS_WARN("VO Ahead of IMU");
		return TOO_EARLY;
	}

	while (fabs(timenow - StateBuffer_[idx].time_) < timedist) // timedist decreases continuously until best point reached... then rises again
	{
		timedist = fabs(timenow - StateBuffer_[idx].time_);
		idx--; // hm: beyond 0, will automatically become 255
	}
	if (idx == (unsigned char)(idx_state_ - 1)){
		ROS_WARN( "getClosestState(), buffer overrun, no match possible" );
		return TOO_OLD;
	}
	idx++; // we subtracted one too much before....

	static bool started = false;
	if (idx == 1 && !started)
		idx = 2;
	started = true;

	if (StateBuffer_[idx].time_ == 0)
	{
		ROS_WARN( "getClosestState(): hit time zero, buffer not full yet?" );
		//timestate->time_ = -1; // hm: add to make sure -1 logic condition holds for all failure cases
		return TOO_OLD; // // early abort // //  not enough predictions made yet to apply measurement (too far in past)
	}

	propPToIdx(idx); // catch up with covariance propagation if necessary

	timestate = &(StateBuffer_[idx]);

	return FOUND;
}

void SSF_Core::propPToIdx(unsigned char idx)
{
	// propagate cov matrix until idx
	if (idx<idx_state_ && (idx_P_<=idx || idx_P_>idx_state_))	//need to propagate some covs
		while (idx!=(unsigned char)(idx_P_-1))
			predictProcessCovariance(StateBuffer_[idx_P_].time_-StateBuffer_[(unsigned char)(idx_P_-1)].time_);
}

// HM: idx_delaystate is the index where it is the closest to the given measurement callback timestamp
bool SSF_Core::applyCorrection(unsigned char idx_delaystate, const ErrorState & res_delayed, 
	double fuzzythres, std_msgs::Header msg_header)
{
	if (config_.fixed_scale)
	{
		correction_(15) = 0; //scale
	}

	if (config_.fixed_bias)
	{
		correction_(9) = 0; //acc bias x
		correction_(10) = 0; //acc bias y
		correction_(11) = 0; //acc bias z
		correction_(12) = 0; //gyro bias x
		correction_(13) = 0; //gyro bias y
		correction_(14) = 0; //gyro bias z
	}

	if (config_.fixed_calib)
	{
		correction_(19) = 0; //q_ic roll
		correction_(20) = 0; //q_ic pitch
		correction_(21) = 0; //q_ic yaw
		correction_(22) = 0; //p_ci x
		correction_(23) = 0; //p_ci y
		correction_(24) = 0; //p_ci z
	}

	// assert( !( config_.fixed_scale || config_.fixed_bias || config_.fixed_calib ) );

	ROS_WARN_STREAM("\ncorrection_ " << correction_.transpose() );
	// state update:

	// store old values in case of fuzzy tracking
	// TODO: what to do with attitude? augment measurement noise?

	State & delaystate = StateBuffer_[idx_delaystate];

	// const auto buff_bw = delaystate.b_w_;
	// const auto buff_ba = delaystate.b_a_;
	// const auto buff_L = delaystate.L_;
	// const auto buff_qwv = delaystate.q_wv_;
	// const auto buff_qci = delaystate.q_ci_;
	// const auto buff_pic = delaystate.p_ci_;

	delaystate.p_ = delaystate.p_ + correction_.block<3, 1> (0, 0);
	delaystate.v_ = delaystate.v_ + correction_.block<3, 1> (3, 0);
	if (std::abs((correction_(3, 0) + correction_(4, 0) + correction_(5, 0)) / 3.0 )  > 0.8)
		ROS_WARN_STREAM("Big Velocity Changed Detected: " << (correction_.block<3, 1> (3, 0)).transpose());
	delaystate.b_w_ = delaystate.b_w_ + correction_.block<3, 1> (9, 0);
	delaystate.b_a_ = delaystate.b_a_ + correction_.block<3, 1> (12, 0);
	delaystate.L_ = delaystate.L_ + correction_(15);
	if (delaystate.L_ < 0)
	{
		ROS_WARN_STREAM_THROTTLE(1,"Negative scale detected: " << delaystate.L_ << ". Correcting to 0.1");
		delaystate.L_ = 0.1;
		exit(-1);
	}

	auto qbuff_q = quaternionFromSmallAngle(correction_.block<3, 1> (6, 0));

	// ROS_WARN_STREAM( "qbuff_q: " << qbuff_q.w() << ", " << qbuff_q.vec().transpose() );
	delaystate.q_ = delaystate.q_ * qbuff_q;
	delaystate.q_.normalize();

	auto qbuff_qwv = quaternionFromSmallAngle(correction_.block<3, 1> (16, 0));
	delaystate.q_wv_ = delaystate.q_wv_ * qbuff_qwv;
	delaystate.q_wv_.normalize();

	auto qbuff_qci = quaternionFromSmallAngle(correction_.block<3, 1> (19, 0));
	delaystate.q_ci_ = delaystate.q_ci_ * qbuff_qci;
	delaystate.q_ci_.normalize();

	delaystate.p_ci_ = delaystate.p_ci_ + correction_.block<3, 1> (22, 0);

	// update qbuff_ and check for fuzzy tracking
	// if (qvw_inittimer_ > nBuff_)
	// {
	// 	// should be unit quaternion if no error
	// 	Eigen::Quaternion<double> errq = delaystate.q_wv_.conjugate() *
	// 			Eigen::Quaternion<double>(
	// 					getMedian(qbuff_.block<nBuff_, 1> (0, 3)),
	// 					getMedian(qbuff_.block<nBuff_, 1> (0, 0)),
	// 					getMedian(qbuff_.block<nBuff_, 1> (0, 1)),
	// 					getMedian(qbuff_.block<nBuff_, 1> (0, 2))
	// 					);

	// 	if (std::max(errq.vec().maxCoeff(), -errq.vec().minCoeff()) / fabs(errq.w()) * 2 > fuzzythres) // fuzzy tracking (small angle approx)
	// 	{
	// 		ROS_WARN_STREAM_THROTTLE(1,"fuzzy tracking triggered: " << std::max(errq.vec().maxCoeff(), -errq.vec().minCoeff())/fabs(errq.w())*2 << " limit: " << fuzzythres <<"\n");

	// 		//state_.q_ = buff_q;
	// 		delaystate.b_w_ = buff_bw;
	// 		delaystate.b_a_ = buff_ba;
	// 		delaystate.L_ = buff_L;
	// 		delaystate.q_wv_ = buff_qwv;
	// 		delaystate.q_ci_ = buff_qci;
	// 		delaystate.p_ci_ = buff_pic;
	// 		correction_.block<16, 1> (9, 0) = Eigen::Matrix<double, 16, 1>::Zero();
	// 		qbuff_q.setIdentity();
	// 		qbuff_qwv.setIdentity();
	// 		qbuff_qci.setIdentity();
	// 	}
	// 	else // if tracking ok: update mean and 3sigma of past N q_vw's
	// 	{
	// 		qbuff_.block<1, 4> (qvw_inittimer_ - nBuff_ - 1, 0) = Eigen::Matrix<double, 1, 4>(delaystate.q_wv_.coeffs());
	// 		qvw_inittimer_ = (qvw_inittimer_) % nBuff_ + nBuff_ + 1;
	// 	}
	// }
	// else // at beginning get mean and 3sigma of past N q_vw's
	// {
	// 	qbuff_.block<1, 4> (qvw_inittimer_ - 1, 0) = Eigen::Matrix<double, 1, 4>(delaystate.q_wv_.coeffs());
	// 	qvw_inittimer_++;
	// }

	// idx fiddeling to ensure correct update until now from the past

	assert(idx_state_ != idx_delaystate);
	idx_time_ = idx_state_;
	idx_state_ = idx_delaystate + 1; // reset current state back in time, to be the one after the corrected state
	idx_P_ = idx_delaystate + 1;

	delaystate.seq_ = msg_header.seq;
	// propagate state matrix until now
	while (idx_state_ != idx_time_)
	{
		StateBuffer_[idx_state_].seq_ = msg_header.seq;
		// idx_state_ is current state, idx_state_ - 1 is previous state
		// idx_state_++ is performed after the routine
		propagateState(StateBuffer_[idx_state_].time_ - StateBuffer_[(unsigned char)(idx_state_ - 1)].time_);
	}
		
 
	assert(checkForNumeric(&correction_[0], HLI_EKF_STATE_SIZE, "update"));


	// ROS_WARN_STREAM("applyCorrection(): now at state time = " << (long long)(StateBuffer_[(unsigned char)(idx_state_ - 1)].time_ * 1e9) << ", state = " << (unsigned int)(idx_state_-1));



	// HM: publicise the most accurate estimate, after correction
	msgPoseCorrected_.header.stamp = msg_header.stamp;
	msgPoseCorrected_.header.seq = delaystate.seq_;

	if (_is_pose_of_camera_not_imu)
		delaystate.toPoseMsg_camera(msgPoseCorrected_);
	else
		delaystate.toPoseMsg_imu(msgPoseCorrected_);
	pubPoseCorrected_.publish(msgPoseCorrected_);


	return 1;
}

void SSF_Core::Config(ssf_core::SSF_CoreConfig& config, uint32_t level)
{
	ROS_INFO_STREAM("Config(): dynamic reconfigure detected, level=" << level);
	for (std::vector<CallbackType>::iterator it = callbacks_.begin(); it != callbacks_.end(); it++)
		(*it)(config, level);
}

void SSF_Core::DynConfig(ssf_core::SSF_CoreConfig& config, uint32_t level)
{
	ROS_INFO_STREAM("DynConfig(): config_ updated!"<< std::endl);
	config_ = config;
}

double SSF_Core::getMedian(const Eigen::Matrix<double, nBuff_, 1> & data)
{
	std::vector<double> mediandistvec;
	mediandistvec.reserve(nBuff_);
	for (int i = 0; i < nBuff_; ++i)
		mediandistvec.push_back(data(i));

	if (mediandistvec.size() > 0)
	{
		std::vector<double>::iterator first = mediandistvec.begin();
		std::vector<double>::iterator last = mediandistvec.end();
		std::vector<double>::iterator middle = first + std::floor((last - first) / 2);
		std::nth_element(first, middle, last); // can specify comparator as optional 4th arg
		return *middle;
	}
	else
		return 0;
}

void SSF_Core::broadcast_ci_transformation(const unsigned char idx, const ros::Time& timestamp, bool gotMeasurement)
{
	static bool isPreMeasurement = true;
	static int seq = 0;

	if (gotMeasurement)
	{
		isPreMeasurement = false;
	}else if (!isPreMeasurement)
		return;

	State &state = StateBuffer_[idx];

	geometry_msgs::TransformStamped tf_stamped;
	state.toTransformMsg(tf_stamped,state.p_ci_,state.q_ci_);
	// Eigen::Affine3d affine;
	// affine.translation() = state.p_ci_;
	// affine.linear() = state.q_ci_.toRotationMatrix();

	// geometry_msgs::TransformStamped tf_stamped = tf2::eigenToTransform(affine);
	tf_stamped.header.stamp = timestamp;
	tf_stamped.header.frame_id = "imu_frame";
	tf_stamped.header.seq = seq;
	tf_stamped.child_frame_id = "camera_frame";


	try{
		tf_broadcaster_.sendTransform(tf_stamped);
	}
	catch (tf2::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}

	seq++;

	//// OLD TF PACKAGE
	// static tf::TransformBroadcaster tf_broadcaster_;
	// std::cout<< "broadcasting!!!!!!!" << std::endl;
	// State &state = StateBuffer_[idx];

	// tf::Quaternion q_ci;
	// tf::quaternionEigenToTF(state.q_ci_,q_ci);
	// tf::Vector3 p_ci;
	// tf::vectorEigenToTF(state.p_ci_,p_ci);

	// tf::Transform tf(q_ci, p_ci);

	// try{
	// 	tf_broadcaster_.sendTransform(
	// 	tf::StampedTransform(tf, timestamp, "imu_frame" , "camera_frame")); // parent, child
	// }
	// catch (tf::TransformException ex){
	// 	ROS_ERROR("%s",ex.what());
	// }
	
}

void SSF_Core::broadcast_iw_transformation(const unsigned char idx, const ros::Time& timestamp, bool gotMeasurement)
{
	static bool isPreMeasurement = true;
	static int seq = 0;

	if (gotMeasurement)
	{
		isPreMeasurement = false;
	}else if (!isPreMeasurement)
		return;
	
	State &state = StateBuffer_[idx];

	geometry_msgs::TransformStamped tf_stamped;
	//state.toTransformMsg(tf_stamped,state.p_,state.q_);
	// USE IMU INTERNAL MEASURMENT QUAT
	state.toTransformMsg(tf_stamped,state.p_,state.q_m_);

	tf_stamped.header.stamp = timestamp;
        tf_stamped.header.frame_id = "map";
	tf_stamped.header.seq = seq;
	tf_stamped.child_frame_id = "imu_frame";

	try{
		tf_broadcaster_.sendTransform(tf_stamped);
	}
	catch (tf2::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
	seq++;

}

}; // end namespace
