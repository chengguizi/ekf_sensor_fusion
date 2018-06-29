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

namespace ssf_core
{

SSF_Core::SSF_Core() : global_start_(0), lastImuInputsTime_(ros::Time(0)), isImuCacheReady(false)
{
	/// ros stuff
	ros::NodeHandle nh("ssf_core");
	ros::NodeHandle pnh("~");

	pubState_ = nh.advertise<sensor_fusion_comm::DoubleArrayStamped> ("state_out", 3);
	//pubCorrect_ = nh.advertise<sensor_fusion_comm::ExtEkf> ("correction", 1);
	pubPose_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped> ("pose", 3);
	pubPoseCorrected_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped> ("pose_corrected", 3);
	//pubPoseCrtl_ = nh.advertise<sensor_fusion_comm::ExtState> ("ext_state", 1);

	msgState_.data.resize(nFullState_, 0);

	subImu_ = nh.subscribe("imu_state_input", N_STATE_BUFFER, &SSF_Core::imuCallback, this);
	// subState_ = nh.subscribe("hl_state_input", 1 /*N_STATE_BUFFER*/, &SSF_Core::stateCallback, this);

	//msgCorrect_.state.resize(HLI_EKF_STATE_SIZE, 0);
	//hl_state_buf_.state.resize(HLI_EKF_STATE_SIZE, 0);

	qvw_inittimer_ = 1;

	//pnh.param("data_playback", data_playback_, false);

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
	state.b_w_ = b_w;
	state.b_a_ = b_a;
	state.L_ = L;
	state.q_wv_ = q_wv;
	state.q_ci_ = q_ci;
	state.p_ci_ = p_ci;
	 
	state.q_int_ = q; //state.q_wv_;
	state.p_int_ = p;
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

void SSF_Core::imuCallback(const sensor_msgs::ImuConstPtr & msg)
// void SSF_Core::imuCallback(const ssf_core::visensor_imuConstPtr & msg)
{
	static int imuCacheIdx = 0;

	struct ImuInputsCache* imuInputsCache_ptr;

	imuInputsCache_ptr = &imuInputsCache[imuCacheIdx%imuInputsCache_size];
	imuInputsCache_ptr->seq = imuCacheIdx;
	imuInputsCache_ptr->a_m_ << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
	imuInputsCache_ptr->w_m_ << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;
	imuInputsCache_ptr->m_m_ << 0, 0, 0;
	//imuInputsCache_ptr->m_m_ << msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w ;

	if (imuCacheIdx == imuInputsCache_size - 1)
		isImuCacheReady = true;

	imuCacheIdx++;

	if (imuInputsCache_ptr->a_m_.norm() > 50)
		ROS_ERROR_STREAM("IMU acceleration input too large: " << imuInputsCache_ptr->a_m_.norm() );

	if (imuInputsCache_ptr->w_m_.norm() > 50)
		ROS_ERROR_STREAM("IMU angular velocity input too large: " << imuInputsCache_ptr->w_m_.norm() );

	////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////

	if (lastImuInputsTime_.isZero())
		ROS_INFO("imuCallback(): First IMU inputs received!");
	
	lastImuInputsTime_ = msg->header.stamp;

	if (global_start_.isZero()) // global_start_ will be initialised by imuCallback
	{
		// ROS_ASSERT(idx_state_ == 1);
		StateBuffer_[0].time_ = lastImuInputsTime_.toSec();
		ROS_WARN_THROTTLE(1,"IMU data received but global_start_ is yet to be initialised, setting initial timestamp to most recent IMU readings");
		return; // // early abort // //
	}else if (global_start_ > msg->header.stamp)
	{
		ROS_WARN_THROTTLE(1,"IMU data arrives before global start time.");
		return;
	}

	// construct new input state
	StateBuffer_[idx_state_].time_ = msg->header.stamp.toSec();

	// get inputs
	StateBuffer_[idx_state_].a_m_ << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
	StateBuffer_[idx_state_].w_m_ << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;

	// std::cout << "imu readings :" << std::endl;
	// std::cout << "a: " << StateBuffer_[idx_state_].a_m_ << std::endl;
	// std::cout << "w: " << StateBuffer_[idx_state_].w_m_ << std::endl;

	// remove acc spikes (TODO: find a cleaner way to do this)
	static Eigen::Matrix<double, 3, 1> last_am = Eigen::Matrix<double, 3, 1>(0, 0, 0);
	if (StateBuffer_[idx_state_].a_m_.norm() > 50)
	{
		ROS_ERROR_STREAM("IMU acceleration too large" << StateBuffer_[idx_state_].a_m_.norm() );
		exit(-1);
		StateBuffer_[idx_state_].a_m_ = last_am;
	}
	else
		last_am = StateBuffer_[idx_state_].a_m_;

	static Eigen::Matrix<double, 3, 1> last_wm = Eigen::Matrix<double, 3, 1>(0, 0, 0);
	if (StateBuffer_[idx_state_].w_m_.norm() > 10)
	{
		ROS_ERROR_STREAM("IMU angular velocity too large" << StateBuffer_[idx_state_].w_m_.norm() );
		exit(-1);
		StateBuffer_[idx_state_].w_m_ = last_am;
	}
	else
		last_wm = StateBuffer_[idx_state_].w_m_;


	if (fabs(StateBuffer_[idx_state_].time_ - StateBuffer_[(unsigned char)(idx_state_ - 1)].time_) > 1.0)
	{
		ROS_ERROR_STREAM("large time-gap detected, resetting previous state to current state time.");
		StateBuffer_[(unsigned char)(idx_state_ - 1)].time_ = StateBuffer_[(idx_state_)].time_;
	}

	propagateState(StateBuffer_[idx_state_].time_ - StateBuffer_[(unsigned char)(idx_state_ - 1)].time_);  
	predictProcessCovariance(StateBuffer_[idx_P_].time_ - StateBuffer_[(unsigned char)(idx_P_ - 1)].time_);
	// HM : from here, both idx_state_ and idx_P_ INCREMENT!

	checkForNumeric((double*)(&StateBuffer_[(unsigned char)(idx_state_ - 1)].p_[0]), 3, "prediction p");

	//predictionMade_ = true;

	msgPose_.header.stamp = msg->header.stamp;
	msgPose_.header.seq = msg->header.seq;

	StateBuffer_[(unsigned char)(idx_state_ - 1)].toPoseMsg(msgPose_);
	pubPose_.publish(msgPose_);

	// publish transforms to help initialising VO
	broadcast_ci_transformation((unsigned char)(idx_state_ - 1),msgPose_.header.stamp);
	broadcast_iw_transformation((unsigned char)(idx_state_ - 1),msgPose_.header.stamp);

	// msgPoseCtrl_.header = msgPose_.header;
	// StateBuffer_[(unsigned char)(idx_state_ - 1)].toExtStateMsg(msgPoseCtrl_);
	//pubPoseCrtl_.publish(msgPoseCtrl_);
}


// void SSF_Core::stateCallback(const sensor_fusion_comm::ExtEkfConstPtr & msg)
// {
	// ROS_WARN("in stateCallback()");
	// if (global_start_.isZero())
	// 	return; // // early abort // //

	// StateBuffer_[idx_state_].time_ = msg->header.stamp.toSec();

	// static int seq = 0;

	// // get inputs
	// StateBuffer_[idx_state_].a_m_ << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
	// StateBuffer_[idx_state_].w_m_ << msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z;

	// // remove acc spikes (TODO: find a cleaner way to do this)
	// static Eigen::Matrix<double, 3, 1> last_am = Eigen::Matrix<double, 3, 1>(0, 0, 0);
	// if (StateBuffer_[idx_state_].a_m_.norm() > 50)
	// 	StateBuffer_[idx_state_].a_m_ = last_am;
	// else
	// 	last_am = StateBuffer_[idx_state_].a_m_;

	// if (!predictionMade_)
	// {
	// 	if (fabs(StateBuffer_[(idx_state_)].time_ - StateBuffer_[(unsigned char)(idx_state_ - 1)].time_) > 5)
	// 	{
	// 		ROS_WARN_STREAM_THROTTLE(2, "large time-gap re-initializing to last state\n");
	// 		StateBuffer_[(unsigned char)(idx_state_ - 1)].time_ = StateBuffer_[(idx_state_)].time_;
	// 		StateBuffer_[(unsigned char)(idx_state_)].time_ = 0;
	// 		return; // // early abort // // (if timegap too big)
	// 	}
	// }

	// int32_t flag = msg->flag;
	// if (data_playback_)
	// 	flag = sensor_fusion_comm::ExtEkf::ignore_state;

	// bool isnumeric = true;
	// if (flag == sensor_fusion_comm::ExtEkf::current_state)
	// 	isnumeric = checkForNumeric(&msg->state[0], 10, "before prediction p,v,q");

	// isnumeric = checkForNumeric((double*)(&StateBuffer_[idx_state_].p_[0]), 3, "before prediction p");

	// if (flag == sensor_fusion_comm::ExtEkf::current_state && isnumeric) // state propagation is made externally, so we read the actual state
	// {
	// 	StateBuffer_[idx_state_].p_ = Eigen::Matrix<double, 3, 1>(msg->state[0], msg->state[1], msg->state[2]);
	// 	StateBuffer_[idx_state_].v_ = Eigen::Matrix<double, 3, 1>(msg->state[3], msg->state[4], msg->state[5]);
	// 	StateBuffer_[idx_state_].q_ = Eigen::Quaternion<double>(msg->state[6], msg->state[7], msg->state[8], msg->state[9]);
	// 	StateBuffer_[idx_state_].q_.normalize();

	// 	// zero props:
	// 	StateBuffer_[idx_state_].b_w_ = StateBuffer_[(unsigned char)(idx_state_ - 1)].b_w_;
	// 	StateBuffer_[idx_state_].b_a_ = StateBuffer_[(unsigned char)(idx_state_ - 1)].b_a_;
	// 	StateBuffer_[idx_state_].L_ = StateBuffer_[(unsigned char)(idx_state_ - 1)].L_;
	// 	StateBuffer_[idx_state_].q_wv_ = StateBuffer_[(unsigned char)(idx_state_ - 1)].q_wv_;
	// 	StateBuffer_[idx_state_].q_ci_ = StateBuffer_[(unsigned char)(idx_state_ - 1)].q_ci_;
	// 	StateBuffer_[idx_state_].p_ci_ = StateBuffer_[(unsigned char)(idx_state_ - 1)].p_ci_;
	// 	idx_state_++;

	// 	//hl_state_buf_ = *msg;
	// }
	// else if (flag == sensor_fusion_comm::ExtEkf::ignore_state || !isnumeric) // otherwise let's do the state prop. here
	// 	propagateState(StateBuffer_[idx_state_].time_ - StateBuffer_[(unsigned char)(idx_state_ - 1)].time_);

	// predictProcessCovariance(StateBuffer_[idx_P_].time_ - StateBuffer_[(unsigned char)(idx_P_ - 1)].time_);

	// isnumeric = checkForNumeric((double*)(&StateBuffer_[idx_state_ - 1].p_[0]), 3, "prediction p");
	// isnumeric = checkForNumeric((double*)(&StateBuffer_[idx_state_ - 1].P_(0)), N_STATE * N_STATE, "prediction done P");

	// predictionMade_ = true;

	// msgPose_.header.stamp = msg->header.stamp;
	// msgPose_.header.seq = msg->header.seq;

	// StateBuffer_[(unsigned char)(idx_state_ - 1)].toPoseMsg(msgPose_);
	// pubPose_.publish(msgPose_);

	// // msgPoseCtrl_.header = msgPose_.header;
	// // StateBuffer_[(unsigned char)(idx_state_ - 1)].toExtStateMsg(msgPoseCtrl_);
	// // pubPoseCrtl_.publish(msgPoseCtrl_);

	// seq++;
// }


void SSF_Core::propagateState(const double dt)
{
	typedef const Eigen::Matrix<double, 4, 4> ConstMatrix4;
	typedef const Eigen::Matrix<double, 3, 1> ConstVector3;
	typedef Eigen::Matrix<double, 4, 4> Matrix4;

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
	Eigen::Matrix<double, 3, 1> dv;
	ConstVector3 ew = cur_state.w_m_ - cur_state.b_w_;
	ConstVector3 ewold = prev_state.w_m_ - prev_state.b_w_;
	ConstVector3 ea = cur_state.a_m_ - cur_state.b_a_; // estimated acceleration of current state
	ConstVector3 eaold = prev_state.a_m_ - prev_state.b_a_; // estimated acceleration of previous state
	ConstMatrix4 Omega = omegaMatJPL(ew);
	ConstMatrix4 OmegaOld = omegaMatJPL(ewold);
	Matrix4 OmegaMean = omegaMatJPL((ew + ewold) / 2);

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

	// first oder quaternion integration
	cur_state.q_.coeffs() = quat_int * prev_state.q_.coeffs();
	cur_state.q_.normalize();

	// first oder quaternion integration
	cur_state.q_int_.coeffs() = quat_int * prev_state.q_int_.coeffs();
	cur_state.q_int_.normalize();

	// hm: this part shows that C(q_) is a passive transformation from imu to world frame
	dv = (cur_state.q_.toRotationMatrix() * ea + prev_state.q_.toRotationMatrix() * eaold) / 2;
	cur_state.v_ = prev_state.v_ + (dv - g_) * dt; // dv is world coordinate accerlation
	cur_state.p_ = prev_state.p_ + ((cur_state.v_ + prev_state.v_) / 2 * dt);
	cur_state.p_int_ = prev_state.p_int_ + ((cur_state.v_ + prev_state.v_) / 2 * dt);
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

	// bias corrected IMU readings
	ConstVector3 ew = StateBuffer_[idx_P_].w_m_ - StateBuffer_[idx_P_].b_w_;  // ew: expectation of w, no bias
	ConstVector3 ea = StateBuffer_[idx_P_].a_m_ - StateBuffer_[idx_P_].b_a_;

	ConstMatrix3 a_sk = skew(ea);
	ConstMatrix3 w_sk = skew(ew);
	ConstMatrix3 eye3 = Eigen::Matrix<double, 3, 3>::Identity();

	ConstMatrix3 C_eq = StateBuffer_[idx_P_].q_.toRotationMatrix();

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

	calc_Q(dt, StateBuffer_[idx_P_].q_, ew, ea, nav, nbav, nwv, nbwv, config_.noise_scale, nqwvv, nqciv, npicv, Qd_);
	StateBuffer_[idx_P_].P_ = Fd_ * StateBuffer_[(unsigned char)(idx_P_ - 1)].P_ * Fd_.transpose() + Qd_;

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

unsigned char SSF_Core::getClosestState(State* timestate, ros::Time tstamp, double delay)
{  
	// if (!predictionMade_)
	// {
	// 	std::cout  << "getClosestState(): predictionMade_ is FALSE" << std::endl;
	// 	timestate->time_ = -1;
	// 	return false;
	// }

	unsigned char idx = (unsigned char)(idx_state_ - 1);
	double timedist = 1e100;
	double timenow = tstamp.toSec() - delay - config_.delay; // delay is zero by default

	while (fabs(timenow - StateBuffer_[idx].time_) < timedist) // timedist decreases continuously until best point reached... then rises again
	{
		timedist = fabs(timenow - StateBuffer_[idx].time_);
		idx--; // hm: beyond 0, will automatically become 255
	}
	idx++; // we subtracted one too much before....

	// static bool started = false;
	// if (idx == 1 && !started)
	// 	idx = 2;
	// started = true;

	if (StateBuffer_[idx].time_ == 0)
	{
		std::cout  << "getClosestState(): too far in past" << std::endl;
		timestate->time_ = -1; // hm: add to make sure -1 logic condition holds for all failure cases
		return false; // // early abort // //  not enough predictions made yet to apply measurement (too far in past)
	}

	propPToIdx(idx); // catch up with covariance propagation if necessary

	*timestate = StateBuffer_[idx];

	return idx;
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

	// state update:

	// store old values in case of fuzzy tracking
	// TODO: what to do with attitude? augment measurement noise?

	State & delaystate = StateBuffer_[idx_delaystate];

	const auto buff_bw = delaystate.b_w_;
	const auto buff_ba = delaystate.b_a_;
	const auto buff_L = delaystate.L_;
	const auto buff_qwv = delaystate.q_wv_;
	const auto buff_qci = delaystate.q_ci_;
	const auto buff_pic = delaystate.p_ci_;

	delaystate.p_ = delaystate.p_ + correction_.block<3, 1> (0, 0);
	delaystate.v_ = delaystate.v_ + correction_.block<3, 1> (3, 0);
	delaystate.b_w_ = delaystate.b_w_ + correction_.block<3, 1> (9, 0);
	delaystate.b_a_ = delaystate.b_a_ + correction_.block<3, 1> (12, 0);
	delaystate.L_ = delaystate.L_ + correction_(15);
	if (delaystate.L_ < 0)
	{
		ROS_WARN_STREAM_THROTTLE(1,"Negative scale detected: " << delaystate.L_ << ". Correcting to 0.1");
		delaystate.L_ = 0.1;
	}

	auto qbuff_q = quaternionFromSmallAngle(correction_.block<3, 1> (6, 0));
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
	if (qvw_inittimer_ > nBuff_)
	{
		// should be unit quaternion if no error
		Eigen::Quaternion<double> errq = delaystate.q_wv_.conjugate() *
				Eigen::Quaternion<double>(
						getMedian(qbuff_.block<nBuff_, 1> (0, 3)),
						getMedian(qbuff_.block<nBuff_, 1> (0, 0)),
						getMedian(qbuff_.block<nBuff_, 1> (0, 1)),
						getMedian(qbuff_.block<nBuff_, 1> (0, 2))
						);

		if (std::max(errq.vec().maxCoeff(), -errq.vec().minCoeff()) / fabs(errq.w()) * 2 > fuzzythres) // fuzzy tracking (small angle approx)
		{
			ROS_WARN_STREAM_THROTTLE(1,"fuzzy tracking triggered: " << std::max(errq.vec().maxCoeff(), -errq.vec().minCoeff())/fabs(errq.w())*2 << " limit: " << fuzzythres <<"\n");

			//state_.q_ = buff_q;
			delaystate.b_w_ = buff_bw;
			delaystate.b_a_ = buff_ba;
			delaystate.L_ = buff_L;
			delaystate.q_wv_ = buff_qwv;
			delaystate.q_ci_ = buff_qci;
			delaystate.p_ci_ = buff_pic;
			correction_.block<16, 1> (9, 0) = Eigen::Matrix<double, 16, 1>::Zero();
			qbuff_q.setIdentity();
			qbuff_qwv.setIdentity();
			qbuff_qci.setIdentity();
		}
		else // if tracking ok: update mean and 3sigma of past N q_vw's
		{
			qbuff_.block<1, 4> (qvw_inittimer_ - nBuff_ - 1, 0) = Eigen::Matrix<double, 1, 4>(delaystate.q_wv_.coeffs());
			qvw_inittimer_ = (qvw_inittimer_) % nBuff_ + nBuff_ + 1;
		}
	}
	else // at beginning get mean and 3sigma of past N q_vw's
	{
		qbuff_.block<1, 4> (qvw_inittimer_ - 1, 0) = Eigen::Matrix<double, 1, 4>(delaystate.q_wv_.coeffs());
		qvw_inittimer_++;
	}

	// idx fiddeling to ensure correct update until now from the past
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
		
 
	checkForNumeric(&correction_[0], HLI_EKF_STATE_SIZE, "update");


	// publish state
	const unsigned char idx = (unsigned char)(idx_state_ - 1); // Hm: This is the most recent idx, with IMU

	msgState_.header.stamp = ros::Time().fromSec(StateBuffer_[idx].time_);
	msgState_.header.seq = StateBuffer_[idx].seq_;
	msgState_.delay_measurement = (msgState_.header.stamp - msg_header.stamp).toSec() ;
	StateBuffer_[idx].toStateMsg(msgState_);
	pubState_.publish(msgState_);

	// HM: publicise the most accurate estimate, after correction
	msgPoseCorrected_.header.stamp = msg_header.stamp;
	msgPoseCorrected_.header.seq = delaystate.seq_;

	delaystate.toPoseMsg(msgPoseCorrected_);
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

void SSF_Core::broadcast_ci_transformation(const unsigned char idx, const ros::Time& timestamp)
{
	static int seq = 0;

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

void SSF_Core::broadcast_iw_transformation(const unsigned char idx, const ros::Time& timestamp)
{
	static int seq = 0;
	
	State &state = StateBuffer_[idx];

	geometry_msgs::TransformStamped tf_stamped;
	state.toTransformMsg(tf_stamped,state.p_,state.q_);

	tf_stamped.header.stamp = timestamp;
	tf_stamped.header.frame_id = "world_frame";
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
