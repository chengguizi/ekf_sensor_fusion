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

#ifdef POSE_MEAS
#include "pose_measurements.h"
#endif
#ifdef POSITION_MEAS
#include "position_measurements.h"
#endif
#ifdef VICONPOSE_MEAS
#include "viconpose_measurements.h"
#endif
#ifdef VISIONPOSE_MEAS
#include "visionpose_measurements.h"
#endif

#include <std_srvs/Empty.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ekf_odometer");
	ros::NodeHandle nh("~");

#ifdef POSE_MEAS
	PoseMeasurements PoseMeas;
	ROS_INFO_STREAM("Filter type: pose_sensor");
#endif

#ifdef POSITION_MEAS
	PositionMeasurements PositionMeas;
	ROS_INFO_STREAM("Filter type: position_sensor");
#endif

#ifdef VICONPOSE_MEAS
        ViconPoseMeasurements ViconPoseMeas;
        ROS_INFO_STREAM("Filter type: viconpose_sensor");
#endif

#ifdef VISIONPOSE_MEAS
		// STEP 0
        VisionPoseMeasurements VisionPoseMeas;
        ROS_INFO_STREAM("Filter type: visionpose_sensor");
#endif

	//  print published/subscribed topics
	ros::V_string topics;
	ros::this_node::getSubscribedTopics(topics);
	std::string nodeName = ros::this_node::getName();
	std::string topicsStr = nodeName + ":\n\tsubscribed to topics:\n";
	for(unsigned int i=0; i<topics.size(); i++)
		topicsStr+=("\t\t" + topics.at(i) + "\n");

	topicsStr += "\tadvertised topics:\n";
	ros::this_node::getAdvertisedTopics(topics);
	for(unsigned int i=0; i<topics.size(); i++)
		topicsStr+=("\t\t" + topics.at(i) + "\n");

	ROS_INFO_STREAM(""<< topicsStr);

	// STEP 1, Wait for IMU input to be available, stabilised
	struct ssf_core::ImuInputsCache imuEstimateMean;
	VisionPoseMeas.initialiseIMU(imuEstimateMean);

	// STEP 2, Initialise State Zero with State at origin and fake IMU Input
	VisionPoseMeas.initStateZero(imuEstimateMean);

	

	// STEP 3, Wait for VO node to have incoming images and publishing pose topic
	std::string vo_topic_str = "/stereo_odometer/pose";
	while (ros::ok())
	{
		ROS_INFO_STREAM("Waiting for messages from topic " << vo_topic_str);
		auto ptr = ros::topic::waitForMessage<geometry_msgs::PoseWithCovarianceStamped>( vo_topic_str, ros::Duration(0.5) );

		if (!ptr)
			ROS_ERROR_THROTTLE(2,"No messages from the topic, check the VO node availability.");
		else
		{
			ROS_INFO("VO messages are detected, triggering EKF initialisation...");
			break;
		}
		ros::spinOnce();
	}

	// STEP 4, Broadcast Initial calibration
	// VisionPoseMeas.boardcastInitialCalibration();
	// ros::spinOnce();

	// STEP 5, set global start (which checks if IMU inputs has arrived), trigger VO node on global start as well
	ros::Time global_start = VisionPoseMeas.setGlobalStart();
	ROS_INFO("==========Enable IMU Callback for Prediction EKF==============");
	ROS_INFO_STREAM("==============Global Start Time: " << std::fixed <<  global_start <<"==============");
	// Reset VO integration to identity, therefore enable Measurement Callback
	ros::ServiceClient vo_client = nh.serviceClient<std_srvs::Empty>("/stereo_odometer/reset_pose");
	std_srvs::Empty srv;
	if (!vo_client.call(srv))
	{
		ROS_ERROR("Fail to call stereo_odometer/reset_pose");
	}
	ROS_WARN_STREAM("Done resetting VO integration, time: " << ros::Time::now());

	ROS_INFO("main(): initialisation done, ROS spinning");
	ros::spin();

	return 0;
}
