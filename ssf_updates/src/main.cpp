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
	ros::init(argc, argv, "ekf_fusion");
	ros::NodeHandle nh;
	ros::NodeHandle local_nh("~");

	ros::Publisher _reset_pub;
	_reset_pub = nh.advertise<std_msgs::Header>("/reset",3);

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

	ros::AsyncSpinner spinner(2);

	// ROS_INFO("Waiting for reset signal...");
	// auto msgptr = ros::topic::waitForMessage<std_msgs::Header>("/reset",nh);

	// std_msgs::Header header;
	// header.stamp = ros::Time::now();
	// _reset_pub.publish(header);
	// ros::spinOnce();

	ros::Duration(0.5).sleep();
	
	while(ros::ok()){
		spinner.start();
		while (ros::ok() && true){
			// STEP 1, Wait for IMU input to be available, stabilised
			struct ssf_core::ImuInputsCache imuEstimateMean;
			VisionPoseMeas.initialiseIMU(imuEstimateMean);

			// STEP 2, Initialise State Zero with State at origin and fake IMU Input
			if (VisionPoseMeas.initStateZero(imuEstimateMean))
				break;

			ros::Duration(0.5).sleep();
		}

		ros::Time global_start = VisionPoseMeas.setGlobalStart();
		ROS_INFO_STREAM("==============Global Start Time: " << std::fixed <<  global_start <<"==============");

		// Send reset signal
		std_msgs::Header header;
		header.stamp = global_start;
		_reset_pub.publish(header);

		// ROS_WARN_STREAM("Done resetting VO integration, time: " << ros::Time::now());
		
		auto msgptr = ros::topic::waitForMessage<std_msgs::Header>("/reset",nh);
		spinner.stop();
		ROS_INFO("In-process RESET detected, restarting...");
	} // end of ros ok
	

	return 0;
}
