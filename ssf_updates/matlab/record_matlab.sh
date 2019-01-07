#!/bin/bash

rosbag record /dead_reckoning/pose /ekf_fusion/pose /ekf_fusion/pose_corrected /ekf_fusion/state_out /right/debug_right /rosout_agg /stereo_odometer/pose /stereo_odometer/velocity /stereo_odometer/odometry -O $1
