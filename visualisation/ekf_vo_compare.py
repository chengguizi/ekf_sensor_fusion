#!/usr/bin/env python

import rospy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

from std_srvs.srv import SetBool

import subprocess
import time

from threading import Lock

# import numpy as np

from geometry_msgs.msg import PoseWithCovarianceStamped

mutex = Lock()

vox,voy,voz = [], [], []
ekfx, ekfy, ekfz = [], [], []
lines = []

start_time = 0

def VOcallback(msg):
    mutex.acquire()
    #global x,y,z
    VOcallback.counter +=1

    if msg.header.stamp == rospy.Time(0):
        #print "VO invalid... ", VOcallback.counter
        mutex.release()
        return
    vox.append(msg.pose.pose.position.x) # left
    voy.append(msg.pose.pose.position.y) # down
    voz.append(msg.pose.pose.position.z) # front
    #print "VO %d" % VOcallback.counter
    mutex.release()
VOcallback.counter = 0
    

def EKFcallback(msg):
    mutex.acquire()
    global start_time
    EKFcallback.counter +=1

    if EKFcallback.counter == 1:
        start_time = time.time()
        print "start time:", start_time

    ekfx.append(msg.pose.pose.position.x) # north
    ekfy.append(msg.pose.pose.position.y) # east
    ekfz.append(msg.pose.pose.position.z) # down
    mutex.release()
    #print "EKF %d" % EKFcallback.counter
EKFcallback.counter = 0

def update(i):
    mutex.acquire()
    lines[0].set_data(voz,vox)
    lines[0].set_3d_properties(voy)

    lines[1].set_data(ekfx,ekfy)
    lines[1].set_3d_properties(ekfz)
    # ax.relim()
    # ax.autoscale_view(True,True,True,True)
    mutex.release()
    return lines

if __name__ == '__main__':
    
    print "initialising plot"
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    plt.ion()
    plt.show()

    # ax.scatter(x,y,z)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Plot of VO')

    ax.set_autoscale_on(True)
    ax.set_autoscalez_on(True)
    
    ax.set_xlim(-20,20)
    ax.set_ylim(-20,20)
    ax.set_zlim(-20,20)

    lineVO, = ax.plot([],[],[], linestyle='-',marker='o', color='g', markersize=3)
    lineEKF, = ax.plot([],[],[], linestyle='-',marker='x', color='b', markersize=3)

    lines.append(lineVO)
    lines.append(lineEKF)

    rospy.init_node("ekf_vo_compare")
    rospy.Subscriber("/stereo_odometer/pose",PoseWithCovarianceStamped, VOcallback, queue_size=1)
    rospy.Subscriber("/ekf_odometer/pose",PoseWithCovarianceStamped, EKFcallback, queue_size=1)
    print "ros spinning"

    ani = FuncAnimation(fig, update, interval=500)

    #rospy.spin()
    #r = rospy.Rate(10)
    while not rospy.is_shutdown():
        out = raw_input('e or d or s-->')
        setbool = rospy.ServiceProxy('/stereo_odometer/set_ekfout', SetBool)
        if out == 'e':
            setbool(True)
            print (time.time() - start_time),  ":Enable VO out to EKF"
        elif out == 'd':
            print (time.time() - start_time),  ":Disable VO out to EKF"
            setbool(False)
        elif out == 's':
            rospy.signal_shutdown('user request shutdown')

        
        # if keyboard.is_pressed('f'):
        #     print "Toggle VO FAIL Mode"
        #     

    #     fig.canvas.flush_events()
    #     #ax.scatter(x,y,z)
    #     fig.canvas.draw()
        #plt.pause(0.5)
