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
from sensor_fusion_comm.msg import DoubleArrayStamped

mutex = Lock()

# for 3D ploting
vox,voy,voz = [], [], []
ekfx, ekfy, ekfz = [], [], []
lines = []

# for bias ploting
timestamp = []
b_ax = []
b_ay = []
b_az = []

q_w_val = []
q_x_val = []
q_y_val = []
q_z_val = []
q_x = []
q_y = []
q_z = []

b_wx = []
b_wy = []
b_wz = []

p_x = []
p_y = []
p_z = []

v_x = []
v_y = []
v_z = []


lines_b_a = []
lines_b_w = []
lines_q = []
lines_q_val = []
lines_p = []
lines_v = []

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

def Statecallback(msg):
    mutex.acquire()
    Statecallback.counter +=1
    timestamp.append(msg.header.stamp.to_sec())

    q_w_val.append(msg.data[6])
    q_x_val.append(msg.data[7])
    q_y_val.append(msg.data[8])
    q_z_val.append(msg.data[9])

    p_x.append(msg.data[28])
    p_y.append(msg.data[29])
    p_z.append(msg.data[30])

    v_x.append(msg.data[31])
    v_y.append(msg.data[32])
    v_z.append(msg.data[33])

    q_x.append(msg.data[34])
    q_y.append(msg.data[35])
    q_z.append(msg.data[36])

    b_wx.append(msg.data[37])
    b_wy.append(msg.data[38])
    b_wz.append(msg.data[39])

    b_ax.append(msg.data[40])
    b_ay.append(msg.data[41])
    b_az.append(msg.data[42])

    mutex.release()
Statecallback.counter = 0

def update(i):
    mutex.acquire()
    lines[0].set_data(voz,vox)
    lines[0].set_3d_properties(voy)

    lines[1].set_data(ekfx,ekfy)
    lines[1].set_3d_properties(ekfz)
    # ax.relim()
    # ax.autoscale_view(True,True,True,True)

    lines_b_a[0].set_data(timestamp,b_ax)
    lines_b_a[1].set_data(timestamp,b_ay)
    lines_b_a[2].set_data(timestamp,b_az)

    lines_b_w[0].set_data(timestamp,b_wx)
    lines_b_w[1].set_data(timestamp,b_wy)
    lines_b_w[2].set_data(timestamp,b_wz)

    lines_q[0].set_data(timestamp,q_x)
    lines_q[1].set_data(timestamp,q_y)
    lines_q[2].set_data(timestamp,q_z)

    lines_q_val[0].set_data(timestamp,q_w_val)
    lines_q_val[1].set_data(timestamp,q_x_val)
    lines_q_val[2].set_data(timestamp,q_y_val)
    lines_q_val[3].set_data(timestamp,q_z_val)

    lines_p[0].set_data(timestamp, p_x)
    lines_p[1].set_data(timestamp, p_y)
    lines_p[2].set_data(timestamp, p_z)

    lines_v[0].set_data(timestamp, v_x)
    lines_v[1].set_data(timestamp, v_y)
    lines_v[2].set_data(timestamp, v_z)


    ax2.relim()
    ax2.autoscale_view(True,True,True)

    ax3.relim()
    ax3.autoscale_view(True,True,True)
    
    ax4.relim()
    ax4.autoscale_view(True,True,True)
    
    ax5.relim()
    ax5.autoscale_view(True,True,True)

    ax6.relim()
    ax6.autoscale_view(True,True,True)
    ax7.relim()
    ax7.autoscale_view(True,True,True)

    mutex.release()
    return lines

if __name__ == '__main__':
    
    print "initialising plot"
    fig = plt.figure(figsize=(18,7))
    ax1 = fig.add_subplot(1,2,1, projection='3d')
    ax2 = fig.add_subplot(2,2,2)
    ax3 = fig.add_subplot(2,2,4)

    ##
    fig_q = plt.figure(figsize=(7,10))
    ax4 = fig_q.add_subplot(4,1,1)
    ax5 = fig_q.add_subplot(4,1,2)
    ax6 = fig_q.add_subplot(4,1,3)
    ax7 = fig_q.add_subplot(4,1,4)

    plt.ion()
    plt.show()

    # ax.scatter(x,y,z)

    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.set_title('3D Plot of VO')

    ax1.set_autoscale_on(True)
    ax1.set_autoscalez_on(True)
    
    ax1.set_xlim(-10,10)
    ax1.set_ylim(-10,10)
    ax1.set_zlim(-10,10)

    lineVO, = ax1.plot([],[],[], linestyle='-',marker='o', color='g', markersize=3)
    lineEKF, = ax1.plot([],[],[], linestyle='-',marker='x', color='b', markersize=3)

    lines.append(lineVO)
    lines.append(lineEKF)

    # ax2 settings
    ax2.set_title('B_a Variance Plot')
    ax2.set_autoscale_on(True)
    ax2.autoscale_view(True, True, True)

    lineB_ax, = ax2.plot([],[], label='b_ax')
    lineB_ay, = ax2.plot([],[], label='b_ay')
    lineB_az, = ax2.plot([],[], label='b_az')
    ax2.legend(handles=[lineB_ax,lineB_ay,lineB_az],loc=3)

    lines_b_a.append(lineB_ax)
    lines_b_a.append(lineB_ay)
    lines_b_a.append(lineB_az)

    # ax3 settings

    ax3.set_title('B_w Variance Plot')
    ax3.set_autoscale_on(True)
    ax3.autoscale_view(True, True, True)

    lineB_wx, = ax3.plot([],[], label='b_wx')
    lineB_wy, = ax3.plot([],[], label='b_wy')
    lineB_wz, = ax3.plot([],[], label='b_wz')
    ax3.legend(handles=[lineB_wx,lineB_wy,lineB_wz],loc=3)

    lines_b_w.append(lineB_wx)
    lines_b_w.append(lineB_wy)
    lines_b_w.append(lineB_wz)

    ### fig_q
    ax4.set_title('q variance')
    ax5.set_title('q value')
    ax4.set_autoscale_on(True)
    ax4.autoscale_view(True, True, True)
    ax5.set_autoscale_on(True)
    ax5.autoscale_view(True, True, True)

    lineQ_x, = ax4.plot([],[], label='q_x')
    lineQ_y, = ax4.plot([],[], label='q_y')
    lineQ_z, = ax4.plot([],[], label='q_z')
    ax4.legend(handles=[lineQ_x,lineQ_y,lineQ_z],loc=3)

    lines_q.append(lineQ_x)
    lines_q.append(lineQ_y)
    lines_q.append(lineQ_z)

    lineQ_w_val, = ax5.plot([],[], label='q_w')
    lineQ_x_val, = ax5.plot([],[], label='q_x')
    lineQ_y_val, = ax5.plot([],[], label='q_y')
    lineQ_z_val, = ax5.plot([],[], label='q_z')
    ax5.legend(handles=[lineQ_w_val,lineQ_x_val,lineQ_y_val,lineQ_z_val],loc=3)

    lines_q_val.append(lineQ_w_val)
    lines_q_val.append(lineQ_x_val)
    lines_q_val.append(lineQ_y_val)
    lines_q_val.append(lineQ_z_val)

    ### fig p and v
    ax6.set_title('P Variance Plot')
    ax7.set_title('V Variance Plot')
    ax6.set_autoscale_on(True)
    ax6.autoscale_view(True, True, True)
    ax7.set_autoscale_on(True)
    ax7.autoscale_view(True, True, True)

    linesP_x, = ax6.plot([],[], label='p_x')
    linesP_y, = ax6.plot([],[], label='p_y')
    linesP_z, = ax6.plot([],[], label='p_z')
    ax6.legend(handles=[linesP_x,linesP_y,linesP_z],loc=3)

    lines_p.append(linesP_x)
    lines_p.append(linesP_y)
    lines_p.append(linesP_z)

    linesV_x, = ax7.plot([],[], label='v_x')
    linesV_y, = ax7.plot([],[], label='v_y')
    linesV_z, = ax7.plot([],[], label='v_z')
    ax7.legend(handles=[linesV_x,linesV_y,linesV_z],loc=3)

    lines_v.append(linesV_x)
    lines_v.append(linesV_y)
    lines_v.append(linesV_z)


    rospy.init_node("ekf_vo_compare")
    rospy.Subscriber("/stereo_odometer/pose",PoseWithCovarianceStamped, VOcallback, queue_size=1)
    

    rospy.Subscriber("/ekf_odometer/pose",PoseWithCovarianceStamped, EKFcallback, queue_size=1)
    rospy.Subscriber("/ekf_odometer/state_out",DoubleArrayStamped, Statecallback, queue_size=1)
    
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
