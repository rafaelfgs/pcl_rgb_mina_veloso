#!/usr/bin/env python

import rospy
import numpy
import sys
import os
from copy import copy
from sensor_msgs.msg import PointCloud
from nav_msgs.msg import Odometry


file_name = "/mnt/WD500/Espeleo/2020_02_ITV/pointcloud_d435i_2.pcd"

freq = 0.5

x0 = []
y0 = []
z0 = []
p0 = []
c0 = []


def callback_odom(data):
    
    global x0_odom, y0_odom, z0_odom
    
    x0_odom = copy(data.pose.pose.position.x)
    y0_odom = copy(data.pose.pose.position.y)
    z0_odom = copy(data.pose.pose.position.z)


def callback_pcl(data):
    
    global t0, x0, y0, z0, p0, c0
    
    t0 = rospy.Time.now()
    
    x0 += [x0_odom]
    y0 += [y0_odom]
    z0 += [z0_odom]
    
    p0 += data.points
    c0 += data.channels[0].values
    
    sys.stdout.write("\rGot PointCloud Message %2d" % len(x0))
    sys.stdout.flush()


def main_function():
    
    global t0
    
    rospy.init_node("pcl_pcd_node", anonymous=True)
    rospy.Subscriber("/points_out", PointCloud, callback_pcl)
    rospy.Subscriber("/t265/odom", Odometry, callback_odom)
    
    if os.path.exists(file_name):
        if raw_input("Output file already exists, do you want to replace it? (y/n): ") == "y":
            os.remove(file_name)
        else:
            sys.exit(0)
    
    sys.stdout.write("\nWaiting for PointCloud Publications...\n\n")
    sys.stdout.flush()
    
    t0 = rospy.Time.now()
    rate = rospy.Rate(10)
    
    while len(x0) == 0 and not rospy.is_shutdown():
        rate.sleep()
    
    while (rospy.Time.to_sec(rospy.Time.now()) - rospy.Time.to_sec(t0)) < 2.0/freq and not rospy.is_shutdown():
        rate.sleep()
    
    sys.stdout.write("\n\n")
    sys.stdout.flush()
    
    m_max = len(x0)
    n_max = len(p0)/len(x0)
    k = 0
    m = 0
    data = []
    
    while m < m_max:
        
        n = 0
        
        while n < n_max and not rospy.is_shutdown():
            
            idx = m*n_max+n
            
            x = x0[m] + p0[idx].z
            y = y0[m] - p0[idx].x
            z = z0[m] - p0[idx].y
            c = c0[idx]
            
            if all(numpy.isfinite([x, y, z, c])):
                data += [[x, y, z, c],]
                k += 1
            
            sys.stdout.write("\rCreating data... %5.1f%%" % (100*(idx+1.0)/(m_max*n_max)))
            sys.stdout.flush()
            
            n += 1
            
        m += 1
        
    points = k
    sys.stdout.write("\n\n%d Points Written\n\n" % points)
    
    f = open(file_name,"a")
    f.write("# .PCD v0.7 - Point Cloud Data file format\n")
    f.write("VERSION 0.7\n")
    f.write("FIELDS x y z rgb\n")
    f.write("SIZE 4 4 4 4\n")
    f.write("TYPE F F F F\n")
    f.write("COUNT 1 1 1 1\n")
    f.write("WIDTH 1\n")
    f.write("HEIGHT %s\n" % points)
    f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
    f.write("POINTS %s\n" % points)
    f.write("DATA ascii")
    
    k = 0
    while not rospy.is_shutdown():
        f.write("\n%s %s %s %s" % (data[k][0],data[k][1],data[k][2],data[k][3]))
        k += 1
        sys.stdout.write("\rCreating data... %5.1f%%" % (100.0*k/points))
        sys.stdout.flush()
    
    sys.stdout.write("\n\nFile Closed\n\n")
    sys.stdout.flush()
    
    f.close()


if __name__ == "__main__":
    try:
        main_function()
    except rospy.ROSInterruptException:
        pass