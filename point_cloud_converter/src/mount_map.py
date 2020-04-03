#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointCloud
from math import sqrt, atan2, exp, atan, cos, sin, acos, pi, asin, atan2, fabs, tan
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from time import sleep
import tf
import sys
import roslib
import Image
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
import copy 

global pointcloud_laser, x_laser, y_laser, z_laser

map_name = sys.argv[1]

f_hz = 10
pointcloud_laser = 0

def callback_laser_point(data):
    global pointcloud_laser

    pointcloud_laser = data.points

    return


def teste_odometria():
    global pointcloud_laser, x_laser, y_laser, z_laser

    #subscribers
    sub2 = rospy.Subscriber("/map_cloud", PointCloud, callback_laser_point)

    #iniciando o no
    rospy.init_node('teste_odometria', anonymous=True)

    #rate
    rate = rospy.Rate(f_hz) # 10hz

    #criando um listener
    listener = tf.TransformListener()

    #pontos
    x_laser = []
    y_laser = []
    z_laser = []

    n = 1
        
    
    #loop principal
    while not rospy.is_shutdown():

        if (pointcloud_laser!=0)and(n==1):
            print "Nuvem = ", pointcloud_laser[0]
            lenght =  len(pointcloud_laser)

            for i in range(len(pointcloud_laser)):
                x_laser.append(pointcloud_laser[i].x)
                y_laser.append(pointcloud_laser[i].y)
                z_laser.append(pointcloud_laser[i].z)
            
            # Salvando txt
            x = np.array(x_laser)
            y = np.array(y_laser)
            z = np.array(z_laser)
            # np.savetxt('map_cloud.txt', zip(x, y, z), delimiter=',')
            np.savetxt(map_name, zip(x, y, z), delimiter=',')

            n = 2
        else:
            if n==1:
                print "Nenhum"
            else:
                print "Completo"
        
        #gastando tempo do rate
        rate.sleep()


if __name__ == '__main__':
    try:
        print "Iniciou"
        teste_odometria()
        
    except rospy.ROSInterruptException:
        pass
    