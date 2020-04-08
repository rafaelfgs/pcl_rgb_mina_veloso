#!/usr/bin/env python

import rospy
import rosbag
import numpy
import sys
import os
from math import pi
from copy import copy
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from tf.transformations import quaternion_from_euler

input_file = ['/mnt/WD500/Espeleo/2020_02_ITV/2020-02-21-17-07-46_0.bag',
              '/mnt/WD500/Espeleo/2020_02_ITV/2020-02-21-17-08-16_1.bag',
              '/mnt/WD500/Espeleo/2020_02_ITV/2020-02-21-17-08-46_2.bag']

output_file = '/mnt/WD500/Espeleo/2020_02_ITV/200221_170746_depthcloud.bag'

match_bags = False

depth_shift_time = 1.7

tf_xsens = [0, 0, 0.085]
tf_os1 = [0, 0, 0.19]
tf_t265 = [0.07, 0, 0.16]
tf_d435i = [0.07, 0, 0.16]
#eu_d435i = [pi/180*0, pi/180*2.5, pi/180*0.5]
eu_d435i = [pi/180*0, pi/180*5, pi/180*0]

qt_d435i = quaternion_from_euler(eu_d435i[0],eu_d435i[1],eu_d435i[2])

tf_args = numpy.array([
                       [0                 , 0                 , 0                 , 0                 , 0                 , 0                 , 1                 , 'world'                               , 'base_init'                           , 400 ],
                       [0                 , 0                 , 0.15              , 0                 , 0                 , 0                 , 1                 , 'base_init'                           , 'chassis_init'                        , 400 ],
#                       [0                 , 0                 , 0                 , 0                 , 0                 , 0                 , 1                 , 'chassis_init'                        , 'wheel_imu_init'                      , 10  ],
#                       [0                 , 0                 , 0                 , 0                 , 0                 , 0                 , 1                 , 'chassis_init'                        , 'wheel_init'                          , 10  ],
                       [tf_t265[0]        , tf_t265[1]        , tf_t265[2]        , 0                 , 0                 , 0                 , 1                 , 'chassis_init'                        , 't265_init'                           , 400 ],
#                       [tf_os1[0]         , tf_os1[1]         , tf_os1[2]         , 0                 , 0                 , 0                 , 1                 , 'chassis_init'                        , 'os1_init'                            , 100 ],
#                       [tf_os1[0]         , tf_os1[1]         , tf_os1[2]         , 0                 , 0                 , 0                 , 1                 , 'chassis_init'                        , 'os1_imu_init'                        , 400 ],
#                       [0                 , 0                 , 0                 , 0                 , 0                 , 0                 , 1                 , 'wheel_pose'                          , 'wheel_chassis'                       , 10  ],
#                       [0                 , 0                 , 0                 , 0                 , 0                 , 0                 , 1                 , 'wheel_imu_pose'                      , 'wheel_imu_chassis'                   , 10  ],
                       [-tf_t265[0]       , -tf_t265[1]       , -tf_t265[2]       , 0                 , 0                 , 0                 , 1                 , 't265_pose'                           , 't265_chassis'                        , 400 ],
#                       [-tf_os1[0]        , -tf_os1[1]        , -tf_os1[2]        , 0                 , 0                 , 0                 , 1                 , 'os1_pose'                            , 'os1_chassis'                         , 100 ],
#                       [-tf_os1[0]        , -tf_os1[1]        , -tf_os1[2]        , 0                 , 0                 , 0                 , 1                 , 'os1_imu_pose'                        , 'os1_imu_chassis'                     , 400 ],
#                       [0.15              , 0.01              , -0.085            , 0                 , 0                 , 0                 , 1                 , 'decawave_pose'                       , 'decawave_chassis'                    , 10  ],
#                       [-0.27             , 0                 , -0.025            , 0                 , 0                 , 0                 , 1                 , 'arfront_pose'                        , 'axis_front_chassis'                  , 30  ],
#                       [0.27              , 0                 , -0.025            , 0                 , 0                 , 0                 , 1                 , 'arback_pose'                         , 'axis_back_chassis'                   , 30  ],
                       [0                 , 0                 , -0.15             , 0                 , 0                 , 0                 , 1                 , 't265_chassis'                        , 'base_link'                           , 400 ],
#                       [0                 , 0                 , -0.15             , 0                 , 0                 , 0                 , 1                 , 'os1_imu_chassis'                     , 'base_link'                           , 400 ],
                       [0                 , 0                 , 0.15              , 0                 , 0                 , 0                 , 1                 , 'base_link'                           , 'chassis'                             , 400 ],
#                       [0.215             , 0.175             , 0                 , 0                 , 0                 , 0                 , 1                 , 'chassis'                             , 'left_front_wheel'                    , 10  ],
#                       [0                 , 0.225             , 0                 , 0                 , 0                 , 0                 , 1                 , 'chassis'                             , 'left_middle_wheel'                   , 10  ],
#                       [-0.215            , 0.175             , 0                 , 0                 , 0                 , 0                 , 1                 , 'chassis'                             , 'left_back_wheel'                     , 10  ],
#                       [0.215             , -0.175            , 0                 , 0                 , 0                 , 0                 , 1                 , 'chassis'                             , 'right_front_wheel'                   , 10  ],
#                       [0                 , -0.225            , 0                 , 0                 , 0                 , 0                 , 1                 , 'chassis'                             , 'right_middle_wheel'                  , 10  ],
#                       [-0.215            , -0.175            , 0                 , 0                 , 0                 , 0                 , 1                 , 'chassis'                             , 'right_back_wheel'                    , 10  ],
#                       [tf_xsens[0]       , tf_xsens[1]       , tf_xsens[2]       , 0                 , 0                 , 0                 , 1                 , 'chassis'                             , 'imu_link'                            , 100 ],
#                       [tf_os1[0]         , tf_os1[1]         , tf_os1[2]         , 0                 , 0                 , 0                 , 1                 , 'chassis'                             , 'os1_sensor'                          , 100 ],
#                       [0.27              , 0                 , 0.025             , 0                 , 0                 , 0                 , 1                 , 'chassis'                             , 'axis_link'                           , 30  ],
#                       [0.27              , 0                 , 0.025             , 0                 , 0                 , 0                 , 1                 , 'chassis'                             , 'axis_front_link'                     , 30  ],
#                       [-0.27             , 0                 , 0.025             , 0                 , 0                 , 0                 , 1                 , 'chassis'                             , 'axis_back_link'                      , 30  ],
#                       [tf_t265[0]        , tf_t265[1]        , tf_t265[2]        , 0                 , 0                 , 0                 , 1                 , 'chassis'                             , 't265_link'                           , 200 ],
                       [tf_d435i[0]       , tf_d435i[1]       , tf_d435i[2]       , qt_d435i[0]       , qt_d435i[1]       , qt_d435i[2]       , qt_d435i[3]       , 'chassis'                             , 'd435i_link'                          , 400 ],
#                       [-0.15             , -0.01             , 0.085             , 0                 , 0                 , 0                 , 1                 , 'chassis'                             , 'decawave_link'                       , 10  ],
#                       [0.2               , 0.0325            , 0.125             , 0                 , 0                 , 0                 , 1                 , 'chassis'                             , 'kinect_link'                         , 60  ],
#                       [0                 , 0                 , 0                 , 0                 , 0                 , 0                 , 1                 , 'imu_link'                            , 'imu'                                 , 100 ],
#                       [0                 , 0                 , 0.03618           , 0                 , 0                 , 1                 , 0                 , 'os1_sensor'                          , 'os1_lidar'                           , 10  ],
#                       [0.006253          , -0.011775         , 0.007645          , 0                 , 0                 , 0                 , 1                 , 'os1_sensor'                          , 'os1_imu'                             , 100 ],
#                       [0                 , 0                 , 0                 , -0.5              , 0.5               , -0.5              , 0.5               , 'axis_link'                           , 'axis_optical_frame'                  , 30  ],
#                       [0                 , 0                 , 0                 , -0.5              , 0.5               , -0.5              , 0.5               , 'axis_front_link'                     , 'axis_front_optical_frame'            , 30  ],
#                       [0                 , 0                 , 0                 , -0.5              , -0.5              , 0.5               , 0.5               , 'axis_back_link'                      , 'axis_back_optical_frame'             , 30  ],
#                       [3.80186684197e-05 , 0.0320889987051   , 2.4195331207e-05  , 0.00326163647696  , -0.999985337257   , -0.00424767192453 , 0.000792266393546 , 't265_link'                           , 't265_fisheye1_frame'                 , 30  ],
#                       [0                 , 0                 , 0                 ,  0.5              , -0.5              , -0.5              , 0.5               , 't265_fisheye1_frame'                 , 't265_fisheye1_optical_frame'         , 30  ],
#                       [-3.80186684197e-05, -0.0320890024304  , -2.41953239311e-05, 0.000553577148821 , -0.99999755621    , -0.00205105380155 , 0.000613214506302 , 't265_link'                           , 't265_fisheye2_frame'                 , 30  ],
#                       [0                 , 0                 , 0                 , 0.5               , -0.5              , -0.5              , 0.5               , 't265_fisheye2_frame'                 , 't265_fisheye2_optical_frame'         , 30  ],
#                       [-3.17073136102e-05, 0.0213896092027   , 0.000115149479825 , 0                 , 0                 , 1                 , 0                 , 't265_link'                           , 't265_gyro_frame'                     , 200 ],
#                       [0                 , 0                 , 0                 , 0.5               , -0.5              , -0.5              , 0.5               , 't265_gyro_frame'                     , 't265_gyro_optical_frame'             , 200 ],
#                       [-3.17073136102e-05, 0.0213896092027   , 0.000115149479825 , 0                 , 0                 , 1                 , 0                 , 't265_link'                           , 't265_accel_frame'                    , 62  ],
#                       [0                 , 0                 , 0                 , 0.5               , -0.5              , -0.5              , 0.5               , 't265_accel_frame'                    , 't265_accel_optical_frame'            , 62  ],
#                       [0                 , 0                 , 0                 , 0                 , 0                 , 0                 , 1                 , 't265_link'                           , 't265_depth_frame'                    , 30  ],
#                       [0                 , 0                 , 0                 , -0.5              , 0.5               , -0.5              , 0.5               , 't265_depth_frame'                    , 't265_depth_optical_frame'            , 30  ],
#                       [0                 , 0                 , 0                 , -0.5              , 0.5               , -0.5              , 0.5               , 't265_pose_frame'                     , 't265_depth_optical_frame'            , 200 ],
#                       [0                 , 0                 , 0                 , 0                 , 0                 , 0                 , 1                 , 't265_pose_frame'                     , 't265_link'                           , 200 ],
#                       [0                 , 0                 , 0                 , 0                 , 0                 , 0                 , 1                 , 't265_pose_frame'                     , 'd435i_link'                          , 100 ],
                       [0                 , 0                 , 0                 , 0                 , 0                 , 0                 , 1                 , 'd435i_link'                          , 'd435i_depth_frame'                   , 100 ],
                       [0                 , 0                 , 0                 , -0.5              , 0.5               , -0.5              , 0.5               , 'd435i_depth_frame'                   , 'd435i_depth_optical_frame'           , 100 ],
                       [-0.000214694606257, 0.0150816757232   , 7.47397498344e-05 , 0.0015756865032   , -0.000297236372717, 0.0036272148136   , 0.999992132187    , 'd435i_link'                          , 'd435i_color_frame'                   , 60  ],
                       [-0.000214694606257, 0.0150816757232   , 7.47397498344e-05 , 0.0015756865032   , -0.000297236372717, 0.0036272148136   , 0.999992132187    , 'd435i_link'                          , 'd435i_aligned_depth_to_color_frame'  , 60  ],
                       [0                 , 0                 , 0                 , -0.5              , 0.5               , -0.5              , 0.5               , 'd435i_aligned_depth_to_color_frame'  , 'd435i_color_optical_frame'           , 60  ],
                       [0                 , 0                 , 0                 , 0                 , 0                 , 0                 , 1                 , 'd435i_link'                          , 'd435i_infra1_frame'                  , 100 ],
                       [0                 , 0                 , 0                 , 0                 , 0                 , 0                 , 1                 , 'd435i_link'                          , 'd435i_aligned_depth_to_infra1_frame' , 100 ],
                       [0                 , 0                 , 0                 , -0.5              , 0.5               , -0.5              , 0.5               , 'd435i_aligned_depth_to_infra1_frame' , 'd435i_infra1_optical_frame'          , 100 ],
                       [0                 , -0.0499469712377  , 0                 , 0                 , 0                 , 0                 , 1                 , 'd435i_link'                          , 'd435i_infra2_frame'                  , 100 ],
                       [0                 , 0                 , 0                 , -0.5              , 0.5               , -0.5              , 0.5               , 'd435i_infra2_frame'                  , 'd435i_infra2_optical_frame'          , 100 ],
                       [-0.012            , -0.006            , 0.005             , 0                 , 0                 , 0                 , 1                 , 'd435i_link'                          , 'd435i_accel_frame'                   , 250 ],
                       [0                 , 0                 , 0                 , -0.5              , 0.5               , -0.5              , 0.5               , 'd435i_accel_frame'                   , 'd435i_accel_optical_frame'           , 250 ],
                       [-0.012            , -0.006            , 0.005             , 0                 , 0                 , 0                 , 1                 , 'd435i_link'                          , 'd435i_gyro_frame'                    , 400 ],
                       [0                 , 0                 , 0                 , -0.5              , 0.5               , -0.5              , 0.5               , 'd435i_gyro_frame'                    , 'd435i_gyro_optical_frame'            , 400 ],
                       ], dtype=object)

def qq_mult(q1, q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    return w, x, y, z

def q_conjugate(q):
    w, x, y, z = q
    return (w, -x, -y, -z)

def qv_mult(q1, v1):
    q2 = (0.0,) + v1
    return qq_mult(qq_mult(q1, q2), q_conjugate(q1))[1:]

def main_function():
    
    rospy.init_node('correct_node', anonymous=True)
    
    for k in range(len(input_file)):
        sys.stdout.write('\n%s' % input_file[k])
        sys.stdout.flush()
    
    if raw_input('\nAre you sure these are the correct files? (y/n): ') == 'y':
        sys.stdout.write('\n%s\n' % output_file)
        sys.stdout.flush()
        if os.path.exists(output_file):
            if raw_input('Output file already exists, do you want to replace it? (y/n): ') == 'y':
                os.remove(output_file)
            else:
                sys.exit(0)
    else:
        sys.exit(0)
    
    bag_in = numpy.empty(len(input_file), dtype=object)
    bag_time = numpy.zeros((2,len(input_file)))
    
    for k in range(len(input_file)):
        
        sys.stdout.write('\nOpening bag %d...' % (k+1))
        sys.stdout.flush()
        bag_in[k] = rosbag.Bag(input_file[k])
        sys.stdout.write(' Ok!')
        sys.stdout.flush()
        
        bag_time[1,k] = bag_in[k].get_end_time() - bag_in[k].get_start_time()
        for topic, msg, t in bag_in[k].read_messages():
            bag_time[0,k] = rospy.Time.to_sec(t)
            break
    
    if match_bags:
        bag_start_time = min(bag_time[0])
        bag_end_time = min(bag_time[0]) + max(bag_time[1])
        bag_shift_time = bag_time[0] - min(bag_time[0])
    else:
        bag_start_time = min(bag_time[0])
        bag_end_time = max(bag_time[0] + bag_time[1])
        bag_shift_time = numpy.zeros(len(input_file))
    
    sys.stdout.write('\n\nBag starting: %10.9f\nBag ending:   %10.9f\nBag duration: %3.1fs\n' % (bag_start_time, bag_end_time, (bag_end_time-bag_start_time)))
    for k in range(len(input_file)):
        sys.stdout.write('Bag %d shift:  %3.1f\n' % ((k+1), bag_shift_time[k]))
    sys.stdout.flush()
    sys.stdout.write('\n')
    
    bag_out = rosbag.Bag(output_file, 'w')
    
    k = 0
    
    while k < len(input_file):
        
        for topic, msg, t in bag_in[k].read_messages():
            
            if rospy.is_shutdown():
                k = len(input_file)
                break
            
            if match_bags and hasattr(msg, 'header'):
                t = rospy.Time.from_sec(rospy.Time.to_sec(t) - bag_shift_time[k])
                msg.header.stamp = copy(t)
            
#            if match_bags and hasattr(msg, 'transforms'):
#                t = rospy.Time.from_sec(rospy.Time.to_sec(t) - bag_shift_time[k])
#                msg.transforms[0].header.stamp = t
                
#            if topic == '/cmd_vel':
#                bag_out.write(topic, msg, t)
                
#            if topic == '/d435i/accel/sample':
#                bag_out.write(topic, msg, t)
                
            if topic == '/d435i/color/camera_info':
                bag_out.write(topic, msg, t)
                
            elif topic == '/d435i/color/image_raw':
                bag_out.write(topic, msg, t)
                
            elif topic == '/d435i/depth/camera_info':
                t_new = rospy.Time.to_sec(t)-depth_shift_time
                if t_new >= bag_start_time:
                    msg.header.stamp = rospy.Time.from_sec(t_new)
                bag_out.write(topic, msg, msg.header.stamp)
                
            elif topic == '/d435i/depth/image_rect_raw':
                t_new = rospy.Time.to_sec(t)-depth_shift_time
                if t_new >= bag_start_time:
                    msg.header.stamp = rospy.Time.from_sec(t_new)
                bag_out.write(topic, msg, msg.header.stamp)
                
#            if topic == '/d435i/gyro/sample':
#                bag_out.write(topic, msg, t)
            
#            if topic == '/odom':
#                topic = '/wheel/odom'
#                msg.header.frame_id = 'wheel_init'
#                msg.child_frame_id = 'wheel_pose'
#                msg.pose.pose.position.x = -msg.pose.pose.position.x
#                msg.pose.pose.position.y = -msg.pose.pose.position.y
#                tf_msg = TransformStamped()
#                tf_msg.header.stamp = copy(t)
#                tf_msg.header.frame_id = copy(msg.header.frame_id)
#                tf_msg.child_frame_id = copy(msg.child_frame_id)
#                tf_msg.transform.translation = copy(msg.pose.pose.position)
#                tf_msg.transform.rotation = copy(msg.pose.pose.orientation)
#                bag_out.write('/tf', TFMessage([tf_msg]), t)
#                bag_out.write(topic, msg, t)
                
#            if topic == '/os1_cloud_node/imu':
#                bag_out.write(topic, msg, t)
                
#            if topic == '/os1_cloud_node/points':
#                bag_out.write(topic, msg, t)
            
            elif topic == '/t265/odom/sample':
                topic = '/t265/odom'
                msg.header.frame_id = 't265_init'
                msg.child_frame_id = 't265_pose'
                tf_msg = TransformStamped()
                tf_msg.header.stamp = copy(t)
                tf_msg.header.frame_id = copy(msg.header.frame_id)
                tf_msg.child_frame_id = copy(msg.child_frame_id)
                tf_msg.transform.translation = copy(msg.pose.pose.position)
                tf_msg.transform.rotation = copy(msg.pose.pose.orientation)
                bag_out.write('/tf', TFMessage([tf_msg]), t)
                bag_out.write(topic, msg, t)
            
            status_time = 100.0 * (rospy.Time.to_sec(t) - bag_start_time) / (bag_end_time - bag_start_time)
            sys.stdout.write('\rRepublishing messages from bag %d... %3.1f%%' % ((k+1), status_time))
            sys.stdout.flush()
        
        sys.stdout.write('\n')
        sys.stdout.flush()
        
        k += 1
        
    sys.stdout.write('\n')
    sys.stdout.flush()
    
    for k in range(len(tf_args)):
        
        t = bag_start_time
        
        while t < bag_end_time and not rospy.is_shutdown():
            
            tf_msg = TransformStamped()
            tf_msg.header.stamp = rospy.Time.from_sec(t)
            tf_msg.header.frame_id = tf_args[k,7]
            tf_msg.child_frame_id = tf_args[k,8]
            tf_msg.transform.translation.x = tf_args[k,0]
            tf_msg.transform.translation.y = tf_args[k,1]
            tf_msg.transform.translation.z = tf_args[k,2]
            tf_msg.transform.rotation.x = tf_args[k,3]
            tf_msg.transform.rotation.y = tf_args[k,4]
            tf_msg.transform.rotation.z = tf_args[k,5]
            tf_msg.transform.rotation.w = tf_args[k,6]
            
            bag_out.write('/tf', TFMessage([tf_msg]), tf_msg.header.stamp)
            
            t += 1.0/tf_args[k,9]
            
            status_time = 100.0 * (t - bag_start_time) / (bag_end_time - bag_start_time)
            sys.stdout.write('\r%2d/%2d - Publishing %s transform... %3.1f%%' % ((k+1),len(tf_args),tf_args[k,8],status_time))
            sys.stdout.flush()
        
        sys.stdout.write('\n')
        sys.stdout.flush()
    
    for k in range(len(input_file)):
        bag_in[k].close()
    bag_out.close()
    
    sys.stdout.write('\nFiles Closed\n\n')
    sys.stdout.flush()

if __name__ == '__main__':
    try:
        main_function()
    except rospy.ROSInterruptException:
        pass
