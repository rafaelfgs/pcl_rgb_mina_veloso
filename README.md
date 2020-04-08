# pcl_rgb_mina_veloso

Repository with the necessary files to generate a colored PointCloud from bags recorded in Mina du Veloso with EspeleoRobô.

The recorded bags are located in the Google Drive account of EspeleoRobô.

The output of this repository is a .pcd file, containing the point cloud generated with the bag.


## Nodes

This repository includes the following nodes:


### *correct_node*

This node is represented by the file *necessary_files/scripts/correct_bag_veloso.py*. It is used to create another bag with the correct configurations for the topics used in point cloud generation:

* */d435i/color/camera_info (sensor_msgs/CameraInfo)*
* */d435i/color/image_raw (sensor_msgs/Image)*
* */d435i/depth/camera_info (sensor_msgs/CameraInfo)*
* */d435i/depth/image_rect_raw (sensor_msgs/Image)*
* */t265/odom/sample (nav_msgs/Odometry)*
* */tf (tf2_msgs/TFMessage)*

The process corrects the time delay of the depth image, as well as includes missing transforms of the original bag.

The main parameters of this node can be edited in the file, where:

* *input_file*: represents the complete name for the input bags for merge.
* *output_file*: represents the name for the output bag.
* *depth_delay*: is the time delay between depth and color images.


### *sync_node*

Using the corrected bag, this node republishes both color and depth images. The file is located in *necessary_files/scripts/sync_depth_to_color.py*. It synchronizes the messages, keeping a specific publication rate, fixes a range for depth message, and makes others general corrections. The topics published for this node are:

* */depth_registered/image_rect (sensor_msgs/CameraInfo)*
* */depth_registered/camera_info (sensor_msgs/Image)*
* */rgb/image_rect_color (sensor_msgs/CameraInfo)*
* */rgb/camera_info (sensor_msgs/Image)*


### *xyzrgb_node

This node uses the synchronized messages to generate the point cloud. The file is located in *depth_image_proc/src/nodelets/point_cloud_xyzrgb.cpp*, obtained from https://github.com/ros-perception/image_pipeline. The topic published is:

* */depth_registered/points (sensor_msgs/PointCloud2)*

Note: it is necessary to play the bag before running this node.


### *convert_node*

For a better view of the points' coordinates, this node is used to convert the type of message from *PointCloud2* to *PointCloud*. The file is located in *point_cloud_converter/src/converter.cpp*, obtained from https://github.com/pal-robotics-forks/point_cloud_converter. The topic published is: 

* */points_out (sensor_msgs/PointCloud)*


### *pcd_node*

The final node is used to create a *.pcd* file using the point cloud from the last node. The file contains the points' coordinates and their respective values of RGB color. Each line in the file represents a point in the form *[x, y, z, rgb]*. This node is located in *necessary_files/scripts/pointcloud_to_pcd.py*.

The main parameter is:

* *output_file*, representing the complete name for the *.pcd* file.


### *rviz_node*

Finally, for visualization of the whole process, this node open RViz with a correct configuration for topics and view.


## Building

The commands for built the package are simple:

```bash
git clone https://github.com/rafaelfgs/imu_turtle.git
cd ..
catkin build
source devel/setup.bash
```

Note: *catkin build* can be replaced by *catkin_make*.


## Running

First, it is necessary to correct the bags, running the first node with the command:

```bash
rosrun necessary_files correct_bag_veloso.py
```

Then, the nodes can be called through following sequence of commands:

```bash
rosrun necessary_files sync_depth_to_color.py
rosrun point_cloud_converter point_cloud_converter_node points2_in:='/depth_registered/points'
rosrun necessary_files pointcloud_to_pcd.py
rosbag play PATH_TO_FILE.bag
rosrun nodelet nodelet standalone depth_image_proc/point_cloud_xyzrgb
```

To simplify these command, a *.launch* file can be used for running all nodes (except for *correct_bag_veloso*), through the commands:

```bash
rosbag play PATH_TO_FILE.bag
roslaunch necessary_files pcd_rgb_mina_veloso.launch 
```
