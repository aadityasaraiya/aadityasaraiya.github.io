---
layout: post
title: "KinectFusion, NBV_planner and workcell exploration"
date: "2018-08-10"
slug: "KinectFusion_NBV_planner_and_workcell_exploration"
description: "This blog post focuses on the ROS package structure for the KinectFusion (YAK), Next-best-view (nbv_planner) and the workcell exploration functionalities"
category: 
  - views
  - featured
# tags will also be used as html meta keywords.
tags:
  - GSoC 2018
show_meta: true
comments: true
mathjax: true
gistembed: true
published: true
noindex: false
nofollow: false
# hide QR code, permalink block while printing.
hide_printmsg: false
# show post summary or full post in RSS feed.
summaryfeed: false
## for twitter summary card with squared image and page description or page excerpt:
#imagesummary: foo.png
## for twitter card with large image:
# imagefeature: "http://img.youtube.com/vi/VEIrQUXm_hY/0.jpg"
## for twitter video card: (active for this page)
#videofeature: "https://www.youtube.com/embed/iG9CE55wbtY"
imagefeature: ros_logo.png
#videocredit: tedtalks
---

Hi everyone! So this is probably the last blog post on the Google Summer of Code series. This post focuses on the ROS package structure for the KinectFusion (YAK), the Next-best-view (nbv_planner) and the workcell exploration functionalities which have been used while understanding the process of workcell exploration. The previous blog post covers the inital aspects of how the ROS package structure was designed.

<!--more-->

So the following flowchart summarises the complete process after the robot has been initalised in Gazebo and RViz. A thorough analysis will reveal the entire pipeline for the Kinect Fusion process. 

![Flowchart of Kinect Fusion, NBV planner and exploration tasks](/images/10_8_2018/kf_flowchart.png)

## Kinect Fusion 

After the robot has been launched in Gazebo and RViz with ROS MoveIt! enabled, the Kinect fusion ROS node has to be launched.

**Step 3- Launch the TSDF reconstruction node in the yak package**

~~~
roslaunch yak launch_gazebo_robot.launch
~~~

This particular launch file can be accessed via [this link](https://github.com/aadityasaraiya/yak_edit/blob/master/yak/launch/launch_gazebo_robot.launch). The parameters in the launch file can be directly tuned in order to check the performance of Kinect Fusion such as adjusting the `bilateral filter` used for noise reduction, `number of iterations` for ICP etc. 

This launch file calls the [Kinect Fusion node](https://github.com/aadityasaraiya/yak_edit/blob/master/yak/src/kinfu_node.cpp), which internally creates the `KinFuServer` object. This object takes up the depth frame from the Kinect sensor data and the fixed frame (the centre of the volume which will be reconstructed) and applies the full Kinect Fusion process.  

The following images showcases the Kinect Fusion data with both pose and motion corrections being recorded. This is essentially what is done in the process of **camera tracking**. 

![Kinect Fusion data updates](/images/10_8_2018/kinfu_message.png)

The following image shows the volume box with the `volume_pose` tf frame. The TSDF volume showcases the reconstructed volume with respect to the frame of the camera. 

![TSDF volume box](/images/10_8_2018/tsdf_volume_box.png)

The Kinect Fusion package does the camera tracking and it uses Octomap in order to do the mapping process which I will discuss in the next title. 

A detailed description of the functions as well as the order in which different things are executed can be found in [this wiki on the workcell_explorer repository](https://github.com/ros-industrial/workcell_explorer/wiki/Kinect-Fusion-node-summary). The theoretical discussion behind Kinect Fusion can be found via these links [part 1](https://aadityasaraiya.github.io//blog/2018/08/07/Kinect_Fusion_for_3-D_reconstruction_Part_1/) and [part 2](https://aadityasaraiya.github.io//blog/2018/08/08/Kinect_Fusion_for_3-D_reconstruction_Part_2/). 

## Next-Best View planner

**Step 4- Start Octomap server and initialising the Next-Best-View planner**

~~~
roslaunch nbv_planner octomap_mapping.launch
~~~

Ths launch file can be accessed via [this link](https://github.com/aadityasaraiya/yak_edit/blob/master/nbv_planner/launch/octomap_mapping.launch).The launch file does the following tasks:

+ It launches the [octomap_server package](http://wiki.ros.org/octomap_server) which loads a 3-D Octomap and allows other nodes to access it and build the octomap as multiple depth frames are collected. 

+ It launches the `nbv_planner_node` which does the process of finding the next-best-views which will be explained in the next heading.

+ It also starts the `octomap_reset` node which resets the Octomap and removes any old Octomap data.

The bounds of the Octomap are set up and then the node waits for the `GetNBV` service to be called by the explorer_controller node.  

## Exploration controller node 


**Step 5- Start the exploration process**

~~~
rosrun nbv_planner exploration_controller_node
~~~

As soon as the `explorer` object is created on running [this node](https://github.com/aadityasaraiya/yak_edit/blob/master/nbv_planner/src/exploration_controller_node.cpp), the `GetNBV` service is called.

**NBV Planner node**

The [GetNBV service](https://github.com/aadityasaraiya/yak_edit/blob/master/nbv_planner/src/exploration_controller_node.cpp) is defined in this file. The tasks of the `GetNBV` can be summarised as follows:

+ Retrieve the current Octomap ans store it in Abstract and color octrees. 

+ Check all the voxels in the current view which has unknown values. Using this, a new octomap is made. The unknown points (centres of the unknown voxels) are converted into values which can be understood by ROS (PCL points) and then visualised in RViz.

+ Then the next task is to choose random poses with respect to the center of the object being explored.

+ The camera distance is randomly varied in a particular range in order to explore new views. 

+ The next-best-view planning is done by raycasting a constant number of rays for each pose. The rays are ranked in a way such that the rays which can explore higher number of unknown voxels are given a better rank. 

+ Hence, using this method, we are indirectky trying to generate poses which are reachable and could potentially reveal new regions of the surface. 

+ The list of poses are sorted as per the rank and then utilised by the `explorer_controller` node. 


A detailed description of the need for the next-best-view planner as well as the sequential order in which things are executed can be found in [this wiki on the workcell_explorer repository](https://github.com/ros-industrial/workcell_explorer/wiki/NBV-Planner-node-summary). 

The following figure showcases the process of raycasting to plan next-best-views.

![Raycasting for planning next best views TSDF volume box](/images/10_8_2018/raycasting.png )

**Explorer Controller node**

The exploration controller node utilises the ranked list of poses which is provided to it to plan a path. The RRT* path planner is utilised in order to search for viable solutions/configurations the robot could move in order to reach to the required poses. 

The `move_group` object is utilised in order to provide the motion commands to the robot. This node also loads the robot model. 

A detailed description on the use of the the `exploration_controller` node can be found in [this wiki on the workcell_explorer repository](https://github.com/ros-industrial/workcell_explorer/wiki/Exploration-controller-node-summary). 


## Extract mesh after reconstruction is good enough

**Step 6- Extract mesh after deciding if reconstruction is good enough**


~~~
rosservice call /get_mesh
~~~

The meshing process fits polygons into the octomap explored and creates a 3-D map from the same. This service to get the mesh can be called any time when sufficient exploration of the workcell is carried out. 


## Thats all folks

Google Summer of Code and working with ROS Industrial has been a wonderful learning experience. There were definitely lot of challenges and a lot of scope of improvement. However, I am happy I took up this project as it forced me to understand ROS and robotics software in a more organised way. Thanks a lot for reading and will see you soon with some fresh content. 


[^3]: [About]({{ site.url }}/about)


