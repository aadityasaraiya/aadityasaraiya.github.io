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

Hi everyone! So this is probably the last blog post on the Google Summer of Code series. This post focuses on the ROS package structure for the KinectFusion (YAK), Next-best-view (nbv_planner) and the workcell exploration functionalities which have bene used while understanding the process of workcell exploration. The previous blog post covers the inital aspects of how the ROS package structure was designed.

<!--more-->

So the following flowchart summarises the complete process after the robot has been initalised in Gazebo and RViz. A thorough analysis will reveal the entire pipeline for the Kinect Fusion process. 

![Flowchart of Kinect Fusion, NBV planner and exploration tasks](/images/10_8_2018/kf_flowchart.png)

## Kinect Fusion 

After the robot has been launched in Gazebo and RViz with ROS MoveIt! enabled, the Kinect fusion ROS node has to be launched.

**Step 3- Launch the TSDF reconstruction node in the yak package**

A detailed description of the functions as well as the order in which different things are executed can be found in [this wiki on the workcell_explorer repository](https://github.com/ros-industrial/workcell_explorer/wiki/Kinect-Fusion-node-summary). 
~~~
roslaunch yak launch_gazebo_robot.launch
~~~


## Next-Best View planner

**Step 4- Start Octomap server and call the Next-Best-View planner**

~~~
roslaunch nbv_planner octomap_mapping.launch
~~~

A detailed description of the need for the next-best-view planner as well as the sequential order in which things are executed can be found in [this wiki on the workcell_explorer repository](https://github.com/ros-industrial/workcell_explorer/wiki/NBV-Planner-node-summary). 


## Exploration controller node 

**Step 5- Start the exploration process**

~~~
rosrun nbv_planner exploration_controller_node
~~~

A detailed description on the use of the the `exploration_controller` node can be found in [this wiki on the workcell_explorer repository](https://github.com/ros-industrial/workcell_explorer/wiki/Exploration-controller-node-summary). 



## Extract mesh after reconstruction is good enough

**Step 6- Extract mesh after deciding if reconstruction is good enough**


~~~
rosservice call /get_mesh
~~~







[^3]: [About]({{ site.url }}/about)


