---
layout: post
title: "ROS Workcell Explorer Package Structure"
date: "2018-08-09"
slug: "ROS_Workcell_Explorer_Package_Structure"
description: "This blog post focuses on the different components in the ROS package which I have used for my GSoC 2018 project"
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

Hi everyone! This is the third blog post related to my GSoC 2018 project on creating a ROS package for WorkCell exploration/discovery. In this one, I will try to systematically decompose the ROS package which I worked on into various components which includes URDFs, controllers, setup for Gazebo, package for Kinect Fusion, planning next-best-views and workcell exploration. To the new readers, thanks a lot for reading my post. You can start off reading about by GSoC journey with [this post](https://aadityasaraiya.github.io//blog/2018/07/16/GSoC_2018_with_ROS_Industrial/). 

**Note**: The parts on Kinect Fusion, planning next-best-views and workcell exploration has been taken up in the [next blog post](https://aadityasaraiya.github.io//blog/2018/08/08/KinectFusion_NBV_planner_and_workcell_exploration/) to maintain readability. 

<!--more-->

## Why use the Robot Operating System (ROS)?

Over the past decade, the robotics community has shown a lot of interest in the development of a common platform to make robotics software re-usable and easy to distribute. This lead to the design of the Robot Operating System (ROS), which can be termed as a *meta-operating system* for robots. It features a modular design which includes nodes, which communicate with each other using *messages/services* through *certain topics*(like a data-bus). This distributed architecture has been successful and has been adopted by the community on a large scale. 

For beginners, I absolutely recommend [ROS Crash-Course Part 1](https://courses.cs.washington.edu/courses/cse466/11au/calendar/ros_cc_1_intro-jrsedit.pdf) and [ROS Crash Course Part 2](https://courses.cs.washington.edu/courses/cse466/11au/calendar/ros_cc_2_patterns.pdf) in order to understand the philosophy and basic tools which are provided by ROS.

I will try to explain the package architecture in as simple/general terms as possible. However, if someone wants a deeper understading of ROS, I would recommend you to start off with the [ROS tutorials](http://wiki.ros.org/ROS/Tutorials). May the [ROS Wiki](http://wiki.ros.org/) be your guiding light! 

## Robot modelling and URDF files 

So to start of with the package architecture, the first step in creation of the ROS package was choosing the robot model. For this project, I chose the UR5 robot from [Universal robots](http://wiki.ros.org/universal_robot) due to its maneuverability, adequate development support as well as its use in the [ROS Industrial tutorials](https://ros-industrial.github.io/industrial_training/). You can have a look at the [URDF file](https://github.com/ros-industrial/workcell_explorer/blob/master/myworkcell_support/urdf/workcell_gazebo_trial1.urdf) which I am using for this project. 

Some key **URDF tags** which can help you get started with understanding the URDF format. 

|Sr no. | Tag name | Properties |
|:----- |:-------- | :-----     |
| 1     | link     | link name, mass,origin,inertia etc.|
| 2     | visual   | how the part looks (using .dae files) |
| 3     | geometry | define primitive shapes such as cone, sphere, box |
| 4     | collision| how the part behaves on collision (using .stl files)|
| 5     | inertial | defining mass, origins and moment of inertia for physics simulators|
| 6     | joint    | joint type, parent and child link, joint limit, dynamics|
| 7     | gazebo   | gazebo sensor/ros control plugins, sensor behavior etc.|

## RViz and ROS MoveIt! 

[RViz](http://wiki.ros.org/rviz) is a 3-D visualisation tool used to observe the robot's behavior as well as observe simulated sensor readings. 

After having created the URDF of the robot model, you can visualise it in RViz using a [launch file like this](https://github.com/ros-industrial/workcell_explorer/blob/master/myworkcell_support/launch/workcell_ur5.launch).

___
**Note**- I have commented out the `robot_state_publisher` and the `joint_state_publisher` packages in this file as they are being called somewhere else. However, if you are using this file solely, go ahead and uncomment those lines. You can use [this link](https://answers.ros.org/question/275079/joint-state-publisher-and-robot-state-publisher/) to undestand the difference between both these packages.

**tl;dr** - The `robot_state_publisher` uses the URDF to calculate the forward kinematics and publishes the `/tf` transforms. The `joint_state_publisher` publishes the `sensor_msgs/JointState` messages for the robot.  
___

[ROS MoveIt!](https://moveit.ros.org/) is a motion planning software which has a lot of inbuilt functionalities for manipulation, mobile manipulation, 3D perception etc. 

To configure MoveIt! with your robot URDF, you need to use the `moveit_setup_assistant`. This tutorial by [The Construct](http://www.theconstructsim.com/ros-movelt/) can give you a brief example in order to illustrate the process. To dive deeper, you can view these tutorials by [MoveIt! for ROS Kinetic](http://docs.ros.org/kinetic/api/moveit_tutorials/html/index.html). After setting up MoveIt!, a `move_group` is exposed for one particular robot. The MoveIt! API allows us to directly command the robot by issuing commands to this `move_group`. 

The image below shows the general MoveIt! workflow 

![MoveIt! workflow](/images/10_8_2018/moveit_flowchart .png)

You can go have a look at the `moveit_config` package generated by the `moveit_setup_assistant` via [this link](https://github.com/ros-industrial/workcell_explorer/tree/master/myworkcell_moveit_config).

## Setting up robot controllers with Gazebo

So after the previous step, we have a robot which is configured to work with RViz and ROS MoveIt!. However, for simulating physics/physical environments, we need a 3-D physics simulator, which is Gazebo in our case. 

Gazebo uses information from the `<gazebo>` tag in order to set up various plugins. Plugins allow us to use ROS inbuilt libraries for stuff like simulating the Kinect sensors performance or using ROS control.  

The following flowcart shows the **ROS Control workflow** and how it can be interfaced with Gazebo. 

![Depiction of the tracking problem](/images/10_8_2018/gazebo_ros_control.png)

In order to setup the controllers, we need to edit multiple `.yaml` files and configure the robot with Gazebo, an example of which can be seen through [this link](https://github.com/ros-industrial/workcell_explorer/tree/kf_branch1/myworkcell_moveit_config/config). 

Instead of explaining the full process, I would suggest you to go through this [Github wiki created by the AS4SR lab](https://github.com/AS4SR/general_info/wiki/Basic-ROS-MoveIt!-and-Gazebo-Integration) which will guide you through the process of integrating Gazebo with ROS MoveIt!. 

## Understanding the complete code workflow  

So finally, having setup the robot and the corresponding simulators, we start the process of analysing how the complete code workflow works. You can follow these commands on the [project repository as well](https://github.com/ros-industrial/workcell_explorer).

**Step 1- Launch Gazebo world + robot mounted with a Kinect camera**

~~~
roslaunch myworkcell_support workcell_gazebo.launch 
~~~

The following image shows the robot initalised in Gazebo with a dumpster which has been added manually. 

![Initalisation of robot in Gazbeo with addition of a Dumpster manu](/images/10_8_2018/gazebo_init.png)

This command initialises the drivers for the simulated Kinect sensor. Some of the important topics published are as follows:

+ `/camera/depth/image_raw`- Raw depth image 

+ `/camera/depth/points`- Points from the depth point cloud

+ `/camera/color/camera_info`- Camera information such as the intrinsics and the extrinsics

+ `/camera/color/image_raw`- Raw color image

This command also initialises the `follow_joint_trajectory` controller which is used by Gazebo for executing motion control commands provided by MoveIt!. 


**Step 2- Launch the MoveIt! planning execution** 

~~~
roslaunch myworkcell_moveit_config myworkcell_planning_execution.launch.xml
~~~

The following image shows the robot initialised in RViz with ROS MoveIt! enabled. 

![Initalisation of robot in RViz and with MoveIt! enabled](/images/10_8_2018/rviz_init.png)

Some of the important topics initialised include the `move_group` related topics, `planning_scene`, `pickup` and `place` topics, `joint_trajectory_action` topics. 

___

**Note**- Using the `move_group` object, we can write C++/ Python code as well to give motion commands to the robotic manipulator in RViz.  
___

The given image shows the default MoveIt! menu which can be used to give commands to the robot.

![Default MoveIt! menu](/images/10_8_2018/moveit_menu.png)

The following image shows different TF frames on RViz initialisation with the robot visual disabled. 

![TF frame visualised in RViz](/images/10_8_2018/rviz_tf_init.png)


## What's next?

So, in this blog post, we talked about how the overall package structure was created in order to simulate the robot in Gazebo and RViz. We also discussed briefly about how ROS MoveIt! is being used. The next blog post will cover how the functionalities of KinectFusion, Next-Best-View planners and workcell exploration is carried out. 


[^3]: [About]({{ site.url }}/about)


