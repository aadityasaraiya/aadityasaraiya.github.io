---
layout: post
title: "Introduction to GSoC 2018 with ROS Industrial"
date: "2018-07-16"
slug: "example_content"
description: "This blog post introduces my Google Summer of Code 2018 project and talks about my experience as a first time GSoC-er"
category: 
  - views
  - featured
# tags will also be used as html meta keywords.
tags:
  - examples
  - common_tag
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
imagesummary: foo.png
## for twitter card with large image:
# imagefeature: "http://img.youtube.com/vi/VEIrQUXm_hY/0.jpg"
## for twitter video card: (active for this page)
videofeature: "https://www.youtube.com/embed/iG9CE55wbtY"
imagefeature: "http://img.youtube.com/vi/iG9CE55wbtY/0.jpg"
videocredit: tedtalks
---

Hi everyone! This is the first blog post in a series of posts where I will talk about my general experience regarding the Google Summer of Code 2018 program. 

<!--more-->

## The What and How?

Google Summer of Code is an annual program by Google where it invites mentoring organisations from all around the world to choose students to mentor and work for developing open source software. This provides students an opportunity to learn directly from these open source organisations. I think the [Google Summer of Code website]((https://summerofcode.withgoogle.com/), [How It Works page](https://summerofcode.withgoogle.com/how-it-works/) and the [About Page](https://summerofcode.withgoogle.com/about/) will get you started with understanding how Google Summer of Code works. So instead, I think sharing my first-hand experience will be much more helpful. 

## Kick-Off to a Wonderful Summer

Having been working in the field of robotics for almost 3 years during my undergraduate studies, I had my brief stints of dealing with the Robot Operating System (ROS). However, I realised that I did not have have a wholesome understanding of the way ROS works. My GSoC journey began when I made the decision to spend my summer trying to understand the intricacies of ROS and I landed up on [this project idea](https://github.com/osrf/osrf_wiki/wiki/GSoC) titled **Robot WorkCell Discovery** with [ROS Industrial](https://rosindustrial.org/), which is an umbrella organisation under the Open Source Robotics Foundation. I contacted Alex Goins, who formerly worked for the [Southwestern Research Institute (SwRI)](https://www.swri.org/) and who is an active contributor to ROS Industrial. With his guidance, I developed a proposal for the project. 

## What is a robotic workcell? 

In industrial settings, robotic manipulators are designed for specialised tasks like arc welding, blending and other miscellaneous industrial tasks. The manipulator, with its peripheral equipment like controllers, networking devices, PCs and other structures specific to the task are isolated from humans for safety reasons, generally in form of a closed enclosure. These isolated areas are termed as **robotic workcells**. An example workcell is shown in the figure below. 

![An Example Robotic Workcell](https://drive.google.com/open?id=1cu3_ktKNSZBNnOShuPMZl_hqGGdD-_e-)


## How does the robot know about it's surroundings?

As the robotic manipulator has to function with a structured environment around it, it has to have information about the environment and where different objects are placed. Traditionally, the robot is provided information about the environment by modelling the environment and providing that information inside the URDF file. Universal Robot Description Format (URDF) files are in the end just simple XML files which have specific tags utilised to describe a robot model. Some tags include the `visual` tags which focuses on how the robot looks and the `geometry` tag which focuses on the structure of the robot. 

I feel there are plenty of tutorials out there explaining URDF files in general like [this one](https://ni.www.techfak.uni-bielefeld.de/files/URDF-XACRO.pdf). However, I will talk about some insights into the URDF files I worked with in a separate blog post probably. You can read more about URDF files and how they are parsed on [this ROS Wiki page](http://wiki.ros.org/urdf). 

## Limitations of static models of the environment

Modelling the environment works for simple and static workcells. However, as the complexity increases, there are limitations to such models which can be summarised as follows:

1. Modern industrial robotic applications work with robots in an unstructured environments. These may include soft/non-rigid bodies or cluttered environments. It would be difficult to model such environments with the traditional methods. 

2. By pre-defining the environment as a static model, the manipulator cannot handle any dynamic changes in the environment. Hence, if the workcell is shifted or slightly adjusted, the URDF file will have be changed each time.

3. With an increasing interest in human-in-the-loop systems and human-robot interaction systems as observed in the Baxter Colloborative robot by Rethink Robotics, robot will have to actively take visual cues from it's surroundings to function accordingly. 

## Focus on Autonomous Workcell Exploration 

Keeping the limitations of static models in mind, the focus of this GSoC 2018 project is to create a **ROS Package for autonomous workcell exploration**. This package should allow an arbitrary robotic manipulator to **intelligently** explore and map it's workcell environment by utilising a 3-D vision sensor, particularly a popular RGB-D sensor called Kinect. The main focus, hence shifts from modelling the environment manually, to allowing the robot to do itself. The various steps involved in this process can be summarised as follows:

1. We need a model of the robot as well as it's work environment to simulate the workflow. For this project, a UR5 robot manipulator and Kinect V1 sensor is utilised. Gazebo is used as the default physics simulator, while the visualisation of sensor data is done using RViz. The default view of the robot in Gazebo and RViz can be seen in the figure below. The visualisation of the ball in RViz seems to be made of tiny cubes. This probabilistic representation of the surface is called an [Octomap](https://octomap.github.io/). 

![(left)(Robot model in Gazebo (right) MoveIt! enabled manipulator visualised in RViz](https://drive.google.com/open?id=1ZUSKa1DH3bVaL7KN7zX0EL7unUTeFWd3)

2. We need a dynamic 3-D reconstruction pipeline which takes in point-clouds from different camera views and fuses it into a single 3-D model. The algorithm also needs to track the estimate of the camera pose. For this process, I have utilised the [yak (Yet another Kinect Fusion) package](https://github.com/AustinDeric/yak) which is basically a ROS wrapper around an algorithm called **Kinect Fusion** which is an RGB-D based 3-D reconstruction pipeline. Kinect fusion uses the **Truncated Signed Difference function (TSDF)** for fusing multiple views into a single model.  

3. The Kinect sensor which is mounted on the robotic manipulator has to be moved around in it's reachable workspace to explore the environment around itself. Instead of randomly moving the manipulator in it's workspace, there is a ROS package called the Next-best view planner, which is used with the yak package in order to increase the probability of exploring environments with previously unseen views.

4. The manipulator needs to physically move to the views which are provided by the the **Next-Best view planner**, with certain constraints such as keeping the object always in the view of the camera.

I have summarised the entire project workflow in the figure below.  

![Summary of Project Workflow](https://drive.google.com/open?id=19VDrCxfMOGcLJ-p3zlOOc0QE4ZDLV9hf)

Thank you guys for reading this blog post. Anyone interested in viewing the detailed inital proposal for this project, they can access it via  [this Google Drive Link](https://drive.google.com/file/d/1JeZgfYfiJNrtUgfcV9WYJWoDF1quoCYu/view?usp=sharing). A lot of ideas have changed after the inital proposal and they may not give you an exact idea of what is currently being done. However, the overall picture, the goals and the techniques utilised remain the same. 

**Note for newbies like me**: I may have introduced a lot of unheard terms such as Octomaps and the process of how 3-D reconstruction works may sound hazy. I can totally understand that as that was my condition when I started working on this project. In my future blog posts, I will try to throw some light on how basic concepts behind the Kinect fusion pipeline for 3-D reconstruction. Also, I will try my best to come up with basic tutorials on how to use MoveIt!, however, I strongly suggest the [ROS Industrial](https://ros-industrial.github.io/industrial_training/) and [MoveIt!](http://docs.ros.org/kinetic/api/moveit_tutorials/html/index.html) tutorials to get going with the process of setting up your own robot model with ROS. 

**System configuration note**- I have used Ubuntu 16.04 and ROS Kinectic for the purpose of this project. 

## Meet the Mentors 



[^3]: [About]({{ site.url }}/about)


