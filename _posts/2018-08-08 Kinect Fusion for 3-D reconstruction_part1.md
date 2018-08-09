---
layout: post
title: "Kinect Fusion for 3-D reconstruction- Part 1"
date: "2018-08-07"
slug: "Kinect_Fusion_for_3-D_reconstruction_Part_1"
description: "With this blog post, I want to focus on Kinect Fusion and the overall philosophy behind 3-D reconstruction explained in a simple fashion"
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
imagefeature: Kinfu.jpeg
#videocredit: tedtalks
---

Hello, world! This is the second blog post in a series of posts talking about my GSoC 2018 project. The focus of this post is on Kinect Fusion and the principles behind 3-D reconstruction. To the new readers, thanks a lot for reading my post. You can refer to my [first blog post](https://aadityasaraiya.github.io//blog/2018/07/16/GSoC_2018_with_ROS_Industrial/) in this series in order to understand the flow of things in a better manner. 

<!--more-->


**Kinect Fusion for 3-D reconstruction- Part 1** of this particular blog post covers the basics of 3-D reconstruction in order to develop a conceptual understanding of the reconstruction pipeline. 

**Kinect Fusion for 3-D reconstruction- Part 2** covers the conceptual details of Kinect Fusion in particular as a pipeline for 3-D reconstruction. 


## What this post is about

+ In Part 1, simple explanations are utilised in order to develop a broader picture of how 3-D reconstruction works. 
+ In Part 2, the key steps involved in the Kinect Fusion pipeline are explained. A brief exploration of the mathematical tools used by Kinect Fusion is carried out. 

## What this post is NOT about 

+ A replacement to deeper mathematical understanding of the concepts related to Kinect Fusion. To truly understand the finer aspects, you would really have to analyse the [Kinect Fusion paper](https://www.microsoft.com/en-us/research/wp-content/uploads/2016/02/ismar2011.pdf)[1] in depth.
 
+ However, you could go through my [notes/summary](https://github.com/ros-industrial/workcell_explorer/wiki/Summary-of-the-'KinectFusion:-Real-Time-Dense-Surface-Mapping-and-Tracking'-paper) of the KinectFusion paper. For more advanced readers, I apologize for any errors in understanding in the summary link which I have provided. Any suggestions for improvement are most welcome. 

## What is 3-D reconstruction?

3-D reconstruction, in general terms, means creation of a 3-D scene from related 2-D image information. In this article, the term 3-D reconstruction refers to [**Multi-View 3-D reconstruction**](https://vision.in.tum.de/research/image-based_3d_reconstruction/multiviewreconstruction), which means we are using multiple views to reconstruct the 3-D model. There is a special case of 3-D reconstruction called [**Single-View 3-D reconstruction**](https://vision.in.tum.de/research/image-based_3d_reconstruction/singleviewreconstruction) which we are not discussing in this case. 

So, that leads to a question. 

~~~
If images are two-dimensional in nature. How do we make the model 3-D? 
~~~

The answer to that is that we need to rely on **multiple-views** to provide us information about the 3-D geometry. This varies in different cases: 

+ If we were using a monocular camera (eg: a normal USB camera), we have to pan the camera according to the environment (using servo motors probably) and use these multiple views to deduce the depth information. Because of this, the software required to map the environment is pretty complicated and [monocular SLAM algorithms](https://www.doc.ic.ac.uk/~ab9515/introductiontomonocular.html) are generally associated with **sparse** maps. 

+ In our case, we are using an RGB-D camera called [Kinect](https://www.jameco.com/jameco/workshop/howitworks/xboxkinect.html). In addition to the 2-D images, the Kinect sensor provides us a depth frame. A depth frame is an image in which the values of the pixels depend on how far the object is from the camera. Objects near the camera have a darker shade while objects far away have a lighter shade. The objects which are too near (~ 40 cm or lesser) or too far (600 cm and above) are given a pixel value of 0 or black. 

![An Example Depth frame](/images/6_8_2018/depth_map.png)

## General steps involved in 3-D reconstruction

A depth frame, combined with the RGB images from the Kinect, needs to be processed further in order to reconstruct a 3-D environment. 

1. **Camera Calibration**- Before we can get any 3-D information, we need to calibrate our camera. Camera calibration refers to finding the intrinsic and extrinsic parameters of a camera. This will allow us to see 3-D points with reference to the Kinect's camera coordinate frame. For more details on this, you can refer to an excerpt from my [undergrad thesis on Camera Calibration](https://drive.google.com/open?id=1hmw90io50R6-XWrM2tfVmhjSRuq1BLlW). 

2. **Depth determination**- The depth map/depth image generated is converted into [3-D point clouds](https://en.wikipedia.org/wiki/Point_cloud), which are aligned in XYZ axes as per the Kinect point of view. This functionality is provided by the [openni ROS driver](http://wiki.ros.org/openni_launch) for Kinect. 

3. **Depth Registration**- Each view generates a point cloud. Our focus is to do the following things. 

+ Track the position (6-DOF pose) of the camera.  
+ Fuse point clouds from different views into a **dense** volumetric model.


## What is the difference between 3-D reconstruction and SLAM? 

There is a popular area in robotics called [Simulatenous Localisation and Mapping (SLAM)](https://en.wikipedia.org/wiki/Simultaneous_localization_and_mapping) which has received a lot of focus over the years. Both SLAM and 3-D reconstruction use similar technology, the difference is in the goal of each algorithm.

SLAM focuses more on navigation and require maps which are **just dense enough** to navigate in an environment. **Camera Tracking** is an important factor as it helps in the process of localisation of the robot in an unknown environment. 

Generally in 3-D reconstruction, the focus is on reconstructing maps, while camera paths may be discarded if required. 

This answer is a helpful aid in understanding the difference between the two. 

Kinect Fusion is one of the earliest algorithms which aims at optimizing both **camera tracking and the map building process**.

**Note**- I decided to break the blog post into two parts as it would be inconvenient to read such a big blog post. However, both posts are in continuation to each other and talk about the same subject. 

## Resources 

1. R. A. Newcombe et al., "KinectFusion: Real-time dense surface mapping and tracking," 2011 10th IEEE International Symposium on Mixed and Augmented Reality, Basel, 2011, pp. 127-136.
doi: 10.1109/ISMAR.2011.6092378



[^3]: [About]({{ site.url }}/about)


