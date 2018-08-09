---
layout: post
title: "Kinect Fusion for 3-D reconstruction- Part 2"
date: "2018-08-08"
slug: "Kinect_Fusion_for_3-D_reconstruction_Part_2"
description: "With this blog post, I want to focus on Kinect Fusion and the salient features behind this successful 3-D reconstruction algorithm"
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

Hello, world! This is the part 2 in the Kinect Fusion for 3-D reconstruction blog post in a series of posts talking about my GSoC 2018 project. The focus of this post is on Kinect Fusion and the principles behind 3-D reconstruction. To the new readers, thanks a lot for reading my post. You can refer to the [first part](https://aadityasaraiya.github.io//blog/2018/08/07/Kinect_Fusion_for_3-D_reconstruction_Part_1/) of this post in order to understand the flow of things in a better manner. 

<!--more-->

**Kinect Fusion for 3-D reconstruction- Part 2** covers the conceptual details of Kinect Fusion in particular as a pipeline for 3-D reconstruction.

The previous part, **Kinect Fusion for 3-D reconstruction- Part 1** of this particular blog post covered the basics of 3-D reconstruction.
 

## Focus of Kinect Fusion

So having developed a general conceptual understanding of 3-D reconstruction, let's shift the focus to Kinect Fusion in particular. The main focus of Kinect Fusion shifts to **both real-time tracking and mapping**

It can be concisely worded out as follows: 


1. **Estimate the relative camera poses (XYZ positions) for different frames in a moving camera.** 

2. **Estimate the geometry (3-D model) of the environment based on the camera frames.**  


## Interleaved components?

So, if we give deeper thought to the focus of Kinect Fusion, it seems that there are two separate processes which need to be done simultanenously. However, thankfully, the two processes **reinforce** each other in a certain way. This can be explained as follows:

+ Using the depth frames with **estimated camera poses**, we build a **dense surface model** of the environment.

![Depiction of the tracking problem](/images/6_8_2018/depict_tracking.png)

+ Using this **dense surface model**, we estimate the **current camera poses** in a better way by aligning the incoming depth frames with the previously obtained dense surface model. 

With the aim of Kinect Fusion clear in our mind, let's move forward to discussing the main features of Kinect Fusion and how the components are successfully achieved. 

## Main features of Kinect Fusion

So here we are, finally. The following image describes the key aspects of Kinect Fusion in the form of a flowchart. 

![Kinect Fusion Flowchart](/images/6_8_2018/Kinfu_flow_chart.png)

___

1. **Depth map conversion**- The depth frame acquired from the Kinect has depth values which suggest how far different objects are from the Kinect sensor. However, all these points need to be transformed into the 3-D coordinate frame with respect to the world. This requires a certain transformation to be made. 

    + Convert depth values to a 3-D point wrt the camera coordinate frame


    v<sub>k</sub>= D<sub>k</sub>(u)K<sup>-1</sup> (Metric point measurement in sensor frame k)

    where D<sub>k</sub>(u)= [x,y,1]<sup>T</sup>

    **Note 1**- A bilateral filter is used before transforming to camera coordinate frame in order to reduce noise. 

    + Transform from camera coordinate frame to the world coordinate frame.

    v<sub>w</sub>= T<sub>w,k</sub>v<sub>k</sub> 

    where T<sub>w,k</sub> is a transformation which helps us to go from the 3-D point in *camera coordinate frame* to a 3-D point in the *world coordinate frame*. A collection of these 3-D points forms what we call a **point cloud**.


    **Note 2**- The robot and the camera have their local coordinate frames. But for us to compare measurements, all values need to be transformed to a world coordinate frame. 

___

2. **Integrating volumes using TSDF Fusion**- There is one important point which I took a bit of time to understand during my initial work with multi-view geometry. **Point clouds** are not continuous **surfaces**. If you zoom into a point cloud, you will see that point clouds have holes/discontinuities. Our task in 3-D reconstruction is to combine/fuse these point clouds into one continuous entity. 

For estimation of the surface geometry, we use something called the [Truncated Signed Distance function](https://en.wikipedia.org/wiki/Signed_distance_function)(TSDF). TSDF assigns certain values to the depth pixels compared to how far they are from the boundaries of the object. The TSDF function has a value of zero at the boundaries, have positive values for depth ranges outside the boundary and negative values for depth ranges inside the boundary.

Hence, TSDF allows us to compute a continuous function whose values is zero at the surfaces and changes inside and outside the surface.

![(left): TSDF value, (right): Object cross-section view in the environment](/images/6_8_2018/tsdf.png)

The TSDF value at each frame needs to be fused into a single TSDF value, which will be a representation of the surface the Kinect sees. Each TSDF value for different scenes is associated with a weight value(which depends on how reliable the point cloud is) and fused it into a single surface. 
 
![Formuala for TSDF fusion](/images/6_8_2018/sdf_fusion.png)

___

3. **Model rendering using raycasting**- So the TSDF fusion process helps in accumulating different TSDF volumes. Now comes the task of rendering the graphics. Instead of rendering continuous surfaces, the TSDF value is discretized (cube-like representation) into basic elements which are called as **voxels**. My project uses the [**Octomap**](https://octomap.github.io/), a probabilistic 3-D mapping package for the process of rendering these voxels. 

![Representation of TSDF in form of voxels](/images/6_8_2018/voxel_rep.png)

The surface rendering is done by a process called [volume raycasting](https://en.wikipedia.org/wiki/Volume_ray_casting) which involves computation of 2-D images from 3-D data (so that they can be displayed on the screen). Ray-casting is a fundamental method in computer vision where geometric rays (these rays are virtual) are traced from the camera to the object in the ray-direction. However, in volume ray-casting, these rays are interpolated into the object as well.  

![Process of ray-casting](/images/6_8_2018/ray_casting.png)

A dense mapping algorithm is utilized to integrate a lot of depth measurements with the rendered surface model.

____ 

4. **Camera tracking using Iterative closest point approach (ICP)**- So as the camera moves and we are acquiring a partially completed surface model, our next task is to track how much the camera has moved. To do that, we use the partially completed surface model to our advantage. The steps to do that are as follows:

+ From the previous camera frames, we have received a partially completed surface model as well as a fused point cloud. 

+ When the next camera frame is received, we align the point cloud of that camera frame to the previous surface model such that the objects(visual features) match. This alignment is done using successive rigid body transformations (rotation + translation) as shown in the figure below using an iterative algorith called Iterative closest point algorithm. This algorithm uses the squared distance between matched points as a metric to minimise (for optimisation).

![Transformation to align point clouds using ICP](/images/6_8_2018/icp.png)


Having computed the rigid body transformation, we are able to track the movement of the camera. 
___

## Summary and whats not possible? 

The beauty behind Kinect Fusion is that with GPU-driven computing (Nvidia CUDA code), the algorithm can do both 3-D reconstruction and camera tracking in real-time. However, the algorithm does have a few limitations.

+ Large-scaled dense 3-D reconstruction is not possible because of memory requirements and drift in camera tracking due to accumulation of error which leads to misaligned point clouds. 

+ The paper, however, suggests that this could be covered up by using KinectFusion with a sub-mapping framework which could add sparsity by using an adaptive grid representation. 

## So, whats next?

I hope this post is able to give a brief idea on the major features of Kinect Fusion and the technical aspects related to the algorithm. So in the next set of posts will cover the following aspects: 


+ Explain the YAK (yet another Kinect Fusion) package which includes details on ROS architecture for this package. [YAK is a package](https://github.com/AustinDeric/yak) published by Austin Deric on a GPU-driven ROS wrapper for implementing Kinect Fusion using TSDFs.  

+ Explain how the robot utilizes the data from the YAK package to choose the **next-best views**  and explore its environment. Certain examples from my simulations will be used to get a better visualization of the same.

I am very thankful to [Austin Deric](https://github.com/AustinDeric), [Joseph Schornak](https://github.com/schornakj) and my mentor Levi Armstrong for guiding me and giving me important hints to understand this package and trying to use it for my project.

## Resources 

I have heavily used the following resources for explaining concepts on the blog post. I have used images from [these course slides](https://courses.cs.washington.edu/courses/cse576/13sp/lectures/ISMAR_lecture.pdf) by Dr. Richard A. Newcombe from the RSE and Grail lab, who is one of the authors of the Kinect Fusion paper. If someone wants to go in depth, I definitely recommend the course slides as well the research paper [1]. 

1. R. A. Newcombe et al., "KinectFusion: Real-time dense surface mapping and tracking," 2011 10th IEEE International Symposium on Mixed and Augmented Reality, Basel, 2011, pp. 127-136.
doi: 10.1109/ISMAR.2011.6092378


[^3]: [About]({{ site.url }}/about)


