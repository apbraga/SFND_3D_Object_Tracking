# SFND 3D Object Tracking

Welcome to the final project of the camera course. By completing all the lessons, you now have a solid understanding of keypoint detectors, descriptors, and methods to match them between successive images. Also, you know how to detect objects in an image using the YOLO deep-learning framework. And finally, you know how to associate regions in a camera image with Lidar points in 3D space. Let's take a look at our program schematic to see what we already have accomplished and what's still missing.

<img src="images/course_code_structure.png" width="779" height="414" />

In this final project, you will implement the missing parts in the schematic. To do this, you will complete four major tasks: 
1. First, you will develop a way to match 3D objects over time by using keypoint correspondences. 
2. Second, you will compute the TTC based on Lidar measurements. 
3. You will then proceed to do the same using the camera, which requires to first associate keypoint matches to regions of interest and then to compute the TTC based on those matches. 
4. And lastly, you will conduct various tests with the framework. Your goal is to identify the most suitable detector/descriptor combination for TTC estimation and also to search for problems that can lead to faulty measurements by the camera or Lidar sensor. In the last course of this Nanodegree, you will learn about the Kalman filter, which is a great way to combine the two independent TTC measurements into an improved version which is much more reliable than a single sensor alone can be. But before we think about such things, let us focus on your final project in the camera course. 

## Key performance impactors for Lidar TTC
Faulty TTC estimation may be caused by the missleading points in the cloud that are not in the end of the vehicle, for example one may use points from vehicle side mirrors as reference for TTC calculation if not filtered properly.
Similarly, since the image bounding box is quite large it may include the vehicle shadow in the road surface, if there are any remaining Lidar points that were not filtered during road separation it will lead to miss estimation of TTC, results on a estimation whre the vehicle is closer than it really is.

## Key performance impactors for Camera TTC
As in Lidar, portiong of the object bounding box scope may include portion that don't reflect the vehicle movement, for example other object, vehicle, or road may be contatined partially in the vehicle bounding box, and its edges may the them used for the estimatation leading to a faulty TTC.

##Keypoint & matchers Performance Analysis
Running over all detectors and descriptors, it is calculate the error of Lidar estimated TTC and Camera estimated TTC, in the table bellow the all frames average error between detector and descriptor is presented.
Akaze detector with brisk, freak and or orb descriptor presented the top 3 performance, when using error minimization as criteria, thus is the recommendation.

| error \|CAMERA - LIDAR\| [s] | DESCRIPTOR | <-    | <-    | <-     | <-    | <-   |
|--------------------------|------------|-------|-------|--------|-------|------|
| DETECTOR                 | AKAZE      | BRIEF | BRISK | FREAK  | ORB   | SIFT |
| AKAZE                    | 1.21       | 1.50  | 1.10  | 1.16   | 1.18  | 1.46 |
| BRISK                    | -          | 3.62  | 2.96  | 3.15   | 3.43  | 4.92 |
| FAST                     | -          | 2.83  | 3.33  | 3.39   | 3.25  | 4.61 |
| HARRIS                   | -          | 4.81  | 14.76 | 18.85  | 9.66  | 6.44 |
| ORB                      | -          | 15.63 | 33.65 | 185.61 | 17.56 | 7.31 |
| SHITOMASI                | -          | 2.27  | 2.67  | 2.85   | 1.81  | 2.12 |
| SIFT                     | -          | 1.23  | 2.73  | 3.09   | -     | 1.81 |

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* Git LFS
  * Weight files are handled using [LFS](https://git-lfs.github.com/)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking`.
