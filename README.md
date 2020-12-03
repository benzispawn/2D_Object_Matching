* MP1
  I research in how the ring buffer is implemented and I understood as a waiting line
  so with an array I would take out the first of the line and add the new one at the end.
* MP2 
  I used the exercises and the documentation of OPENCV as a reference to build the diferent 
  detectors.
* MP3
  I stay here for a while as this Rest function was new to me but with time and research I found the right way to implemented.
* MP4
  I used the exercises and the documentation of OPENCV as a reference to build the diferent 
  extractors.
* MP5
  I used the exercises and the documentation of OPENCV as a reference to build the diferent 
  matching methods.
* MP6
  I used the exercises as a reference to build the ratio filtering.
* MP7
  ## RESULTS FROM THE MID-TERM
  HARRIS -> 1737 keypoints - 
  FAST -> 17874 keypoints
  BRISK -> 27116 keypoints
  ORB -> 5000 keypoints
  AKAZE -> 13429 keypoints
  SIFT -> 13862 keypoints

* MP8
  ##RESULTS###
 HARRIS & BRISK - 755 matches
 FAST & BRISK - 8742 matches
 BRISK & BRISK - 12834 matches
 ORB & BRISK - 2759 matches
 AKAZE & BRISK - 8256 matches
 SIFT & BRISK - 4953 matches
         
 HARRIS & ORB - 894 matches
 FAST & ORB - 10491 matches
 BRISK & ORB - 12525 matches
 ORB & ORB - 3031 matches
 AKAZE & ORB - 7993 matches
 SIFT & ORB -  matches
         
 HARRIS & FREAK - 753 matches
 FAST & FREAK - 8274 matches
 BRISK & FREAK - 11911 matches
 ORB & FREAK - 1124 matches
 AKAZE & FREAK - 7836 matches
 SIFT & FREAK - 4737 matches
          
 HARRIS & AKAZE -  matches
 FAST & AKAZE -  matches
 BRISK & AKAZE -  matches
 ORB & AKAZE -  matches
 AKAZE & AKAZE - 9213 matches
 SIFT & AKAZE -  matches
         
 HARRIS & SIFT - 999 matches
 FAST & SIFT - 12359 matches
 BRISK & SIFT - 14741 matches
 ORB & SIFT - 3341 matches
 AKAZE & SIFT - 9449 matches
 SIFT & SIFT - 6964 matches
 
        
* MP9       
 I WOULD RECOMEND DE COMBINATION AKAZE, ORB OR FAST WITH SIFT DESCRIPTOR CAUSE SIFT
 HAS THE LOWEST FALSE POSITIVE AND BY THE DATA OBSERVED IT WAS THE MOST PRECISE WITH 
 THE THREE TYPES OF DETECTORS. SEE THE Book1.XLSX IN THE SRC DIRECTORY FOR THE DATA, IN THE .XLSX WILL SEE THREE TABLES: THE FIRST TWO ARE THE SAME DATA ABOVE AND THE THIRD IS THE RATIO OVER THE NUMBER OF MATCHES BY ALL THE KEYPOINTS. 

# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.


