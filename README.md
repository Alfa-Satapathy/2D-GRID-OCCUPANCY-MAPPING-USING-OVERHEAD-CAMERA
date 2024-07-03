Documentation is avalable at:
    <b>https://github.com/Alfa-Satapathy/2D-GRID-OCCUPANCY-MAPPING-USING-OVERHEAD-CAMERA/blob/main/2D%20Grid%20Mapping%20Using%20Overhead%20Camera.pdf</b>
_____________________________________________________________________________
_____________________________________________________________________________

# 2D GRID OCCUPANCY MAPPING USING OVERHEAD CAMERA - Intel Unnati Industrial Training Program 2024 

## Problem Statement
Robots navigating in unknown environments require accurate mapping and localization to avoid obstacles and plan paths efficiently. Traditional mapping techniques can struggle with dynamic environments and sensor noise. The challenge is to develop a robust system that can generate a real-time 2D occupancy grid from 3D points produced by a monocular vision-based SLAM system, while accurately handling uncertainties in the robot's pose.

## Unique Idea Brief (Solution)
The proposed solution leverages ORB-SLAM for generating 3D points and keyframes, and converts these into a 2D occupancy grid in real-time. The system enhances the accuracy of the grid map by implementing local and global counter techniques, thresholding, Gaussian smoothing, edge detection, and slope thresholding. It integrates ROS for real-time visualization and navigation, and employs probabilistic modeling to account for uncertainties in robot pose and sensor measurements.

## Features Offered

### Real-time 2D Occupancy Grid Generation
Converts 3D points from ORB-SLAM into 2D grid maps in real-time for effective robot navigation.

### Local and Global Counter Enhancement
Prevents misrepresentation of co-linear points, ensuring accurate differentiation between occupied and free spaces by counting observations.

### Thresholding Techniques
- **Visit Thresholding**: Filters out noise by setting a minimum number of observations required for a cell.
- **Height Thresholding**: Filters out points below a certain height, assuming they are part of the floor.

### Gaussian Smoothing
Smooths transitions between free and occupied cells to reduce noise and create a realistic map.

### Canny Edge Detection
Marks obstacle boundaries accurately in the grid map to help the robot identify and avoid obstacles.

### Slope Thresholding
Differentiates between horizontal planes (floor) and obstacles based on slope, ensuring significant height differences are classified as obstacles.

### ROS Integration
Uses ROS nodes to visualize the generated map and support navigation in Rviz, allowing real-time monitoring and interaction with the mapping system.

### Probabilistic Modeling
Utilizes the robot’s pose, pose uncertainties, and a homographic matrix for accurate environmental mapping, addressing sensor and movement uncertainties.

### Image Segmentation
Classifies the environment into "floor" and "non-floor" regions, helping identify free space and obstacles.

### Reclassification and Mapping
Reclassifies non-floor cells as obstacles or occlusive regions and maps these using the homography matrix accurately.

### Pose Uncertainty Handling
Expands obstacle cells based on the robot’s pose uncertainties, improving map accuracy by accounting for potential errors in the robot's position.

## Process Flow

1. **ORB-SLAM Setup**
   - Install and configure ORB-SLAM on the robot. Calibrate the camera for accurate data capture.

2. **Data Reproduction**
   - Reproduce ORB-SLAM results using standard datasets like KITTI and TUM to ensure system functionality.

3. **2D Grid Map Generation**
   - Create a Python script to convert ORB-SLAM’s keyframes and map points into a 2D occupancy grid by projecting 3D points onto a 2D plane.

4. **Visualization**
   - Use ROS nodes to visualize the generated 2D map in Rviz for real-time monitoring and debugging.

5. **Real-time Processing**
   - Convert the Python prototype into a C++ implementation for real-time performance, and create ROS nodes for efficient data handling.

6. **Enhancements Implementation**
   - Apply local/global counters, thresholding, Gaussian smoothing, edge detection, and slope thresholding to improve map accuracy and reliability.

7. **Evaluation**
   - Compare the generated map against ground truth data to measure accuracy and completeness. Adjust parameters to optimize performance.

8. **Homography Matrix Utilization**
   - Calculate the homography matrix at the start of exploration for accurate mapping of image lines to world coordinates.

9. **Segmentation and Classification**
   - Segment the image into "floor" (free space) and "non-floor" (obstacles) regions, and use this classification to update the occupancy grid.

10. **Reclassification**
    - Reclassify non-floor cells as obstacles or occlusive regions, and map these using the homography matrix.

11. **Obstacle Expansion**
    - Expand obstacle cells based on the robot’s pose uncertainties to account for potential errors in the robot’s position.

## Architecture

### ORB-SLAM System
- **Input**: Monocular camera captures the environment.
- **Process**: ORB-SLAM performs feature extraction, matching, pose estimation, and map point generation.

### 2D Grid Map Generation
- **Projection**: Keyframes and map points from ORB-SLAM are projected onto the XZ plane to create a 2D grid.
- **Enhancements**: Apply local/global counters, thresholding, Gaussian smoothing, edge detection, and slope thresholding to the projected points.

### Occupancy Grid Modeling
- **Matrix Calculation**: Calculate the homography matrix for accurate mapping of image lines to world coordinates.
- **Segmentation**: Segment the image into "floor" and "non-floor" regions.
- **Reclassification**: Reclassify non-floor cells as obstacles or occlusive regions.
- **Mapping**: Use the homography matrix to map cells to the world coordinates. Expand obstacle cells based on pose uncertainties.

### ROS Integration
- **Nodes**: Create ROS nodes (monopub and monosub) for publishing and subscribing to map data.
- **Visualization**: Use Rviz and Gazebo for real-time visualization of the map and robot navigation.

### Probabilistic Modeling
- **Sensor Modeling**: Model the camera as a distance sensor using a Gaussian function to calculate cell occupation probabilities.
- **Variance Calculation**: Calculate the sensor’s variance behavior to account for measurement inaccuracies. Use experimentally determined constants to refine this model.

## Technology Used
- **ORB-SLAM**: For feature extraction, matching, pose estimation, and 3D map point generation.
- **Python and C++**: For scripting and real-time implementation of the grid map generation and enhancements.
- **ROS (Robot Operating System)**: For real-time data handling, visualization, and navigation.
- **Rviz and Gazebo**: For visualizing the generated maps and robot navigation.
- **Probabilistic Modeling**: For handling uncertainties in sensor measurements and robot pose.

# ORB-SLAM2


ORB-SLAM2 is a state-of-the-art system for real-time SLAM, capable of building large-scale maps of the environment while simultaneously tracking the camera's position within these maps. It supports various camera configurations, including monocular, stereo, and RGB-D setups, making it versatile for different robotic and augmented reality applications.

Key features of ORB-SLAM2 include:

1. **ORB Features**: It uses the ORB (Oriented FAST and Rotated BRIEF) feature detector and descriptor, which are efficient and robust to changes in viewpoint and lighting conditions. These features are used for keypoint extraction, matching, and tracking across frames.

2. **Simultaneous Localization and Mapping (SLAM)**: ORB-SLAM2 performs real-time localization of the camera (pose estimation) and mapping of the environment concurrently. This is essential for tasks like robot navigation and mapping of unknown environments.

3. **Loop Closure**: It incorporates a loop closing mechanism to detect and close loops in the map, improving accuracy and consistency over time. Loop closure helps in correcting drift and maintaining the integrity of the map.

4. **Relocalization**: ORB-SLAM2 can relocalize the camera within a previously mapped environment if tracking is lost, enabling robust and continuous operation even in challenging conditions where tracking may temporarily fail.

5. **Map Optimization**: The system optimizes the map structure and camera poses to improve accuracy and efficiency. This involves refining the map based on new observations and adjusting the estimated poses of the camera to minimize errors.

6. **Open Source**: ORB-SLAM2 is released under an open-source license, allowing researchers and developers to use, modify, and extend the system for their specific applications. This has contributed to its widespread adoption and continuous improvement in the computer vision and robotics communities.

Overall, ORB-SLAM2 represents a significant advancement in SLAM technology, providing robust, real-time performance suitable for both academic research and practical deployment in autonomous systems and augmented reality platforms.


#2. Prerequisites
We have tested the library in **Ubuntu 12.04**, **14.04** and **16.04**, but it should be easy to compile in other platforms. A powerful computer (e.g. i7) will ensure real-time performance and provide more stable and accurate results.

## C++11 or C++0x Compiler
We use the new thread and chrono functionalities of C++11.

## Pangolin
We use [Pangolin](https://github.com/stevenlovegrove/Pangolin) for visualization and user interface. Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.

## OpenCV
We use [OpenCV](http://opencv.org) to manipulate images and features. Dowload and install instructions can be found at: http://opencv.org. **Required at leat 2.4.3. Tested with OpenCV 2.4.11 and OpenCV 3.2**.

## Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. **Required at least 3.1.0**.

## DBoW2 and g2o (Included in Thirdparty folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the *Thirdparty* folder.

## ROS (optional)
We provide some examples to process the live input of a monocular, stereo or RGB-D camera using [ROS](ros.org). Building these examples is optional. In case you want to use ROS, a version Hydro or newer is needed.

#3. Building ORB-SLAM2 library and TUM/KITTI examples

We provide a script `build.sh` to build the *Thirdparty* libraries and *ORB-SLAM2*. Please make sure you have installed all required dependencies (see section 2). Execute:
```
cd ORB_SLAM2
chmod +x build.sh
./build.sh
```

This will create **libORB_SLAM2.so**  at *lib* folder and the executables **mono_tum**, **mono_kitti**, **rgbd_tum**, **stereo_kitti**, **mono_euroc** and **stereo_euroc** in *Examples* folder.

#4. Monocular Examples

## TUM Dataset

1. Download a sequence from http://vision.in.tum.de/data/datasets/rgbd-dataset/download and uncompress it.

2. Execute the following command. Change `TUMX.yaml` to TUM1.yaml,TUM2.yaml or TUM3.yaml for freiburg1, freiburg2 and freiburg3 sequences respectively. Change `PATH_TO_SEQUENCE_FOLDER`to the uncompressed sequence folder.
```
./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt Examples/Monocular/TUMX.yaml PATH_TO_SEQUENCE_FOLDER
```

## KITTI Dataset  

1. Download the dataset (grayscale images) from http://www.cvlibs.net/datasets/kitti/eval_odometry.php 

2. Execute the following command. Change `KITTIX.yaml`by KITTI00-02.yaml, KITTI03.yaml or KITTI04-12.yaml for sequence 0 to 2, 3, and 4 to 12 respectively. Change `PATH_TO_DATASET_FOLDER` to the uncompressed dataset folder. Change `SEQUENCE_NUMBER` to 00, 01, 02,.., 11. 
```
./Examples/Monocular/mono_kitti Vocabulary/ORBvoc.txt Examples/Monocular/KITTIX.yaml PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER
```

## EuRoC Dataset

1. Download a sequence (ASL format) from http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

2. Execute the following first command for V1 and V2 sequences, or the second command for MH sequences. Change PATH_TO_SEQUENCE_FOLDER and SEQUENCE according to the sequence you want to run.
```
./Examples/Monocular/mono_euroc Vocabulary/ORBvoc.txt Examples/Monocular/EuRoC.yaml PATH_TO_SEQUENCE_FOLDER/mav0/cam0/data Examples/Monocular/EuRoC_TimeStamps/SEQUENCE.txt 
```

```
./Examples/Monocular/mono_euroc Vocabulary/ORBvoc.txt Examples/Monocular/EuRoC.yaml PATH_TO_SEQUENCE/cam0/data Examples/Monocular/EuRoC_TimeStamps/SEQUENCE.txt 
```

#5. Stereo Examples

## KITTI Dataset

1. Download the dataset (grayscale images) from http://www.cvlibs.net/datasets/kitti/eval_odometry.php 

2. Execute the following command. Change `KITTIX.yaml`to KITTI00-02.yaml, KITTI03.yaml or KITTI04-12.yaml for sequence 0 to 2, 3, and 4 to 12 respectively. Change `PATH_TO_DATASET_FOLDER` to the uncompressed dataset folder. Change `SEQUENCE_NUMBER` to 00, 01, 02,.., 11. 
```
./Examples/Stereo/stereo_kitti Vocabulary/ORBvoc.txt Examples/Stereo/KITTIX.yaml PATH_TO_DATASET_FOLDER/dataset/sequences/SEQUENCE_NUMBER
```

## EuRoC Dataset

1. Download a sequence (ASL format) from http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets

2. Execute the following first command for V1 and V2 sequences, or the second command for MH sequences. Change PATH_TO_SEQUENCE_FOLDER and SEQUENCE according to the sequence you want to run.
```
./Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml PATH_TO_SEQUENCE/mav0/cam0/data PATH_TO_SEQUENCE/mav0/cam1/data Examples/Stereo/EuRoC_TimeStamps/SEQUENCE.txt
```
```
./Examples/Stereo/stereo_euroc Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml PATH_TO_SEQUENCE/cam0/data PATH_TO_SEQUENCE/cam1/data Examples/Stereo/EuRoC_TimeStamps/SEQUENCE.txt
```

#6. RGB-D Example

## TUM Dataset

1. Download a sequence from http://vision.in.tum.de/data/datasets/rgbd-dataset/download and uncompress it.

2. Associate RGB images and depth images using the python script [associate.py](http://vision.in.tum.de/data/datasets/rgbd-dataset/tools). We already provide associations for some of the sequences in *Examples/RGB-D/associations/*. You can generate your own associations file executing:

  ```
  python associate.py PATH_TO_SEQUENCE/rgb.txt PATH_TO_SEQUENCE/depth.txt > associations.txt
  ```

3. Execute the following command. Change `TUMX.yaml` to TUM1.yaml,TUM2.yaml or TUM3.yaml for freiburg1, freiburg2 and freiburg3 sequences respectively. Change `PATH_TO_SEQUENCE_FOLDER`to the uncompressed sequence folder. Change `ASSOCIATIONS_FILE` to the path to the corresponding associations file.

  ```
  ./Examples/RGB-D/rgbd_tum Vocabulary/ORBvoc.txt Examples/RGB-D/TUMX.yaml PATH_TO_SEQUENCE_FOLDER ASSOCIATIONS_FILE
  ```

#7. ROS Examples

### Building the nodes for mono, monoAR, stereo and RGB-D
1. Add the path including *Examples/ROS/ORB_SLAM2* to the ROS_PACKAGE_PATH environment variable. Open .bashrc file and add at the end the following line. Replace PATH by the folder where you cloned ORB_SLAM2:

  ```
  export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:PATH/ORB_SLAM2/Examples/ROS
  ```
  
2. Execute `build_ros.sh` script:

  ```
  chmod +x build_ros.sh
  ./build_ros.sh
  ```
  
### Running Monocular Node
For a monocular input from topic `/camera/image_raw` run node ORB_SLAM2/Mono. You will need to provide the vocabulary file and a settings file. See the monocular examples above.

  ```
  rosrun ORB_SLAM2 Mono PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
  ```
  
### Running Monocular Augmented Reality Demo
This is a demo of augmented reality where you can use an interface to insert virtual cubes in planar regions of the scene.
The node reads images from topic `/camera/image_raw`.

  ```
  rosrun ORB_SLAM2 MonoAR PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
  ```
  
### Running Stereo Node
For a stereo input from topic `/camera/left/image_raw` and `/camera/right/image_raw` run node ORB_SLAM2/Stereo. You will need to provide the vocabulary file and a settings file. If you **provide rectification matrices** (see Examples/Stereo/EuRoC.yaml example), the node will recitify the images online, **otherwise images must be pre-rectified**.

  ```
  rosrun ORB_SLAM2 Stereo PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE ONLINE_RECTIFICATION
  ```
  
**Example**: Download a rosbag (e.g. V1_01_easy.bag) from the EuRoC dataset (http://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets). Open 3 tabs on the terminal and run the following command at each tab:
  ```
  roscore
  ```
  
  ```
  rosrun ORB_SLAM2 Stereo Vocabulary/ORBvoc.txt Examples/Stereo/EuRoC.yaml true
  ```
  
  ```
  rosbag play --pause V1_01_easy.bag /cam0/image_raw:=/camera/left/image_raw /cam1/image_raw:=/camera/right/image_raw
  ```
  
Once ORB-SLAM2 has loaded the vocabulary, press space in the rosbag tab. Enjoy!. Note: a powerful computer is required to run the most exigent sequences of this dataset.

### Running RGB_D Node
For an RGB-D input from topics `/camera/rgb/image_raw` and `/camera/depth_registered/image_raw`, run node ORB_SLAM2/RGBD. You will need to provide the vocabulary file and a settings file. See the RGB-D example above.

  ```
  rosrun ORB_SLAM2 RGBD PATH_TO_VOCABULARY PATH_TO_SETTINGS_FILE
  ```
  
#8. Processing your own sequences
You will need to create a settings file with the calibration of your camera. See the settings file provided for the TUM and KITTI datasets for monocular, stereo and RGB-D cameras. We use the calibration model of OpenCV. See the examples to learn how to create a program that makes use of the ORB-SLAM2 library and how to pass images to the SLAM system. Stereo input must be synchronized and rectified. RGB-D input must be synchronized and depth registered.

#9. SLAM and Localization Modes
You can change between the *SLAM* and *Localization mode* using the GUI of the map viewer.

### SLAM Mode
This is the default mode. The system runs in parallal three threads: Tracking, Local Mapping and Loop Closing. The system localizes the camera, builds new map and tries to close loops.

### Localization Mode
This mode can be used when you have a good map of your working area. In this mode the Local Mapping and Loop Closing are deactivated. The system localizes the camera in the map (which is no longer updated), using relocalization if needed. 

## Conclusion
The proposed system provides a robust and efficient solution for generating real-time 2D occupancy grids from monocular vision-based SLAM data. By integrating advanced enhancement techniques and probabilistic modeling, the system ensures accurate mapping and reliable navigation. The use of ROS for visualization and real-time processing further enhances the system’s capabilities, making it a valuable tool for robotic navigation in dynamic and uncertain environments.



