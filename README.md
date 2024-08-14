# TC2LI-SLAM
TC2LI-SLAM: A Tightly-Coupled Camera-LiDAR-Inertial SLAM System
## 1. Dependencies
We have tested the library in Ubuntu 20.04.
### PCL
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html). Tested with PCL 1.12.0.

### OpenCV
Download and install instructions can be found at: http://opencv.org. Tested with OpenCV 4.2.0.

### Eigen3
Required by g2o (see below). Download and install instructions can be found at: http://eigen.tuxfamily.org. Required at least 3.3.7.

### DBoW2 and g2o (Included in Thirdparty folder)
We use modified versions of the [DBoW2](https://github.com/dorian3d/DBoW2) library to perform place recognition and [g2o](https://github.com/RainerKuemmerle/g2o) library to perform non-linear optimizations. Both modified libraries (which are BSD) are included in the Thirdparty folder.

### Sophus (Included in Thirdparty folder)
We use a modified version of [Sophus](https://github.com/strasdat/Sophus). It is included in the Thirdparty folder.

## 2. Build & Run
```bash
mkdir -p ~/$Workspace_DIR$/src
cd ~/$Workspace_DIR$/src
git clone https://github.com/sigerson925/TC2LI-SLAM.git
cd ..
git catkin build
source devel/setup.bash
roslauch tc2li_slam $Launch_File_Name$
```