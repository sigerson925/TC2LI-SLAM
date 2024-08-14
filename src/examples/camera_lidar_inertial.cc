/*
 *  This file is part of TC2LI-SLAM
 *
 *  Copyright (C) 2024 Yunze Tong, Nankai University.
 *  Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 *  Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 *  Copyright (C) 2022 Wei Xu, Yixi Cai, Dongjiao He, Jiarong Lin, and Fu Zhang, University of Hong Kong.
 *  Copyright (C) 2023 Zheng Liu, Xiyuan Liu, and Fu Zhang, University of Hong Kong.
 *
 *  TC2LI-SLAM is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  TC2LI-SLAM is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with TC2LI-SLAM. If not, see <http://www.gnu.org/licenses/>.
 *
 *  Portions of TC2LI-SLAM are derived from FAST-LIO and BALM, which is
 *  licensed under the GNU General Public License v2.0, and from
 *  ORB-SLAM3, which is licensed under the GNU General Public License v3.0.
 */

#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>
#include<vector>
#include<queue>
#include<thread>
#include<mutex>

#include<ros/ros.h>
#include<cv_bridge/cv_bridge.h>
#include<sensor_msgs/Imu.h>

#include<opencv2/core/core.hpp>

#include"System.h"
#include"ImuTypes.h"
#include "Viewer.h"

using namespace std;

class ImuGrabber {
public:
    ImuGrabber() {};

    void GrabImu(const sensor_msgs::ImuConstPtr &imu_msg);

    queue<sensor_msgs::ImuConstPtr> imuBuf;
    std::mutex mBufMutex;
};

class PointCloudGrabber {
public:
    PointCloudGrabber() {}

    void GrabPointCloud(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg);

    queue<sensor_msgs::PointCloud2> pointCloudBuf;
    sensor_msgs::PointCloud2 currentCloudMsg;
    std_msgs::Header cloudHeader;
    std::mutex mBufMutexPC;
};

class ImageGrabber {
public:
    ImageGrabber(TC2LI_SLAM::System *pSLAM, ImuGrabber *pImuGb, PointCloudGrabber *pPointCloudGb, const bool bClahe)
            : mpSLAM(pSLAM), mpImuGb(pImuGb), mpPointCloudGb(pPointCloudGb), mbClahe(bClahe) {}

    void GrabImageLeft(const sensor_msgs::ImageConstPtr &msg);

    void GrabImageRight(const sensor_msgs::ImageConstPtr &msg);

    cv::Mat GetImage(const sensor_msgs::ImageConstPtr &img_msg);

    void SyncWithImu();

    queue<sensor_msgs::ImageConstPtr> imgLeftBuf, imgRightBuf;
    std::mutex mBufMutexLeft, mBufMutexRight;

    TC2LI_SLAM::System *mpSLAM;
    ImuGrabber *mpImuGb;

    PointCloudGrabber *mpPointCloudGb;

    ros::Publisher *mpPclPub;
    cv::Mat M1l, M2l, M1r, M2r;

    const bool mbClahe;
    cv::Ptr<cv::CLAHE> mClahe = cv::createCLAHE(3.0, cv::Size(8, 8));
};

double cameraFps = -1.0;
sensor_msgs::PointCloud2 msg;
int latestCameraId = 0;
double latestCameraTime = -1.0;
double halfCameraFreq = 1 / (cameraFps * 2);
int liDARId = -1;

int main(int argc, char **argv) {
    ros::init(argc, argv, "Stereo_Inertial");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
    std::thread thProcess(&LidarInertialProcess);
    bool bEqual = false;

    if (argc > 1) {
        ROS_WARN ("Arguments supplied via command line are ignored.");
    }
    std::string node_name = ros::this_node::getName();
    std::string voc_file, settings_file;
    n.param<std::string>(node_name + "/voc_file", voc_file, "file_not_set");
    n.param<std::string>(node_name + "/settings_file", settings_file, "file_not_set");

    if (voc_file == "file_not_set" || settings_file == "file_not_set") {
        ROS_ERROR("Please provide voc_file and settings_file in the launch file");
        ros::shutdown();
        return 1;
    }
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    TC2LI_SLAM::System SLAM(voc_file, settings_file, TC2LI_SLAM::System::IMU_STEREO_LIDAR, true);

    SLAM.InitRvizViewers(n, node_name);
    SetSeneorMode(TC2LI_SLAM::System::IMU_STEREO_LIDAR);

    ImuGrabber imugb;


    PointCloudGrabber pointcloudgb;

    ImageGrabber igb(&SLAM, &imugb, &pointcloudgb, bEqual);

    ros::Subscriber sub_imu = n.subscribe("/imu", 1000, &ImuGrabber::GrabImu, &imugb);
    ros::Subscriber sub_img_left = n.subscribe("/camera/left/image_raw", 1, &ImageGrabber::GrabImageLeft, &igb);
    ros::Subscriber sub_img_right = n.subscribe("/camera/right/image_raw", 1, &ImageGrabber::GrabImageRight, &igb);
    ros::Subscriber sub_pointcloud = n.subscribe("/velodyne_points", 100, &PointCloudGrabber::GrabPointCloud,
                                                 &pointcloudgb);

    std::thread sync_thread(&ImageGrabber::SyncWithImu, &igb);

    cv::FileStorage fSettings(settings_file, cv::FileStorage::READ);
    cv::FileNode node = fSettings["Camera.fps"];
    if (!node.empty() && node.isReal()) {
        cameraFps = node.real();
    } else {
        std::cerr << "*Camera.fps parameter doesn't exist or is not an integer*" << std::endl;
    }

    halfCameraFreq = 1 / (cameraFps * 2);

    ros::spin();
    stop_lidar = true;
    thProcess.join();
    SLAM.Shutdown();
    SLAM.SaveKeyFrameTrajectoryTUM(
            "/home/dianel/SLAM/results/stereo-inertial-lidar_no_loop/KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("/home/dianel/SLAM/results/stereo-inertial-lidar_no_loop/FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI(
            "/home/dianel/SLAM/results/stereo-inertial-lidar_no_loop/FrameTrajectory_KITTI_Format.txt");
    ros::shutdown();
    return 0;
}


void ImageGrabber::GrabImageLeft(const sensor_msgs::ImageConstPtr &img_msg) {
    static int ditch_count = 0;
    mBufMutexLeft.lock();
    if (!imgLeftBuf.empty()) {
        imgLeftBuf.pop();
    }
    imgLeftBuf.push(img_msg);
    mBufMutexLeft.unlock();
}

void ImageGrabber::GrabImageRight(const sensor_msgs::ImageConstPtr &img_msg) {
    mBufMutexRight.lock();
    if (!imgRightBuf.empty())
        imgRightBuf.pop();
    imgRightBuf.push(img_msg);
    mBufMutexRight.unlock();
}

cv::Mat ImageGrabber::GetImage(const sensor_msgs::ImageConstPtr &img_msg) {
    cv_bridge::CvImageConstPtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    if (cv_ptr->image.type() == 0) {
        return cv_ptr->image.clone();
    } else {
        std::cout << "Error type" << std::endl;
        return cv_ptr->image.clone();
    }
}

void ImageGrabber::SyncWithImu() {
    const double maxTimeDiff = 0.1;
    while (1) {
        cv::Mat imLeft, imRight;
        ros::Time timestamp;
        double tImLeft = 0, tImRight = 0, tCloud = -1;

        // check if there are images and imu data
        if (!imgLeftBuf.empty() && !imgRightBuf.empty() && !mpImuGb->imuBuf.empty()) {
            // get the timestamp of the images
            timestamp = imgLeftBuf.front()->header.stamp;
            tImLeft = timestamp.toSec();
            tImRight = imgRightBuf.front()->header.stamp.toSec();

            // make sure the images are not too old
            this->mBufMutexRight.lock();
            while ((tImLeft - tImRight) > maxTimeDiff && imgRightBuf.size() > 1) {
                imgRightBuf.pop();
                tImRight = imgRightBuf.front()->header.stamp.toSec();
            }
            this->mBufMutexRight.unlock();

            this->mBufMutexLeft.lock();
            while ((tImRight - tImLeft) > maxTimeDiff && imgLeftBuf.size() > 1) {
                imgLeftBuf.pop();
                tImLeft = imgLeftBuf.front()->header.stamp.toSec();
            }
            this->mBufMutexLeft.unlock();

            // check if the images are synchronized
            if ((tImLeft - tImRight) > maxTimeDiff || (tImRight - tImLeft) > maxTimeDiff) {
                continue;
            }

            // check if the imu data is too old
            if (tImLeft > mpImuGb->imuBuf.back()->header.stamp.toSec()) {
                continue;
            }

            // get image
            this->mBufMutexLeft.lock();
            imLeft = GetImage(imgLeftBuf.front());
            imgLeftBuf.pop();
            this->mBufMutexLeft.unlock();

            this->mBufMutexRight.lock();
            imRight = GetImage(imgRightBuf.front());
            imgRightBuf.pop();
            this->mBufMutexRight.unlock();

            // get imu data
            vector<TC2LI_SLAM::IMU::Point> vImuMeas;
            mpImuGb->mBufMutex.lock();
            if (!mpImuGb->imuBuf.empty()) {
                vImuMeas.clear();
                while (!mpImuGb->imuBuf.empty() && mpImuGb->imuBuf.front()->header.stamp.toSec() <= tImLeft) {
                    double t = mpImuGb->imuBuf.front()->header.stamp.toSec();
                    cv::Point3f acc(mpImuGb->imuBuf.front()->linear_acceleration.x,
                                    mpImuGb->imuBuf.front()->linear_acceleration.y,
                                    mpImuGb->imuBuf.front()->linear_acceleration.z);
                    cv::Point3f gyr(mpImuGb->imuBuf.front()->angular_velocity.x,
                                    mpImuGb->imuBuf.front()->angular_velocity.y,
                                    mpImuGb->imuBuf.front()->angular_velocity.z);
                    vImuMeas.push_back(TC2LI_SLAM::IMU::Point(acc, gyr, t));
                    mpImuGb->imuBuf.pop();
                }
            }
            mpImuGb->mBufMutex.unlock();

            if (mbClahe) {
                mClahe->apply(imLeft, imLeft);
                mClahe->apply(imRight, imRight);
            }

            mpPointCloudGb->mBufMutexPC.lock();
            latestCameraId = TC2LI_SLAM::Frame::nNextId; //* current camera id
            latestCameraTime = tImLeft;
            mpPointCloudGb->mBufMutexPC.unlock();

            //* front end process
            Sophus::SE3f Tcw = mpSLAM->TrackStereoLidar(imLeft, imRight, tImLeft, vImuMeas);
            //* publish msgs
            mpSLAM->PublishMsgs(timestamp);

            std::chrono::milliseconds tSleep(1);
            std::this_thread::sleep_for(tSleep);
        }
    }
}

void ImuGrabber::GrabImu(const sensor_msgs::ImuConstPtr &imu_msg) {
    mBufMutex.lock();
    imu_cbk(imu_msg);
    imuBuf.push(imu_msg);
    mBufMutex.unlock();
    return;
}

void PointCloudGrabber::GrabPointCloud(const sensor_msgs::PointCloud2ConstPtr &laserCloudMsg) {

    mBufMutexPC.lock();
    pcl::PointCloud<velodyne_ros::Point> cloud_msg;
    pcl::fromROSMsg(*laserCloudMsg, cloud_msg);
    double lidarTime = laserCloudMsg->header.stamp.toSec() + cloud_msg.points.back().time;
    liDARId = latestCameraId;
    double timeDiff = lidarTime - latestCameraTime;
    //* if lidar comes first
    if (timeDiff < 0 && -timeDiff > halfCameraFreq) {
        do {
            --liDARId;
            timeDiff += (2 * halfCameraFreq);
        } while (timeDiff < 0);

    } else if (timeDiff > halfCameraFreq && latestCameraTime > 0) {
        do {
            ++liDARId;
            timeDiff -= (2 * halfCameraFreq);
        } while (timeDiff > 0);
    }

    //* transfer pcl to lidar front end
    standard_pcl_cbk(laserCloudMsg, liDARId, lidarTime);
    mBufMutexPC.unlock();
    return;
}