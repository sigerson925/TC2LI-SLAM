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

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>

#include<opencv2/core/core.hpp>
// #include "lidar_front_end/LidarFrontEnd.h"
#include"System.h"
#include"LidarTypes.h"
#include "Viewer.h"

using namespace std;

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
    ImageGrabber(TC2LI_SLAM::System *pSLAM, PointCloudGrabber *pPointCloudGb) : mpSLAM(pSLAM),
                                                                                mpPointCloudGb(pPointCloudGb) {}

    void GrabStereo(const sensor_msgs::ImageConstPtr &msgLeft, const sensor_msgs::ImageConstPtr &msgRight);

    TC2LI_SLAM::System *mpSLAM;
    PointCloudGrabber *mpPointCloudGb;
    cv::Mat M1l, M2l, M1r, M2r;
};

int latestCameraId = 0;
double latestCameraTime = -1.0;
double cameraFps = -1.0;
double halfCameraFreq = 1 / (cameraFps * 2);
int liDARId = -1;

int main(int argc, char **argv) {
    ros::init(argc, argv, "Stereo-Lidar");
    ros::start();
    std::thread thProcess(&LidarCameraProcess);

    if (argc > 1) {
        ROS_WARN ("Arguments supplied via command line are ignored.");
    }
    std::string node_name = ros::this_node::getName();
    ros::NodeHandle nh;
    std::string voc_file, settings_file;
    nh.param<std::string>(node_name + "/voc_file", voc_file, "file_not_set");
    nh.param<std::string>(node_name + "/settings_file", settings_file, "file_not_set");
    if (voc_file == "file_not_set" || settings_file == "file_not_set") {
        ROS_ERROR("Please provide voc_file and settings_file in the launch file");
        ros::shutdown();
        return 1;
    }
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    TC2LI_SLAM::System SLAM(voc_file, settings_file, TC2LI_SLAM::System::STEREO_LIDAR, true);

    SLAM.InitRvizViewers(nh, node_name);
    SetSeneorMode(TC2LI_SLAM::System::STEREO_LIDAR);

    PointCloudGrabber pointcloudgb;
    ImageGrabber igb(&SLAM, &pointcloudgb);

    message_filters::Subscriber<sensor_msgs::Image> left_sub(nh, "/camera/left/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> right_sub(nh, "/camera/right/image_raw", 1);
    ros::Subscriber sub_pointcloud = nh.subscribe("/velodyne_points", 100, &PointCloudGrabber::GrabPointCloud,
                                                  &pointcloudgb);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), left_sub, right_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabStereo, &igb, _1, _2));
    cv::FileStorage fSettings(settings_file, cv::FileStorage::READ);
    cv::FileNode node = fSettings["Camera.fps"];
    if (!node.empty() && node.isReal()) {
        cameraFps = node.real();
    } else {
        std::cerr << "*Camera.fps parameter doesn't exist or is not an integer*" << std::endl;
    }
    halfCameraFreq = 1 / (cameraFps * 2);

    ros::spin();
    SLAM.Shutdown();
    stop_lidar = true;
    thProcess.join();

    SLAM.SaveKeyFrameTrajectoryTUM("/home/dianel/SLAM/results/stereo-lidar_no_loop/KeyFrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryTUM("/home/dianel/SLAM/results/stereo-lidar_no_loop/FrameTrajectory_TUM_Format.txt");
    SLAM.SaveTrajectoryKITTI("/home/dianel/SLAM/results/stereo-lidar_no_loop/FrameTrajectory_KITTI_Format.txt");
    ros::shutdown();
    return 0;
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
        --liDARId;
    }
    if (timeDiff > halfCameraFreq && latestCameraTime > 0) {
        ++liDARId;
    }
    //* transfer pcl to lidar front end
    standard_pcl_cbk(laserCloudMsg, liDARId, lidarTime);
    mBufMutexPC.unlock();
    return;
}


sensor_msgs::PointCloud2 msg;
cv_bridge::CvImageConstPtr cv_ptrLeft;
cv_bridge::CvImageConstPtr cv_ptrRight;
cv::Mat imLeft;
cv::Mat imRight;

void ImageGrabber::GrabStereo(const sensor_msgs::ImageConstPtr &msgLeft, const sensor_msgs::ImageConstPtr &msgRight) {
    try {
        cv_ptrLeft = cv_bridge::toCvShare(msgLeft);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    try {
        cv_ptrRight = cv_bridge::toCvShare(msgRight);
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    double tImLeft = cv_ptrLeft->header.stamp.toSec();
    mpPointCloudGb->mBufMutexPC.lock();

    latestCameraId = TC2LI_SLAM::Frame::nNextId; //* current imag id
    latestCameraTime = tImLeft;

    mpPointCloudGb->mBufMutexPC.unlock();
    //* front end process
    Sophus::SE3f Tcw = mpSLAM->TrackStereoLidar(cv_ptrLeft->image, cv_ptrRight->image, tImLeft);
    Sophus::SE3f Twl = Tcw.inverse() * TC2LI_SLAM::LIDAR::LidarFrontEndTools::mLidarParam->mTcl;

    //* publish msgs
    mpSLAM->PublishMsgs(cv_ptrLeft->header.stamp);

}