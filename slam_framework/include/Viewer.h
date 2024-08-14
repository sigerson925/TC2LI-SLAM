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


#ifndef VIEWER_H
#define VIEWER_H

#include "Frame.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Tracking.h"
#include "System.h"
#include "Settings.h"
//* For Rviz Viewer
#include<cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <mutex>

namespace TC2LI_SLAM
{
class Frame;
class Tracking;
class FrameDrawer;
class MapDrawer;
class System;
class Settings;

class RvizViewer
{
public:
    RvizViewer(ros::NodeHandle& nh, std::string node_name,
               System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Tracking *pTracking);

    sensor_msgs::PointCloud2 mappoint_to_pointcloud(std::vector<TC2LI_SLAM::MapPoint*> map_points, ros::Time msg_time);
    void publish_tracked_points(std::vector<TC2LI_SLAM::MapPoint*> tracked_points, ros::Time msg_time);
    void publish_opt_frame_path(std::vector<TC2LI_SLAM::KeyFrame*> opt_keyframes, std::list<TC2LI_SLAM::KeyFrame *> ref_keyframes,
                                std::list<bool> lost_keyframes, std::list<Sophus::SE3f> ref_poses);
    void publish_opt_path(std::vector<TC2LI_SLAM::KeyFrame*> opt_keyframes);
    void publish_all_points(std::vector<TC2LI_SLAM::MapPoint*> map_points, ros::Time msg_time);
    void publish_camera_pose(Sophus::SE3f Tcw_SE3f, ros::Time msg_time);
    void publish_all(const Frame &frsame);
    void publish_tracked_image();
    template<typename T>
    void publish_keyframe_pcl(Sophus::SE3f Twl_SE3f, T outCloud, ros::Time thisStamp);

private:
    System* mpSystem;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    Tracking* mpTracker;

    ros::Publisher tracked_mappoints_pub, all_mappoints_pub, pose_pub, pub_odometry, pub_path, pub_frame_path;
    ros::Publisher pub_tracked_image;
    ros::Publisher pub_synced_pcl, pub_keyframe_pcl, pub_opt_pcl;
    nav_msgs::Path path;

    Sophus::SE3f Tw2_w1; //* lidar world to camera/imu world

};
}


#endif // VIEWER_H
	

