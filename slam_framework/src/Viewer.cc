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


#include "Viewer.h"

#include <mutex>

namespace TC2LI_SLAM
{

RvizViewer::RvizViewer(ros::NodeHandle& nh, std::string node_name,
                       System* pSystem, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Tracking *pTracking)
        :mpSystem(pSystem), mpFrameDrawer(pFrameDrawer),mpMapDrawer(pMapDrawer), mpTracker(pTracking)
{
    // path.header.frame_id = "world";
    // path.header.stamp = ros::Time::now();
    tracked_mappoints_pub = nh.advertise<sensor_msgs::PointCloud2>(node_name + "/tracked_points", 1);
    all_mappoints_pub = nh.advertise<sensor_msgs::PointCloud2>(node_name + "/all_points", 1);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>(node_name + "/camera_pose", 1);
    pub_odometry = nh.advertise<nav_msgs::Odometry>(node_name + "/visual_odometry", 1000);
    pub_path = nh.advertise<nav_msgs::Path>(node_name + "/opt_path", 1000);
    pub_frame_path = nh.advertise<nav_msgs::Path>(node_name + "/opt_frame_path", 1000);

    pub_tracked_image = nh.advertise<sensor_msgs::Image>(node_name + "/tracked_image", 1000);

    pub_synced_pcl = nh.advertise<sensor_msgs::PointCloud2>("/lidar/synced_pcl", 5);
    pub_keyframe_pcl = nh.advertise<sensor_msgs::PointCloud2>("/lidar/keyframe_pcl", 5);
    pub_opt_pcl = nh.advertise<sensor_msgs::PointCloud2>("/lidar/opt_pcl", 5);
    if(mpTracker->mSensor == mpSystem-> IMU_STEREO_LIDAR)
    {
        Eigen::Matrix3f Rwb_wl;
        Rwb_wl << 0,1,0,
                -1,0,0,
                0,0,1;
        Eigen::Vector3f t(0,0,0);
        Tw2_w1 = Sophus::SE3f(Rwb_wl,t);
    }
    else if(mpTracker->mSensor == mpSystem->STEREO_LIDAR)
    {
        Eigen::Matrix3f Rwc_wl;
        Rwc_wl << 0,0,1,
                -1,0,0,
                0,-1,0;
        Eigen::Vector3f t(0,0,0);
        Tw2_w1 = Sophus::SE3f(Rwc_wl,t);
    }
}

sensor_msgs::PointCloud2 RvizViewer::mappoint_to_pointcloud(std::vector<TC2LI_SLAM::MapPoint*> map_points, ros::Time msg_time)
{
    const int num_channels = 3; // x y z

    if (map_points.size() == 0)
    {
        std::cout << "Map point vector is empty!" << std::endl;
    }
    Eigen::Matrix3f Rw2_w1 = Tw2_w1.rotationMatrix();

    sensor_msgs::PointCloud2 cloud;

    cloud.header.stamp = msg_time;
    cloud.header.frame_id = "base_link";
    cloud.height = 1;
    cloud.width = map_points.size();
    cloud.is_bigendian = false;
    cloud.is_dense = true;
    cloud.point_step = num_channels * sizeof(float);
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.fields.resize(num_channels);

    std::string channel_id[] = { "x", "y", "z"};

    for (int i = 0; i < num_channels; i++)
    {
        cloud.fields[i].name = channel_id[i];
        cloud.fields[i].offset = i * sizeof(float);
        cloud.fields[i].count = 1;
        cloud.fields[i].datatype = sensor_msgs::PointField::FLOAT32;
    }

    cloud.data.resize(cloud.row_step * cloud.height);

    unsigned char *cloud_data_ptr = &(cloud.data[0]);


    for (unsigned int i = 0; i < cloud.width; i++)
    {
        if (map_points[i])
        {
            Eigen::Vector3f P3Dw = map_points[i]->GetWorldPos();

            P3Dw = Rw2_w1*P3Dw;


            tf::Vector3 point_translation(P3Dw.x(), P3Dw.y(), P3Dw.z());

            float data_array[num_channels] = {
                    point_translation.x(),
                    point_translation.y(),
                    point_translation.z()
            };

            memcpy(cloud_data_ptr+(i*cloud.point_step), data_array, num_channels*sizeof(float));
        }
    }
    return cloud;
}

void RvizViewer::publish_tracked_points(std::vector<TC2LI_SLAM::MapPoint*> tracked_points, ros::Time msg_time)
{
    sensor_msgs::PointCloud2 cloud = mappoint_to_pointcloud(tracked_points, msg_time);

    tracked_mappoints_pub.publish(cloud);
}

void RvizViewer::publish_opt_frame_path(std::vector<TC2LI_SLAM::KeyFrame*> opt_keyframes, std::list<TC2LI_SLAM::KeyFrame *> ref_keyframes,
                                        std::list<bool> lost_keyframes, std::list<Sophus::SE3f> ref_poses)
{
    nav_msgs::Path opt_path;
    ros::Time msg_time = ros::Time::now();
    opt_path.header.stamp = msg_time;
    opt_path.header.frame_id = "base_link";
    sort(opt_keyframes.begin(),opt_keyframes.end(),KeyFrame::lId);

    list<TC2LI_SLAM::KeyFrame*>::iterator lRit = ref_keyframes.begin();
    list<bool>::iterator lbL = lost_keyframes.begin();
    for(list<Sophus::SE3f>::iterator lit = ref_poses.begin(),
                lend=ref_poses.end();lit!=lend;lit++, lRit++, lbL++)
    {
        if(*lbL)
            continue;

        KeyFrame* pKF = *lRit;

        Sophus::SE3f Trw;

        while(pKF->isBad())
        {
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
        }
        Trw = Trw * pKF->GetPose();

        Sophus::SE3f Tcw = (*lit) * Trw;
        Sophus::SE3f Twc = Tw2_w1 * Tcw.inverse();

        Eigen::Vector3f twc = Twc.translation();
        Eigen::Quaternionf q = Twc.unit_quaternion();

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = msg_time;
        pose_stamped.header.frame_id = "base_link";
        pose_stamped.pose.position.x = twc.x();
        pose_stamped.pose.position.y = twc.y();
        pose_stamped.pose.position.z = twc.z();
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        opt_path.poses.push_back(pose_stamped);
    }

    pub_frame_path.publish(opt_path);
}

void RvizViewer::publish_opt_path(std::vector<TC2LI_SLAM::KeyFrame*> opt_keyframes)
{
    nav_msgs::Path opt_path;
    ros::Time msg_time = ros::Time::now();
    opt_path.header.stamp = msg_time;
    opt_path.header.frame_id = "base_link";
    sort(opt_keyframes.begin(),opt_keyframes.end(),KeyFrame::lId);

    for(TC2LI_SLAM::KeyFrame* &kf : opt_keyframes)
    {
        Sophus::SE3f Twc = Tw2_w1*(kf->GetPoseInverse());

        Eigen::Vector3f twc = Twc.translation();
        Eigen::Quaternionf q = Twc.unit_quaternion();

        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = msg_time;
        pose_stamped.header.frame_id = "base_link";
        pose_stamped.pose.position.x = twc.x();
        pose_stamped.pose.position.y = twc.y();
        pose_stamped.pose.position.z = twc.z();
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        opt_path.poses.push_back(pose_stamped);

    }

    pub_path.publish(opt_path);
}

void RvizViewer::publish_all_points(std::vector<TC2LI_SLAM::MapPoint*> map_points, ros::Time msg_time)
{
    sensor_msgs::PointCloud2 cloud = mappoint_to_pointcloud(map_points, msg_time);

    all_mappoints_pub.publish(cloud);
}

void RvizViewer::publish_camera_pose(Sophus::SE3f Tcw_SE3f, ros::Time msg_time)
{
    Eigen::Quaternionf q;
    Sophus::SE3f Twc = Tw2_w1*(Tcw_SE3f.inverse());

    geometry_msgs::PoseStamped pose_msg;

    pose_msg.header.frame_id = "base_link";
    pose_msg.header.stamp = msg_time;

    pose_msg.pose.position.x = Twc.translation().x();
    pose_msg.pose.position.y = Twc.translation().y();
    pose_msg.pose.position.z = Twc.translation().z();

    q = Twc.unit_quaternion();
    pose_msg.pose.orientation.w = q.coeffs().w();
    pose_msg.pose.orientation.x = q.coeffs().x();
    pose_msg.pose.orientation.y = q.coeffs().y();
    pose_msg.pose.orientation.z = q.coeffs().z();

    pose_pub.publish(pose_msg);

    //* Odometry
    nav_msgs::Odometry odometry;
    odometry.header.stamp = msg_time;
    odometry.header.frame_id = "base_link";
    odometry.child_frame_id = "slam_pose";
    Eigen::Quaternionf tmp_Q;
    tmp_Q = q;
    odometry.pose.pose.position.x = Twc.translation().x();
    odometry.pose.pose.position.y = Twc.translation().y();
    odometry.pose.pose.position.z = Twc.translation().z();
    odometry.pose.pose.orientation.x = tmp_Q.x();
    odometry.pose.pose.orientation.y = tmp_Q.y();
    odometry.pose.pose.orientation.z = tmp_Q.z();
    odometry.pose.pose.orientation.w = tmp_Q.w();
    pub_odometry.publish(odometry);

    //* TF
    static tf::TransformBroadcaster br;
    tf::Quaternion q_tf;


    Eigen::Quaternionf q2;
    q2 = Twc.unit_quaternion();
    q_tf.setX(q2.x());
    q_tf.setY(q2.y());
    q_tf.setZ(q2.z());
    q_tf.setW(q2.w());

    tf::Transform t_odom_to_lidar;
    t_odom_to_lidar.setOrigin(tf::Vector3(Twc.translation().x(), Twc.translation().y(), Twc.translation().z()));
    t_odom_to_lidar.setRotation(q_tf);
    tf::StampedTransform trans_odom_to_lidar = tf::StampedTransform(t_odom_to_lidar, msg_time, "base_link", "visal_odom");
    br.sendTransform(trans_odom_to_lidar);
}
void RvizViewer::publish_tracked_image()
{
    float trackedImageScale = mpTracker->GetImageScale();
    cv_bridge::CvImage cv_image;
    cv_image.image = mpFrameDrawer->DrawFrame(trackedImageScale);
    cv_image.encoding = "bgr8";

    sensor_msgs::Image ros_image;
    cv_image.toImageMsg(ros_image);
    pub_tracked_image.publish(ros_image);
}

void RvizViewer::publish_all(const Frame &frame)
{
    ros::Time msg_time = ros::Time(frame.mTimeStamp);
    Sophus::SE3f Tcw = frame.GetPose();
    publish_tracked_points(frame.mvpMapPoints, msg_time);
    publish_camera_pose(Tcw, msg_time);

    publish_tracked_image();

    if(mpTracker->mSensor == mpSystem-> IMU_STEREO_LIDAR && !mpMapDrawer->mpAtlas->GetIniertialBA2())
        return;
    Sophus::SE3f Twl = Tcw.inverse() * LIDAR::LidarFrontEndTools::mLidarParam->mTcl;
    if (frame.mLidarProps->GetSurfacePcl()->size() > 0)
        publish_keyframe_pcl(Twl, frame.mLidarProps->GetSurfacePcl(), msg_time);

    // from double to ros::time

}

template<typename T>
void RvizViewer::publish_keyframe_pcl(Sophus::SE3f Twl_SE3f, T outCloud, ros::Time thisStamp)
{
    // cout << "publish pcl" << endl;
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*TC2LI_SLAM::LIDAR::LidarFrontEndTools::transformPointCloud(outCloud, Tw2_w1 * Twl_SE3f), tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = "base_link";
    pub_keyframe_pcl.publish(tempCloud);
}

} // namespace TC2LI_SLAM
