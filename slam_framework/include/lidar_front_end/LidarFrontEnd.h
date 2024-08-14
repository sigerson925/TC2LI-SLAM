
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
#ifndef LIDARFRONTEND_H
#define LIDARFRONTEND_H
// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <lidar_front_end/so3_math.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <lidar_front_end/IMU_Processing.hpp>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include "lidar_front_end/preprocess.h"
#include <ikd-Tree/ikd_Tree.h>
#include "LidarTypes.h"

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

/*** Time Log Variables ***/
extern double kdtree_incremental_time, kdtree_search_time, kdtree_delete_time;
extern double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
extern double match_time, solve_time, solve_const_H_time;
extern int kdtree_size_end, add_point_size, kdtree_delete_counter;
extern bool time_sync_en, extrinsic_est_en;
/**************************/

/*** Sync Variables *******/
enum Sensor{
        STEREO_LIDAR,
        IMU_STEREO_LIDAR,
    };
extern int sensor;
extern int mCurrCameraId;
extern double mCurrTimeStamp;
extern pcl::PointCloud<pcl::PointXYZINormal>::Ptr mCurrFeatPoints;
extern pcl::PointCloud<pcl::PointXYZINormal>::Ptr GetCurrFeatPoints();
extern std::mutex syncMutex;
extern bool stop_lidar;
extern bool track_ends;
extern int GetCurrCameraId();
extern double GetCurrTimeStamp();
void SetSeneorMode(int sensorMode);
/**************************/
extern float res_last[100000];
extern float DET_RANGE;
extern const float MOV_THRESHOLD;
extern double time_diff_lidar_to_imu;

extern mutex mtx_buffer;
extern condition_variable sig_buffer;

extern string map_file_path;

extern double res_mean_last, total_residual;
extern double last_timestamp_lidar , last_timestamp_imu;
extern double gyr_cov, acc_cov , b_gyr_cov , b_acc_cov;
extern double filter_size_corner_min, filter_size_surf_min, filter_size_map_min, fov_deg;
extern double cube_len , HALF_FOV_COS, FOV_DEG, total_distance, lidar_end_time, first_lidar_time;
extern int    effct_feat_num, time_log_counter , scan_count, publish_count;
extern int    iterCount , feats_down_size, NUM_MAX_ITERATIONS, laserCloudValidNum , pcd_save_interval, pcd_index;
extern bool   point_selected_surf[100000];
extern bool   lidar_pushed, flg_first_scan, flg_exit , flg_EKF_inited;
extern bool   scan_pub_en, dense_pub_en, scan_body_pub_en;

extern vector<vector<int>>  pointSearchInd_surf; 
extern vector<BoxPointType> cub_needrm;
extern vector<PointVector>  Nearest_Points; 
extern vector<double>       extrinT;
extern vector<double>       extrinR;
extern deque<double>                     time_buffer;
extern deque<PointCloudXYZIN::Ptr>        lidar_buffer;
extern deque<sensor_msgs::Imu::ConstPtr> imu_buffer;


extern PointCloudXYZIN::Ptr featsFromMap;
extern PointCloudXYZIN::Ptr feats_undistort;
extern PointCloudXYZIN::Ptr feats_down_body;
extern PointCloudXYZIN::Ptr feats_down_world;
extern PointCloudXYZIN::Ptr normvec;
extern PointCloudXYZIN::Ptr laserCloudOri;
extern PointCloudXYZIN::Ptr corr_normvect;
extern PointCloudXYZIN::Ptr _featsArray;

extern pcl::VoxelGrid<PointXYZINormal> downSizeFilterSurf;
extern pcl::VoxelGrid<PointXYZINormal> downSizeFilterMap;

extern KD_TREE<PointXYZINormal> ikdtree;
extern V3F XAxisPoint_body;
extern V3F XAxisPoint_world;
extern V3D euler_cur;
extern V3D position_last;
extern V3D Lidar_T_wrt_IMU;
extern M3D Lidar_R_wrt_IMU;

/*** EKF inputs and output ***/
extern MeasureGroup Measures;
extern esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
extern state_ikfom state_point;
extern vect3 pos_lid;

extern nav_msgs::Path path;
extern nav_msgs::Odometry odomAftMapped;
extern geometry_msgs::Quaternion geoQuat;
extern geometry_msgs::PoseStamped msg_body_pose;

extern shared_ptr<Preprocess> p_pre;
extern shared_ptr<ImuProcess> p_imu;

extern void pointBodyToWorld_ikfom(PointXYZINormal const * const pi, PointXYZINormal * const po, state_ikfom &s);


extern void pointBodyToWorld(PointXYZINormal const * const pi, PointXYZINormal * const po);

template<typename T>
extern void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po);

extern void RGBpointBodyToWorld(PointXYZINormal const * const pi, PointXYZINormal * const po);

extern void RGBpointBodyLidarToIMU(PointXYZINormal const * const pi, PointXYZINormal * const po);
extern void points_cache_collect();

extern BoxPointType LocalMap_Points;
extern bool Localmap_Initialized;
extern void lasermap_fov_segment();

extern void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg, int cameraId, double timestamp);

extern double timediff_lidar_wrt_imu;
extern bool   timediff_set_flg;

extern void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in);

extern double lidar_mean_scantime;
extern int    scan_num;

extern bool sync_packages(MeasureGroup &meas);

extern int process_increments;
extern void map_incremental();

extern PointCloudXYZIN::Ptr pcl_wait_pub;
extern PointCloudXYZIN::Ptr pcl_wait_save;

template<typename T>
extern void set_posestamp(T & out);

extern void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data);

extern bool lidarFinished;
extern std::mutex finishMutex;
extern void LidarInertialProcess();
extern void LidarCameraProcess();
extern int GetLatestLiDARID();

template<typename T>
extern bool EstiPlane(Eigen::Matrix<T, 4, 1> &pca_result, const PointVector &point, const T &threshold);
extern void feature_extraction();
extern void UpdateMap(const TC2LI_SLAM::Frame& frame);
extern void UpdateLidarPose(const TC2LI_SLAM::Frame& lastFrame, const Sophus::SE3f& velocity, const double& timeFromLastFrame);

#endif