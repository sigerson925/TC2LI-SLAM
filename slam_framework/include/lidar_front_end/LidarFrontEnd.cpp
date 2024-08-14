
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
#include "lidar_front_end/LidarFrontEnd.h"

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
/*** Time Log Variables ***/
double kdtree_incremental_time, kdtree_search_time, kdtree_delete_time;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time, solve_time, solve_const_H_time;
int kdtree_size_end, add_point_size, kdtree_delete_counter;
bool time_sync_en = false, extrinsic_est_en = true;
/**************************/

/*** Sync Variables *******/
int sensor = -1;
int mCurrCameraId = -1;
double mCurrTimeStamp = -0.1;
PointCloudXYZIN::Ptr  mCurrFeatPoints(new PointCloudXYZIN());
bool stop_lidar = false;
int lidar_count_id = -1;
bool track_ends = false;
/**************************/

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;
double time_diff_lidar_to_imu = 0.0;

mutex mtx_buffer;
condition_variable sig_buffer;

string map_file_path;

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
bool   point_selected_surf[100000] = {0};
bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;

vector<vector<int>>  pointSearchInd_surf; 
vector<BoxPointType> cub_needrm;
vector<PointVector>  Nearest_Points; 
vector<double>       extrinT(3, 0.0);
vector<double>       extrinR(9, 0.0);
deque<double>                     time_buffer;
deque<int>                     CameraId_buffer;
deque<int>                     LiDARId_buffer;
deque<PointCloudXYZIN::Ptr>        lidar_buffer;
deque<sensor_msgs::Imu::ConstPtr> imu_buffer;


PointCloudXYZIN::Ptr featsFromMap(new PointCloudXYZIN());
PointCloudXYZIN::Ptr feats_undistort(new PointCloudXYZIN());
PointCloudXYZIN::Ptr feats_down_body(new PointCloudXYZIN());
PointCloudXYZIN::Ptr feats_down_world(new PointCloudXYZIN());
PointCloudXYZIN::Ptr normvec(new PointCloudXYZIN(100000, 1));
PointCloudXYZIN::Ptr laserCloudOri(new PointCloudXYZIN(100000, 1));
PointCloudXYZIN::Ptr corr_normvect(new PointCloudXYZIN(100000, 1));
PointCloudXYZIN::Ptr _featsArray;

pcl::VoxelGrid<PointXYZINormal> downSizeFilterSurf;
pcl::VoxelGrid<PointXYZINormal> downSizeFilterMap;

KD_TREE<PointXYZINormal> ikdtree;
V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;

nav_msgs::Path path;
nav_msgs::Odometry odomAftMapped;
geometry_msgs::Quaternion geoQuat;
geometry_msgs::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

void pointBodyToWorld_ikfom(PointXYZINormal const * const pi, PointXYZINormal * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}


void pointBodyToWorld(PointXYZINormal const * const pi, PointXYZINormal * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointXYZINormal const * const pi, PointXYZINormal * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void RGBpointBodyLidarToIMU(PointXYZINormal const * const pi, PointXYZINormal * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history);
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;    
     pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;

    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}
std::mutex syncMutex;
void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg, int cameraId = -1, double timestamp = 0.0) 
{
    mtx_buffer.lock();
    scan_count ++;
    double preprocess_start_time = omp_get_wtime();

    if (timestamp < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }

    if(sensor == STEREO_LIDAR)
    {
        unique_lock<mutex> lock(syncMutex);
        mCurrTimeStamp = timestamp;
    }

    PointCloudXYZIN::Ptr  ptr(new PointCloudXYZIN());
    //* process
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(timestamp);
    CameraId_buffer.push_back(cameraId);
    last_timestamp_lidar = timestamp;
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

int GetCurrCameraId()
{
    unique_lock<mutex> lock(syncMutex);
    return mCurrCameraId;
}

double GetCurrTimeStamp()
{
    unique_lock<mutex> lock(syncMutex);
    return mCurrTimeStamp;
}

pcl::PointCloud<pcl::PointXYZINormal>::Ptr  GetCurrFeatPoints()
{
    unique_lock<mutex> lock(syncMutex);
    return mCurrFeatPoints;
}
void SetSeneorMode(int sensorMode)
{
    sensor = sensorMode;
}

double timediff_lidar_wrt_imu = 0.0;
bool   timediff_set_flg = false;

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) 
{
    publish_count ++;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    msg->header.stamp = ros::Time().fromSec(msg_in->header.stamp.toSec() - time_diff_lidar_to_imu);
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp = \
        ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
    }

    double timestamp = msg->header.stamp.toSec();

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        ROS_WARN("imu loop back, clear buffer");
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double lidar_mean_scantime = 0.0;
int    scan_num = 0;
bool sync_packages(MeasureGroup &meas)
{
    mtx_buffer.lock();
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        mtx_buffer.unlock();
        return false;
    }
    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        {
            while(!lidar_buffer.empty())
            {
                meas.lidar = lidar_buffer.front();
                lidar_buffer.pop_front();
                meas.cameraId  =  CameraId_buffer.front();
                CameraId_buffer.pop_front();
                meas.lidar_beg_time = time_buffer.front();
                time_buffer.pop_front();
            }
            if(mCurrTimeStamp > meas.lidar_beg_time)    
            {
                
            }
        }
        if (meas.lidar->points.size() <= 1) // time too little
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            ROS_WARN("Too few input point cloud!\n");
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {
            scan_num ++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }
        meas.lidar_end_time = lidar_end_time;

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        if(lidar_pushed)
        mtx_buffer.unlock();
        return false;
    }
    /*** push imu data, and pop from imu buffer ***/
    double imu_time = imu_buffer.front()->header.stamp.toSec();
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = imu_buffer.front()->header.stamp.toSec();
        if(imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_pushed = false;
    mtx_buffer.unlock();
    return true;
}

int process_increments = 0;
void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointXYZINormal downsample_result, mid_point;
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false); 
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();

    kdtree_incremental_time = omp_get_wtime() - st_time;
}

PointCloudXYZIN::Ptr pcl_wait_pub(new PointCloudXYZIN(500000, 1));
PointCloudXYZIN::Ptr pcl_wait_save(new PointCloudXYZIN());

template<typename T>
void set_posestamp(T & out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;
    
}

template<typename T>
bool esti_plane(Matrix<T, 4, 1> &pca_result, const PointVector &point, const T &threshold)
{
    Matrix<T, NUM_MATCH_POINTS, 3> A;
    Matrix<T, NUM_MATCH_POINTS, 1> b;
    A.setZero();
    b.setOnes();
    b *= -1.0f;
    for (int j = 0; j < NUM_MATCH_POINTS; j++)
    {
        A(j,0) = point[j].x;
        A(j,1) = point[j].y;
        A(j,2) = point[j].z;
    }
    Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);
    T n = normvec.norm();
    pca_result(0) = normvec(0) / n;
    pca_result(1) = normvec(1) / n;
    pca_result(2) = normvec(2) / n;
    pca_result(3) = 1.0 / n;

    for (int j = 0; j < NUM_MATCH_POINTS; j++)
    {
        if (fabs(pca_result(0) * point[j].x + pca_result(1) * point[j].y + pca_result(2) * point[j].z + pca_result(3)) > threshold)
        {
            return false;
        }
    }
    return true;
}


void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    double search_start = omp_get_wtime();
    double search_time;
    laserCloudOri->clear(); 
    corr_normvect->clear(); 
    total_residual = 0.0; 

    /** closest surface search and residual computation **/
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointXYZINormal &point_body  = feats_down_body->points[i];
        PointXYZINormal &point_world = feats_down_world->points[i];

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];
        search_start = omp_get_wtime();
        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }
        search_time += omp_get_wtime() - search_start;


        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        point_selected_surf[i] = false;
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }
    
    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num ++;
        }
    }

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        ROS_WARN("No Effective Points! \n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num;

    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointXYZINormal &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointXYZINormal &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
}


int GetLatestLiDARID()
{
    // lidar_buffer.back();
    return CameraId_buffer.empty()? -1 : CameraId_buffer.front();
}


double camera_timestamp;
bool lidarFinished = false;
std::mutex finishMutex;
void LidarInertialProcess()
{
    std::cout << "LiDAR - Inertial Thread" << std::endl;
    ros::NodeHandle nh_lio;

    nh_lio.param<int>("max_iteration", NUM_MAX_ITERATIONS, 4);
    nh_lio.param<string>("map_file_path", map_file_path, "");
    nh_lio.param<bool>("common/time_sync_en", time_sync_en, false);
    nh_lio.param<double>("common/time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
    nh_lio.param<double>("filter_size_corner", filter_size_corner_min, 0.5);
    nh_lio.param<double>("filter_size_surf", filter_size_surf_min, 0.5);
    nh_lio.param<double>("filter_size_map", filter_size_map_min, 0.5);
    nh_lio.param<double>("cube_side_length", cube_len, 200);
    nh_lio.param<float>("mapping/det_range", DET_RANGE, 300.f);
    nh_lio.param<double>("mapping/fov_degree", fov_deg, 180);
    nh_lio.param<double>("mapping/gyr_cov", gyr_cov, 0.1);
    nh_lio.param<double>("mapping/acc_cov", acc_cov, 0.1);
    nh_lio.param<double>("mapping/b_gyr_cov", b_gyr_cov, 0.0001);
    nh_lio.param<double>("mapping/b_acc_cov", b_acc_cov, 0.0001);
    nh_lio.param<double>("preprocess/blind", p_pre->blind, 0.01);
    nh_lio.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);
    nh_lio.param<int>("preprocess/timestamp_unit", p_pre->time_unit, US);
    nh_lio.param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 10);
    nh_lio.param<int>("point_filter_num", p_pre->point_filter_num, 2);
    nh_lio.param<bool>("feature_extract_enable", p_pre->feature_enabled, false);
    nh_lio.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, true);
    nh_lio.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
    nh_lio.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());

    path.header.stamp    = ros::Time::now();
    path.header.frame_id ="base_link";

    /*** variables definition ***/
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;
    
    FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

    _featsArray.reset(new PointCloudXYZIN());

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));

    cout << "filter_size_surf_min " << filter_size_surf_min<<endl;
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));

    Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    double epsi[23] = {0.001};
    fill(epsi, epsi+23, 0.001);
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);
    while (!stop_lidar)
    {  
        if (flg_exit) break;
        
        if(sync_packages(Measures)) 
        {
            //* LIDAR Front End Finish Flag
            lidarFinished = false;
            TicToc wholeProcess(1);
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                lidarFinished = true;
                continue;
            }

            p_imu->Process(Measures, kf, feats_undistort);

            
            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                lidarFinished = true;
                continue;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;
            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();

            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            feats_down_size = feats_down_body->points.size();

            /*** initialize the map kdtree ***/
            if(ikdtree.Root_Node == nullptr)
            {
                if(feats_down_size > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for(int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    ikdtree.Build(feats_down_world->points);
                }
                lidarFinished = true;
                continue;
            }
            int featsFromMapNum = ikdtree.validnum();
            ikdtree.size();

            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5)
            {
                lidarFinished = true;
                continue;
            }
            
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);
            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);

            /*** iterated state estimation ***/
            double solve_H_time = 0;
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
            state_point = kf.get_x();
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            geoQuat.x = state_point.rot.coeffs()[0];
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];


            /*** add the feature points to map kdtree ***/
            map_incremental();

            {
                //* sync id and planar feature points
                unique_lock<mutex> lock(syncMutex);
                mCurrCameraId = Measures.cameraId;
                mCurrTimeStamp = Measures.lidar_beg_time;
                mCurrFeatPoints->clear();
                mCurrFeatPoints->reserve(effect_feat_num);
                for(auto it = laserCloudOri->points.begin(); it != laserCloudOri->points.begin() + effct_feat_num; ++it)
                {
                    PointXYZINormal p1 = *it;
                    mCurrFeatPoints->push_back(p1);
                }
            }
            lidarFinished = true;
        }
        usleep(3000);
    }
    return;
}

std::mutex updateMapMutex;
std::mutex updatePosMutex;

bool poseReady = false;
void UpdateLidarPose(const TC2LI_SLAM::Frame& lastFrame, const Sophus::SE3f& velocity, const double& timeFromLastFrame)
{
    unique_lock<mutex> lock(finishMutex);
    Sophus::SE3f  Twc = lastFrame.GetPose().inverse() * Sophus::SE3f::exp(timeFromLastFrame * velocity.inverse().log());
    Eigen::Matrix4f Mwc = (Twc).matrix();
    Eigen::Matrix4f Mwl = Mwc * TC2LI_SLAM::LIDAR::LidarFrontEndTools::mLidarParam->mTcl.matrix();
    Eigen::Matrix3f Rw2_w1;
    Rw2_w1 << 0,0,1,
            -1,0,0,
            0,-1,0;
    state_point.rot = (Rw2_w1*Mwl.block<3,3>(0,0)).cast<double>();
    state_point.pos = (Rw2_w1*Mwl.block<3,1>(0,3)).cast<double>();
    pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
    poseReady = true;
}
void LidarCameraProcess()
{
    ros::NodeHandle nh_lio;
    nh_lio.param<int>("max_iteration", NUM_MAX_ITERATIONS, 4);
    nh_lio.param<string>("map_file_path", map_file_path, "");
    nh_lio.param<bool>("common/time_sync_en", time_sync_en, false);
    nh_lio.param<double>("common/time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
    nh_lio.param<double>("filter_size_corner", filter_size_corner_min, 0.5);
    nh_lio.param<double>("filter_size_surf", filter_size_surf_min, 0.5);
    nh_lio.param<double>("filter_size_map", filter_size_map_min, 0.5);
    nh_lio.param<double>("cube_side_length", cube_len, 200);
    nh_lio.param<float>("mapping/det_range", DET_RANGE, 300.f);
    nh_lio.param<double>("mapping/fov_degree", fov_deg, 180);
    nh_lio.param<double>("mapping/gyr_cov", gyr_cov, 0.1);
    nh_lio.param<double>("mapping/acc_cov", acc_cov, 0.1);
    nh_lio.param<double>("mapping/b_gyr_cov", b_gyr_cov, 0.0001);
    nh_lio.param<double>("mapping/b_acc_cov", b_acc_cov, 0.0001);
    nh_lio.param<double>("preprocess/blind", p_pre->blind, 0.01);
    nh_lio.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);
    nh_lio.param<int>("preprocess/timestamp_unit", p_pre->time_unit, US);
    nh_lio.param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 10);
    nh_lio.param<int>("point_filter_num", p_pre->point_filter_num, 2);
    nh_lio.param<bool>("feature_extract_enable", p_pre->feature_enabled, false);
    nh_lio.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, true);
    nh_lio.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
    nh_lio.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());

    path.header.stamp    = ros::Time::now();
    path.header.frame_id ="base_link";

    /*** variables definition ***/
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;
    
    FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);

    _featsArray.reset(new PointCloudXYZIN());

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));

    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));

    Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    double epsi[23] = {0.001};
    fill(epsi, epsi+23, 0.001);
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

//------------------------------------------------------------------------------------------------------
    while (!stop_lidar)
    {

        if(!lidar_buffer.empty() && poseReady) 
        {
            poseReady = false;
            {
            unique_lock<mutex> lock(finishMutex);
            //* LIDAR Front End Finish Flag
            // lidarFinished = false;
            if (flg_first_scan)
            {
                unique_lock<mutex> lock(syncMutex);
                first_lidar_time = time_buffer.front();
                Measures.cameraId  =  CameraId_buffer.front();
                time_buffer.pop_front();
                lidar_buffer.pop_front();
                CameraId_buffer.pop_front();
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                lidarFinished = true;
                continue;
            }
            //* process sync data
            while(!lidar_buffer.empty())
            {
                //* buffer lock
                mtx_buffer.lock();
                feats_undistort->clear();
                feats_undistort = lidar_buffer.front();
                lidar_buffer.pop_front();
                Measures.cameraId  =  CameraId_buffer.front();
                CameraId_buffer.pop_front();
                Measures.lidar_beg_time = time_buffer.front();
                time_buffer.pop_front();
                mtx_buffer.unlock();
            }

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                ROS_WARN("No point, skip this scan!\n");
                lidarFinished = true;
                continue;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;

            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();

            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            feats_down_size = feats_down_body->points.size();

            //* init ikd-tree
            if(ikdtree.Root_Node == nullptr)
            {
                if(feats_down_size > 5)
                {
                    ikdtree.set_downsample_param(filter_size_map_min);
                    feats_down_world->resize(feats_down_size);
                    for(int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    ikdtree.Build(feats_down_world->points);
                }
                continue;
            }

            if (feats_down_size < 5)
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }
            feats_down_world->resize(feats_down_size);
            Nearest_Points.resize(feats_down_size); 

            //* feature extraction
            feature_extraction();
            {
                //* sync id and planar feature points
                unique_lock<mutex> lock(syncMutex);
                mCurrCameraId = Measures.cameraId;
                mCurrTimeStamp = Measures.lidar_beg_time;
                mCurrFeatPoints->clear();
                mCurrFeatPoints->reserve(effect_feat_num);
                for(auto it = laserCloudOri->points.begin(); it != laserCloudOri->points.begin() + effct_feat_num; ++it)
                {
                    PointXYZINormal p1 = *it;
                    mCurrFeatPoints->push_back(p1);
                }
            }
            }
            lidarFinished = true;
        }
        usleep(3000);
    }

}

template<typename T>
bool EstiPlane(Eigen::Matrix<T, 4, 1> &pca_result, const PointVector &point, const T &threshold)
{
    Eigen::Matrix<T, NUM_MATCH_POINTS, 3> A;
    Eigen::Matrix<T, NUM_MATCH_POINTS, 1> b;
    A.setZero();
    b.setOnes();
    b *= -1.0f;

    for (int j = 0; j < NUM_MATCH_POINTS; j++)
    {
        A(j,0) = point[j].x;
        A(j,1) = point[j].y;
        A(j,2) = point[j].z;
    }

    //* plane normal vector
    Eigen::Matrix<T, 3, 1> normvec = A.colPivHouseholderQr().solve(b);

    T n = normvec.norm();
    pca_result(0) = normvec(0) / n;
    pca_result(1) = normvec(1) / n;
    pca_result(2) = normvec(2) / n;
    pca_result(3) = 1.0 / n;

    for (int j = 0; j < NUM_MATCH_POINTS; j++)
    {
        if (fabs(pca_result(0) * point[j].x + pca_result(1) * point[j].y + pca_result(2) * point[j].z + pca_result(3)) > threshold)
        {
            return false;
        }
    }
    return true;
}

void feature_extraction()
{
    
    normvec->resize(feats_down_size);
    laserCloudOri->clear(); 
    corr_normvect->clear(); 
    double total_residual = 0.0; 
    effct_feat_num = 0;

    /** closest surface search and residual computation **/
    // #ifdef MP_EN
    //     omp_set_num_threads(MP_PROC_NUM);
    //     #pragma omp parallel for
    // #endif
    double search_time = 0.0;int count = 0;
    for (int i = 0; i < feats_down_size; i++)
    {
        pcl::PointXYZINormal &point_body  = feats_down_body->points[i]; 
        pcl::PointXYZINormal &point_world = feats_down_world->points[i]; 

        /* transform to world frame */
        Eigen::Vector3d p_body(point_body.x, point_body.y, point_body.z);
        pointBodyToWorld(&point_body, &point_world);

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

        auto &points_near = Nearest_Points[i];

        /** Find the closest surfaces in the map **/
        TicToc searchTime;
        ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
        search_time += searchTime.toc("11");

        point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        
        if (!point_selected_surf[i]) continue;

        Eigen::Matrix<float, 4, 1> pabcd;
        point_selected_surf[i] = false;
        
        if (EstiPlane(pabcd, points_near, 0.1f))
        {
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());
            
            if (s > 0.9)
            {
                ++count;
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num ++;
        }
    }

    if (effct_feat_num < 1)
    {
        ROS_WARN("No Effective Points! \n");
        return;
    }
}

void UpdateMap(const TC2LI_SLAM::Frame& frame)
{
    if(Nearest_Points.size() <= 0) return;
    map_incremental();
}
