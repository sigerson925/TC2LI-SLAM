
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

/**
LiDAROdometry
*/
#ifndef LIDARODOMETRY_H
#define LIDARODOMETRY_H
#include "Viewer.h"
#include "FrameDrawer.h"
#include "Atlas.h"
#include "LocalMapping.h"
#include "Frame.h"
#include "KeyFrame.h"

#include "tools.h"
#include "bavoxel.h"
#include "LidarTypes.h"
#include <mutex>
#include <unordered_set>

#include <time.h>

namespace TC2LI_SLAM
{

class LidarCovisRes
{
private:
    std::mutex mMutexSurfMap;
    unordered_map<VOXEL_LOC, OCTO_TREE_ROOT*> mSurfMap, mCornMap;
    unordered_map<int, unsigned long> mKeyId;

    std::mutex mMutexVoxHess;
    VOX_HESS mVoxHess, mVoxHessCorn;

    BALM2 opt_lsv;
    std::mutex mMutexPose;
    vector<IMUST> mPoseBuf;
    IMUST mPose0;
    vector<pcl::PointCloud<PointXYZINormal>::Ptr> pl_fulls;

    std::mutex mMutexId;

    Sophus::SE3f mTcl;
    Sophus::SE3f mTlc;
    Sophus::SE3f mTbl;
    Sophus::SE3f mTlb;

    int mCurrPosId;
    // int win_size_ = 100;

    bool* mbStopFlag;

    double mwCornerLess;
public:
    int win_size_ = 20;
    LidarCovisRes(const Sophus::SE3f &Tcl, const Sophus::SE3f &Tbl, double cornerless = 0.0, bool* stopFlag = NULL): mCurrPosId(0)
    {
        mTcl = Tcl;
        mTlc = mTcl.inverse();
        mTbl = Tbl;
        mTlb =  mTbl.inverse();

        mbStopFlag = stopFlag;
        mwCornerLess = 0.01;
    }
    ~LidarCovisRes()
    {
        for(auto iter=mSurfMap.begin(); iter!=mSurfMap.end();)
        {
            delete iter->second;
            mSurfMap.erase(iter++);
        }
        mSurfMap.clear();
        for(auto iter=mCornMap.begin(); iter!=mCornMap.end();)
        {
            delete iter->second;
            mCornMap.erase(iter++);
        }
        mCornMap.clear();
    }
    void AddFromKeyFrame(KeyFrame* pKF);
    void UpdateMap();
    void UpdatePose(int i, Eigen::Matrix3d pRcw, Eigen::Vector3d ptcw, Eigen::Matrix3d pRwb, Eigen::Vector3d ptwb);
    void UpdatePose(int i, Eigen::Matrix3d pRcw, Eigen::Vector3d ptcw);
    void BuildVoxHess();
    void ComputeJandH(Eigen::VectorXd &JacT, Eigen::MatrixXd &Hess);
    void ComputeJandHSE3(Eigen::VectorXd &JacT, Eigen::MatrixXd &Hess);
    double ComputeError();

    Eigen::MatrixXd ComputeJfromH();
    Eigen::VectorXd ComputeErrorfromJ();

    template<typename T = double>
    Eigen::Matrix<T,3,3> NormalizeRotation(const Eigen::Matrix<T,3,3> &R) {
    Eigen::JacobiSVD<Eigen::Matrix<T,3,3>> svd(R,Eigen::ComputeFullU | Eigen::ComputeFullV);
    return svd.matrixU() * svd.matrixV().transpose();
    }
    Eigen::Matrix3d ExpSO3(const Eigen::Vector3d &w);

    Eigen::Matrix3d ExpSO3(const double x, const double y, const double z);

    Eigen::Vector3d LogSO3(const Eigen::Matrix3d &R);

    Eigen::Matrix3d InverseRightJacobianSO3(const Eigen::Vector3d &v);

    Eigen::Matrix3d InverseRightJacobianSO3(const double x, const double y, const double z);

    Eigen::Matrix3d RightJacobianSO3(const Eigen::Vector3d &v);

    Eigen::Matrix3d RightJacobianSO3(const double x, const double y, const double z);

    Eigen::Matrix3d Skew(const Eigen::Vector3d &w);
};

}

#endif