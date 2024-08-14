
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
#include "LidarTypes.h"
#include "Converter.h"

#include "GeometricTools.h"

#include<iostream>

namespace TC2LI_SLAM
{

namespace LIDAR
{
LidarParam* LidarFrontEndTools::mLidarParam = new LIDAR::LidarParam();

pcl::PointCloud<PointXYZINormal>::Ptr LidarFrontEndTools::transformPointCloud(pcl::PointCloud<PointXYZINormal>::Ptr cloudIn, const Sophus::SE3f &transformIn)
{
    pcl::PointCloud<PointXYZINormal>::Ptr cloudOut(new pcl::PointCloud<PointXYZINormal>());

    PointXYZINormal *pointFrom;
    Eigen::Vector3f pointPos;
    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);
    
    
    Eigen::Matrix3f Rwl = transformIn.rotationMatrix();
    Eigen::Vector3f twl = transformIn.translation();
    for (int i = 0; i < cloudSize; ++i)
    {
      pointFrom = &cloudIn->points[i];
      pointPos = Eigen::Vector3f(pointFrom->x,pointFrom->y,pointFrom->z);
      pointPos = Rwl*pointPos+twl;
      cloudOut->points[i].x = pointPos(0);
      cloudOut->points[i].y = pointPos(1);
      cloudOut->points[i].z = pointPos(2);
      cloudOut->points[i].intensity = pointFrom->intensity;
    }
    return cloudOut;
}

LidarParam::LidarParam(const LidarParam &lidarParam)
{
    mTcl = lidarParam.mTcl;
    mTlc = lidarParam.mTlc;
    mTbl = lidarParam.mTbl;
    mTlb = lidarParam.mTlb;
    mTbc = lidarParam.mTbc;
    mbIsSet = lidarParam.mbIsSet;
    mWeightLocalBA = lidarParam.mWeightLocalBA;
}

void LidarParam::Set(const Sophus::SE3<float> &sophTcl,const Sophus::SE3<float> &sophTbc,const float &weightLocalBA)
{
    mbIsSet = true;
    mTbc = sophTbc;
    mTcl = sophTcl;
    mTlc = mTcl.inverse();
    mTbl = sophTbc*sophTcl;
    mTlb = mTbl.inverse();
    mWeightLocalBA = weightLocalBA;
}

} // namespace LIDAR
} // namespace TC2LI_SLAM