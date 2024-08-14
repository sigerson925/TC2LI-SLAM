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


#ifndef LIDARTYPES_H
#define LIDARTYPES_H
#include "include/IKFoM_toolkit/mtk/build_manifold.hpp"
#include "include/use-ikfom.hpp"
#include "lidar_front_end/common_lib.h"

#define NUM_MATCH_POINTS    (5)

#include <vector>
#include <utility>
#include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <sophus/se3.hpp>
#include <mutex>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl_conversions/pcl_conversions.h>

#include "tictoc.h"
#include "SerializationUtils.h"
#include <boost/serialization/serialization.hpp>
#include <boost/serialization/vector.hpp>


#include "include/ikd-Tree/ikd_Tree.h"


//* -----------------------------------------
#include "Frame.h"

namespace TC2LI_SLAM {

    class Frame;
    namespace LIDAR {
//********************************************

        class LidarParam {
            friend class boost::serialization::access;

            template<class Archive>
            void serialize(Archive &ar, const unsigned int version) {
                serializeSophusSE3(ar, mTcl, version);
                serializeSophusSE3(ar, mTlc, version);
                serializeSophusSE3(ar, mTbl, version);
                serializeSophusSE3(ar, mTlb, version);
                serializeSophusSE3(ar, mTlb, mTbc);
                ar & mbIsSet;
                ar & mWeightLocalBA;
            }

        public:

            LidarParam(const Sophus::SE3<float> &Tcl, const Sophus::SE3<float> &Tbc, const float &weightLocalBA) {
                Set(Tcl, Tbc, weightLocalBA);
            }

            LidarParam(const LidarParam &lidarParam);

            LidarParam() { mbIsSet = false; }

            void Set(const Sophus::SE3<float> &sophTcl, const Sophus::SE3<float> &sophTbc, const float &weightLocalBA);

        public:
            // Sophus/Eigen implementation
            Sophus::SE3<float> mTcl;
            Sophus::SE3<float> mTlc;
            Sophus::SE3<float> mTbl;
            Sophus::SE3<float> mTlb;
            Sophus::SE3<float> mTbc;

            double mWeightLocalBA;
            bool mbIsSet;
        };

        class LidarProperties {
        private:
            PointCloudXYZIN::Ptr mCornerPcl;
            PointCloudXYZIN::Ptr mSurfacePcl;
            double mLidarTimeStamp;
        public:
            PointCloudXYZIN::Ptr mFullPcl;

            LidarProperties() {
                mFullPcl.reset(new PointCloudXYZIN());
                mCornerPcl.reset(new PointCloudXYZIN());
                mSurfacePcl.reset(new PointCloudXYZIN());
            }

            ~LidarProperties() {
                mFullPcl->clear();
                mCornerPcl->clear();
                mSurfacePcl->clear();
            }

            LidarProperties(double time, const PointCloudXYZIN::Ptr &LidarCloudIn) :
                    mLidarTimeStamp(time), mFullPcl(LidarCloudIn) {
                //、
                mCornerPcl.reset(new PointCloudXYZIN());
                mSurfacePcl.reset(new PointCloudXYZIN());
            }

            LidarProperties(const LidarProperties &lidarProps) {
                mFullPcl.reset(new PointCloudXYZIN(*lidarProps.mFullPcl));
                mCornerPcl.reset(new PointCloudXYZIN(*lidarProps.mCornerPcl));
                mSurfacePcl.reset(new PointCloudXYZIN(*lidarProps.mSurfacePcl));
                mLidarTimeStamp = lidarProps.mLidarTimeStamp;
            }

            PointCloudXYZIN::Ptr GetFullPcl() {
                return mFullPcl;
            }

            PointCloudXYZIN::Ptr GetCornerPcl() { return mCornerPcl; }

            PointCloudXYZIN::Ptr GetSurfacePcl() { return mSurfacePcl; }


            void SetPcl(PointCloudXYZIN::Ptr &surfCloudPtr) {
                mSurfacePcl.reset(new PointCloudXYZIN(*surfCloudPtr));
            }

            double GetCloudTime() { return mLidarTimeStamp; }

            void SetCloudTime(const double &time) { mLidarTimeStamp = time; }
        };

        class LidarFrontEndTools {
        public:
            static LidarParam *mLidarParam;

            void static SetLidarParam(LidarParam *pLidarParam) {
                mLidarParam = new LidarParam(*pLidarParam);
            }

            static PointCloudXYZIN::Ptr
            transformPointCloud(PointCloudXYZIN::Ptr cloudIn, const Sophus::SE3f &transformIn);
        };

    }

} //namespace TC2LI_SLAM

#endif // LIDARTYPES_H
