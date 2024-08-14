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


#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "System.h"
#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "Frame.h"
#include "LidarTypes.h"
#include <math.h>

#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/sparse_block_matrix.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_gauss_newton.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"

namespace TC2LI_SLAM
{

class Optimizer
{
    static Sophus::SE3<float> mTcl;
    static Sophus::SE3<float> mTbl;
    static LIDAR::LidarParam* mLidarParam;
public:

    void static BundleAdjustment(const std::vector<KeyFrame*> &vpKF, const std::vector<MapPoint*> &vpMP,
                                 int nIterations = 5, bool *pbStopFlag=NULL, const unsigned long nLoopKF=0,
                                 const bool bRobust = true);

    void static FullInertialBA(Map *pMap, int its, const bool bFixLocal=false, const unsigned long nLoopKF=0, bool *pbStopFlag=NULL, bool bInit=false, float priorG = 1e2, float priorA=1e6, Eigen::VectorXd *vSingVal = NULL, bool *bHess=NULL);

    void static LocalBundleAdjustment(KeyFrame* pKF, bool *pbStopFlag, Map *pMap, int& num_fixedKF, int& num_OptKF, int& num_MPs, int& num_edges);

    int static PoseOptimization(Frame* pFrame);
    int static PoseInertialOptimizationLastKeyFrame(Frame* pFrame, bool bRecInit = false);
    int static PoseInertialOptimizationLastFrame(Frame *pFrame, bool bRecInit = false);


    // For inertial systems

    void static LocalInertialBA(KeyFrame* pKF, bool *pbStopFlag, Map *pMap, int& num_fixedKF, int& num_OptKF, int& num_MPs, int& num_edges, bool bLarge = false, bool bRecInit = false);

    // void static LocalLVBundleAdjustment(KeyFrame* pMainKF,vector<KeyFrame*> vpAdjustKF, vector<KeyFrame*> vpFixedKF, bool *pbStopFlag);
    // Marginalize block element (start:end,start:end). Perform Schur complement.
    // Marginalized elements are filled with zeros.
    static Eigen::MatrixXd Marginalize(const Eigen::MatrixXd &H, const int &start, const int &end);

    // Inertial pose-graph
    void static InertialOptimization(Map *pMap, Eigen::Matrix3d &Rwg, double &scale, Eigen::Vector3d &bg, Eigen::Vector3d &ba, bool bMono, Eigen::MatrixXd  &covInertial, bool bFixedVel=false, bool bGauss=false, float priorG = 1e2, float priorA = 1e6);
    void static InertialOptimization(Map *pMap, Eigen::Matrix3d &Rwg, double &scale);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
};

} //namespace TC2LI_SLAM

#endif // OPTIMIZER_H
