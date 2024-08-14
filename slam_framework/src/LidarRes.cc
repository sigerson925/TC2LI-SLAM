
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
#include "LidarRes.h"

namespace TC2LI_SLAM
{
void LidarCovisRes::AddFromKeyFrame(KeyFrame* pKF)
{
    static bool bWinIsFull = false;
    if(pKF->mLidarProps->GetSurfacePcl()->size() <= 0)
    {
        return;
    }
    
    IMUST curr;

    // transforms
    Sophus::SE3f Twl = pKF->GetPoseInverse()*mTcl;
    Eigen::Matrix3f Rwl = Twl.rotationMatrix();
    Eigen::Vector3f twl = Twl.translation();
    
    curr.R = Rwl.cast<double>(); 
    curr.p = twl.cast<double>(); 
    curr.t = pKF->mTimeStamp;

    // update mPose0
    if(mPoseBuf.size() <= 0) mPose0 = curr;
    curr.p = mPose0.R.transpose() * (curr.p - mPose0.p);
    curr.R = mPose0.R.transpose() * curr.R;
    mPoseBuf.push_back(curr);

    pcl::PointCloud<PointXYZINormal>::Ptr pl_ptr(new pcl::PointCloud<PointXYZINormal>(*pKF->mLidarProps->GetSurfacePcl()));
    // cut_voxel
    cut_voxel(mSurfMap, *pl_ptr ,curr, mCurrPosId, win_size_);

    mCurrPosId++;
}
void LidarCovisRes::BuildVoxHess()
{
    int win_count = mCurrPosId;
    for(auto iter=mSurfMap.begin(); iter!=mSurfMap.end(); iter++)
    {
        iter->second->recut(win_count, win_size_);
        iter->second->tras_opt(mVoxHess, win_count, win_size_);
    }

    clock();
    for(auto iter=mCornMap.begin(); iter!=mCornMap.end(); iter++)
    {
        iter->second->recut(win_count, win_size_);
        iter->second->tras_opt(mVoxHessCorn, win_count, win_size_);
    }
    clock();
}
void LidarCovisRes::UpdateMap()
{
    for(int i=0; i<win_size_; i++)
    {
        cut_voxel(mSurfMap,*pl_fulls[i], mPoseBuf[i], i, win_size_);
    }
    BuildVoxHess();
}

void LidarCovisRes::ComputeJandH(Eigen::VectorXd &JacT, Eigen::MatrixXd &Hess)
{

    Hess.resize(6*win_size_, 6*win_size_);
    Hess.setZero();
    JacT.resize(6*win_size_);
    JacT.setZero();
    double residual1 = opt_lsv.divide_thread(mPoseBuf, mVoxHess, mPoseBuf, Hess, JacT, win_size_);
    Eigen::Matrix3d Rlb = mTlb.rotationMatrix().cast<double>();
    Eigen::Vector3d tlb = mTlb.translation().cast<double>();
    Eigen::Vector3d tbl = mTbl.translation().cast<double>();

    for(int i=0;i<win_size_;++i)
    {
        Eigen::Matrix3d Rwbi = mPoseBuf[i].R * Rlb;
        Eigen::Vector3d JacwT = JacT.block<3, 1>(6*i, 0); // 对旋转量的雅可比
        Eigen::Vector3d JactT = JacT.block<3, 1>(6*i + 3, 0); // 对平移量的雅可比 
        Eigen::Vector3d rwl = Sophus::SO3f(mPoseBuf[i].R.cast<float>()).log().cast<double>();
        Eigen::Matrix3d inverseJr_Rlb_T = (InverseRightJacobianSO3(rwl)* Rlb).transpose() ;
        Eigen::Matrix3d Rwb_tbl_skew_T = (Rwbi * Sophus::SO3d::hat(tbl)).transpose();

        JacT.block<3, 1>(6*i, 0) = inverseJr_Rlb_T * JacwT - Rwb_tbl_skew_T * JactT;
        JacT.block<3, 1>(6*i + 3, 0) = Rwbi.transpose()*JactT;

        Eigen::Matrix<double,6,6> Di_T; Di_T.setZero();
        Di_T.block<3,3>(0,0) += inverseJr_Rlb_T;
        Di_T.block<3,3>(0,3) -= Rwb_tbl_skew_T;
        Di_T.block<3,3>(3,3) += Rwbi.transpose();
        Eigen::Matrix<double,6,6> Di; Di.setZero();
        Di+=Di_T.transpose();
        for(int j = 0; j < win_size_; ++j)
        {
            Eigen::Matrix<double,6,6> Hij = Hess.block<6,6>(6*i, 6*j);
            Hess.block<6,6>(6*i, 6*j) = Di_T * Hij;

            Eigen::Matrix<double,6,6> Hji = Hess.block<6,6>(6*j, 6*i);
            Hess.block<6,6>(6*j, 6*i) = Hji * Di;
        }
    }
}

void LidarCovisRes::ComputeJandHSE3(Eigen::VectorXd &JacT, Eigen::MatrixXd &Hess)
{
    Hess.resize(6*win_size_, 6*win_size_);
    Hess.setZero();
    JacT.resize(6*win_size_);
    JacT.setZero();
    double residual1 = opt_lsv.divide_thread(mPoseBuf, mVoxHess, mPoseBuf, Hess, JacT, win_size_);

    Eigen::Matrix3d Rlc = mTlc.rotationMatrix().cast<double>();
    Eigen::Vector3d tlc = mTlc.translation().cast<double>();
    Eigen::Matrix3d Rcl = mTcl.rotationMatrix().cast<double>();
    Eigen::Vector3d tcl = mTcl.translation().cast<double>();;
    for(int i=0;i<win_size_;++i)
    {
        Eigen::Matrix3d Rwci = mPoseBuf[i].R * Rlc;
        Eigen::Matrix3d Rcwi = Rwci.transpose();
        Eigen::Vector3d twci = mPoseBuf[i].R * tlc + mPoseBuf[i].p;
        Eigen::Vector3d tcwi = - Rwci.transpose() * twci;
        Eigen::Vector3d JacwT = JacT.block<3, 1>(6*i, 0); // 对旋转量的雅可比
        Eigen::Vector3d JactT = JacT.block<3, 1>(6*i + 3, 0); // 对平移量的雅可比 
        Eigen::Vector3d rwl = Sophus::SO3f(mPoseBuf[i].R.cast<float>()).log().cast<double>();
        Eigen::Matrix3d inverseJr_Rlc_T = (InverseRightJacobianSO3(rwl)* Rlc).transpose() ;
        Eigen::Matrix3d Rwc_tcl_tcw_T = (Rwci * Sophus::SO3d::hat(tcl - tcwi)).transpose();
        Eigen::Vector3d JacwT_ =  - inverseJr_Rlc_T * JacwT + Rwc_tcl_tcw_T * JactT;
        Eigen::Vector3d JactT_ = - Rcwi *JactT;

        JacT.block<3, 1>(6*i, 0) = Rcwi * JacwT_ - Sophus::SO3d::hat(tcwi).transpose() * JactT_;
        JacT.block<3, 1>(6*i + 3, 0) = JactT_;

        Eigen::Matrix<double,6,6> Di_T; Di_T.setZero();
        Di_T.block<3,3>(0,0) -= Rcwi * inverseJr_Rlc_T;
        Di_T.block<3,3>(0,3) += Rcwi * Rwc_tcl_tcw_T;
        Di_T.block<3,3>(0,3) += Sophus::SO3d::hat(tcwi).transpose() * Rcwi;
        Di_T.block<3,3>(3,3) -= Rcwi;
        Eigen::Matrix<double,6,6> Di; Di.setZero();
        Di+=Di_T.transpose();
        for(int j = 0; j < win_size_; ++j)
        {
            Eigen::Matrix<double,6,6> Hij = Hess.block<6,6>(6*i, 6*j);
            Hess.block<6,6>(6*i, 6*j) = Di_T * Hij;

            Eigen::Matrix<double,6,6> Hji = Hess.block<6,6>(6*j, 6*i);
            Hess.block<6,6>(6*j, 6*i) = Hji * Di;
        }
    }
}


double LidarCovisRes::ComputeError()
{
    return opt_lsv.only_residual(mPoseBuf, mVoxHess, mPoseBuf, win_size_);
}

Eigen::VectorXd LidarCovisRes::ComputeErrorfromJ()
{
    Eigen::VectorXd JacT(6*win_size_);
    double residual1 = opt_lsv.divide_threadJ(mPoseBuf, mVoxHess, JacT, win_size_);
    return JacT;
}

Eigen::MatrixXd LidarCovisRes::ComputeJfromH()
{
    // win_size = win_size_;
    Eigen::MatrixXd D(6*win_size_, 6*win_size_), Hess(6*win_size_, 6*win_size_);
    Eigen::VectorXd JacT(6*win_size_);
    double residual1 = opt_lsv.divide_thread(mPoseBuf, mVoxHess, mPoseBuf, Hess, JacT, win_size_);
    return Hess;
}

void LidarCovisRes::UpdatePose(int i, Eigen::Matrix3d pRcw, Eigen::Vector3d ptcw, Eigen::Matrix3d pRwb, Eigen::Vector3d ptwb)
{
    if(i>=win_size_)
    {
        return;
    }

    Sophus::SE3f Tcw(pRcw.cast<float>(),ptcw.cast<float>());
    Sophus::SE3f Twl = Tcw.inverse()*mTcl;

    Eigen::Matrix3f Rwl = Twl.rotationMatrix();
    Eigen::Vector3f twl = Twl.translation();
    mPoseBuf[i].R = Rwl.cast<double>();
    mPoseBuf[i].p = twl.cast<double>();

    if(i == 0) mPose0 = mPoseBuf[i];

    mPoseBuf[i].p = mPose0.R.transpose() * (mPoseBuf[i].p - mPose0.p);
    mPoseBuf[i].R = mPose0.R.transpose() * mPoseBuf[i].R;

}

void LidarCovisRes::UpdatePose(int i, Eigen::Matrix3d pRcw, Eigen::Vector3d ptcw)
{
    if(i>=win_size_)
    {
        return;
    }

    Sophus::SE3f Tcw(pRcw.cast<float>(),ptcw.cast<float>());
    Sophus::SE3f Twl = Tcw.inverse()*mTcl;

    Eigen::Matrix3f Rwl = Twl.rotationMatrix();
    Eigen::Vector3f twl = Twl.translation();
    mPoseBuf[i].R = Rwl.cast<double>();
    mPoseBuf[i].p = twl.cast<double>();    
}

Eigen::Matrix3d LidarCovisRes::ExpSO3(const Eigen::Vector3d &w)
{
    return ExpSO3(w[0],w[1],w[2]);
}

Eigen::Matrix3d LidarCovisRes::ExpSO3(const double x, const double y, const double z)
{
    const double d2 = x*x+y*y+z*z;
    const double d = sqrt(d2);
    Eigen::Matrix3d W;
    W << 0.0, -z, y,z, 0.0, -x,-y,  x, 0.0;
    if(d<1e-5)
    {
        Eigen::Matrix3d res = Eigen::Matrix3d::Identity() + W +0.5*W*W;
        return NormalizeRotation(res);
    }
    else
    {
        Eigen::Matrix3d res =Eigen::Matrix3d::Identity() + W*sin(d)/d + W*W*(1.0-cos(d))/d2;
        return NormalizeRotation(res);
    }
}

Eigen::Vector3d LidarCovisRes::LogSO3(const Eigen::Matrix3d &R)
{
    const double tr = R(0,0)+R(1,1)+R(2,2);
    Eigen::Vector3d w;
    w << (R(2,1)-R(1,2))/2, (R(0,2)-R(2,0))/2, (R(1,0)-R(0,1))/2;
    const double costheta = (tr-1.0)*0.5f;
    if(costheta>1 || costheta<-1)
        return w;
    const double theta = acos(costheta);
    const double s = sin(theta);
    if(fabs(s)<1e-5)
        return w;
    else
        return theta*w/s;
}

Eigen::Matrix3d LidarCovisRes::InverseRightJacobianSO3(const Eigen::Vector3d &v)
{
    return InverseRightJacobianSO3(v[0],v[1],v[2]);
}

Eigen::Matrix3d LidarCovisRes::InverseRightJacobianSO3(const double x, const double y, const double z)
{
    const double d2 = x*x+y*y+z*z;
    const double d = sqrt(d2);

    Eigen::Matrix3d W;
    W << 0.0, -z, y,z, 0.0, -x,-y,  x, 0.0;
    if(d<1e-5)
        return Eigen::Matrix3d::Identity();
    else
        return Eigen::Matrix3d::Identity() + W/2 + W*W*(1.0/d2 - (1.0+cos(d))/(2.0*d*sin(d)));
}

Eigen::Matrix3d LidarCovisRes::RightJacobianSO3(const Eigen::Vector3d &v)
{
    return RightJacobianSO3(v[0],v[1],v[2]);
}

Eigen::Matrix3d LidarCovisRes::RightJacobianSO3(const double x, const double y, const double z)
{
    const double d2 = x*x+y*y+z*z;
    const double d = sqrt(d2);

    Eigen::Matrix3d W;
    W << 0.0, -z, y,z, 0.0, -x,-y,  x, 0.0;
    if(d<1e-5)
    {
        return Eigen::Matrix3d::Identity();
    }
    else
    {
        return Eigen::Matrix3d::Identity() - W*(1.0-cos(d))/d2 + W*W*(d-sin(d))/(d2*d);
    }
}

Eigen::Matrix3d LidarCovisRes::Skew(const Eigen::Vector3d &w)
{
    Eigen::Matrix3d W;
    W << 0.0, -w[2], w[1],w[2], 0.0, -w[0],-w[1],  w[0], 0.0;
    return W;
}
}