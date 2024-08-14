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
#include "tools.h"

using namespace std;

Eigen::Matrix3d I33(Eigen::Matrix3d::Identity());
Eigen::Matrix<double, DIMU, DIMU> I_imu(Eigen::Matrix<double, DIMU, DIMU>::Identity());
Eigen::Matrix<double, 12, 12> I12(Eigen::Matrix<double, 12, 12>::Identity());


Eigen::Matrix3d Exp(const Eigen::Vector3d &ang)
{
  double ang_norm = ang.norm();
  // if (ang_norm >= 0.0000001)
  if (ang_norm >= 1e-11)
  {
    Eigen::Vector3d r_axis = ang / ang_norm;
    Eigen::Matrix3d K;
    K << SKEW_SYM_MATRX(r_axis);
    /// Roderigous Tranformation
    return I33 + std::sin(ang_norm) * K + (1.0 - std::cos(ang_norm)) * K * K;
  }
  
  return I33;
  
}

Eigen::Matrix3d Exp(const Eigen::Vector3d &ang_vel, const double &dt)
{
  double ang_vel_norm = ang_vel.norm();
  if (ang_vel_norm > 0.0000001)
  {
    Eigen::Vector3d r_axis = ang_vel / ang_vel_norm;
    Eigen::Matrix3d K;

    K << SKEW_SYM_MATRX(r_axis);
    double r_ang = ang_vel_norm * dt;

    /// Roderigous Tranformation
    return I33 + std::sin(r_ang) * K + (1.0 - std::cos(r_ang)) * K * K;
  }
  
  return I33;
  
}

Eigen::Vector3d Log(const Eigen::Matrix3d &R)
{
  double theta = (R.trace() > 3.0 - 1e-6) ? 0.0 : std::acos(0.5 * (R.trace() - 1));
  Eigen::Vector3d K(R(2,1) - R(1,2), R(0,2) - R(2,0), R(1,0) - R(0,1));
  return (std::abs(theta) < 0.001) ? (0.5 * K) : (0.5 * theta / std::sin(theta) * K);
}

Eigen::Matrix3d hat(const Eigen::Vector3d &v)
{
  Eigen::Matrix3d Omega;
  Omega <<  0, -v(2),  v(1)
      ,  v(2),     0, -v(0)
      , -v(1),  v(0),     0;
  return Omega;
}

Eigen::Matrix3d jr(Eigen::Vector3d vec)
{
  double ang = vec.norm();

  if(ang < 1e-9)
  {
    return I33;
  }
  else
  {
    vec /= ang;
    double ra = sin(ang)/ang;
    return ra*I33 + (1-ra)*vec*vec.transpose() - (1-cos(ang))/ang * hat(vec);
  }
}

Eigen::Matrix3d jr_inv(const Eigen::Matrix3d &rotR)
{
  Eigen::AngleAxisd rot_vec(rotR);
  Eigen::Vector3d axi = rot_vec.axis();
  double ang = rot_vec.angle();

  if(ang < 1e-9)
  {
    return I33;
  }
  else
  {
    double ctt = ang / 2 / tan(ang/2);
    return ctt*I33 + (1-ctt)*axi*axi.transpose() + ang/2 * hat(axi);
  }
}


void down_sampling_voxel(pcl::PointCloud<PointXYZINormal> &pl_feat, double voxel_size)
{
  if(voxel_size < 0.001) return;

  unordered_map<VOXEL_LOC, PointXYZINormal> feat_map;
  float loc_xyz[3];
  for(PointXYZINormal &p_c : pl_feat.points)
  {
    for(int j=0; j<3; j++)
    {
      loc_xyz[j] = p_c.data[j] / voxel_size;
      if(loc_xyz[j] < 0)
        loc_xyz[j] -= 1.0;
    }

    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
    auto iter = feat_map.find(position);
    if(iter == feat_map.end())
    {
      PointXYZINormal pp = p_c;
      pp.curvature = 1;
      feat_map[position] = pp;
    }
    else
    {
      PointXYZINormal &pp = iter->second;
      pp.x = (pp.x * pp.curvature + p_c.x) / (pp.curvature + 1);
      pp.y = (pp.y * pp.curvature + p_c.y) / (pp.curvature + 1);
      pp.z = (pp.z * pp.curvature + p_c.z) / (pp.curvature + 1);
      pp.curvature += 1;
    }
  }

  // pl_feat.clear();
  pcl::PointCloud<PointXYZINormal> pl_feat2;
  pl_feat.swap(pl_feat2); pl_feat.reserve(feat_map.size());
  for(auto iter=feat_map.begin(); iter!=feat_map.end(); ++iter)
    pl_feat.push_back(iter->second);
  
}

void down_sampling_serie(pcl::PointCloud<PointXYZINormal> &pl_feat, int num)
{
  if(num < 1) num = 1;

  pcl::PointCloud<PointXYZINormal> pl_down;
  int psize = pl_feat.size();
  pl_down.reserve(psize);
  for(int i=0; i<psize; i+=num)
    pl_down.push_back(pl_feat[i]);
  pl_feat.swap(pl_down);
}

void pl_transform(pcl::PointCloud<PointXYZINormal> &pl1, const Eigen::Matrix3d &rr, const Eigen::Vector3d &tt)
{
  for(PointXYZINormal &ap : pl1.points)
  {
    Eigen::Vector3d pvec(ap.x, ap.y, ap.z);
    pvec = rr * pvec + tt;
    ap.x = pvec[0];
    ap.y = pvec[1];
    ap.z = pvec[2];
  }
}

void pl_transform(pcl::PointCloud<PointXYZINormal> &pl1, const IMUST &xx)
{
  for(PointXYZINormal &ap : pl1.points)
  {
    Eigen::Vector3d pvec(ap.x, ap.y, ap.z);
    pvec = xx.R * pvec + xx.p;
    ap.x = pvec[0];
    ap.y = pvec[1];
    ap.z = pvec[2];
  }
}

void plvec_trans(PLV(3) &porig, PLV(3) &ptran, IMUST &stat)
{
  uint asize = porig.size();
  ptran.resize(asize);
  for(uint i=0; i<asize; i++)
    ptran[i] = stat.R * porig[i] + stat.p;
}

bool time_compare(PointXYZINormal &x, PointXYZINormal &y) {return (x.curvature < y.curvature);}

/* comment
plane equation: Ax + By + Cz + D = 0
convert to: A/D*x + B/D*y + C/D*z = -1
solve: A0*x0 = b0
where A0_i = [x_i, y_i, z_i], x0 = [A/D, B/D, C/D]^T, b0 = [-1, ..., -1]^T
normvec:  normalized x0
*/


