
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
#include "bavoxel.h"

int layer_limit = 2;
int layer_size[] = {30, 30, 30, 30};
float eigen_value_array[4] = {1.0/36, 1.0/25, 1.0/25, 1.0/25};
int min_ps = 15;
double one_three = (1.0 / 3.0);

double voxel_size = 1;
int life_span = 1000;
int win_size = 30;

int merge_enable = 1;

void cut_voxel(unordered_map<VOXEL_LOC, OCTO_TREE_ROOT*> &feat_map, pcl::PointCloud<PointXYZINormal> &pl_feat, const IMUST &x_key, int fnum, int win_size)
{
  float loc_xyz[3];
  for(PointXYZINormal &p_c : pl_feat.points)
  {
    Eigen::Vector3d pvec_orig(p_c.x, p_c.y, p_c.z);
    Eigen::Vector3d pvec_tran = x_key.R*pvec_orig + x_key.p;

    for(int j=0; j<3; j++)
    {
      loc_xyz[j] = pvec_tran[j] / voxel_size;
      if(loc_xyz[j] < 0) loc_xyz[j] -= 1.0;
    }
    VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1], (int64_t)loc_xyz[2]);
    auto iter = feat_map.find(position);
    if(iter != feat_map.end())
    {
        if(iter->second->octo_state != 2)
        {
          iter->second->vec_orig[fnum].push_back(pvec_orig);
          iter->second->vec_tran[fnum].push_back(pvec_tran);
        }
        
        if(iter->second->octo_state != 1)
        {
          //unknown
          iter->second->sig_orig[fnum].push(pvec_orig);
          iter->second->sig_tran[fnum].push(pvec_tran);
        }
        iter->second->is2opt = true;
        iter->second->life = life_span;
        iter->second->each_num[fnum]++;
    }
    else
    {
        OCTO_TREE_ROOT *ot = new OCTO_TREE_ROOT(win_size);
        ot->vec_orig[fnum].push_back(pvec_orig);
        ot->vec_tran[fnum].push_back(pvec_tran);
        ot->sig_orig[fnum].push(pvec_orig);
        ot->sig_tran[fnum].push(pvec_tran);
        ot->each_num[fnum]++;
        ot->voxel_center[0] = (0.5+position.x) * voxel_size;
        ot->voxel_center[1] = (0.5+position.y) * voxel_size;
        ot->voxel_center[2] = (0.5+position.z) * voxel_size;
        ot->quater_length = voxel_size / 4.0;
        ot->layer = 0;
        feat_map[position] = ot;
    }
  }
}

