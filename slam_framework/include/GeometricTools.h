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


#ifndef GEOMETRIC_TOOLS_H
#define GEOMETRIC_TOOLS_H

#include <opencv2/core/core.hpp>
#include <sophus/se3.hpp>
#include <Eigen/Core>

namespace TC2LI_SLAM
{

class KeyFrame;

class GeometricTools
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // Compute the Fundamental matrix between KF1 and KF2
    static Eigen::Matrix3f ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);

    //Triangulate point with KF1 and KF2
    static bool Triangulate(Eigen::Vector3f &x_c1, Eigen::Vector3f &x_c2,Eigen::Matrix<float,3,4> &Tc1w ,Eigen::Matrix<float,3,4> &Tc2w , Eigen::Vector3f &x3D);

    template<int rows, int cols>
    static bool CheckMatrices(const cv::Mat &cvMat, const Eigen::Matrix<float,rows,cols> &eigMat) {
        const float epsilon = 1e-3;
        if(rows != cvMat.rows || cols != cvMat.cols) {
            std::cout << "wrong cvmat size\n";
            return false;
        }
        for(int i = 0; i < rows; i++)
            for(int j = 0; j < cols; j++)
                if ((cvMat.at<float>(i,j) > (eigMat(i,j) + epsilon)) ||
                    (cvMat.at<float>(i,j) < (eigMat(i,j) - epsilon))){
                    std::cout << "cv mat:\n" << cvMat << std::endl;
                    std::cout << "eig mat:\n" << eigMat << std::endl;
                    return false;
                }
        return true;
    }

    template<typename T, int rows, int cols>
    static bool CheckMatrices( const Eigen::Matrix<T,rows,cols> &eigMat1, const Eigen::Matrix<T,rows,cols> &eigMat2) {
        const float epsilon = 1e-3;
        for(int i = 0; i < rows; i++)
            for(int j = 0; j < cols; j++)
                if ((eigMat1(i,j) > (eigMat2(i,j) + epsilon)) ||
                    (eigMat1(i,j) < (eigMat2(i,j) - epsilon))){
                    std::cout << "eig mat 1:\n" << eigMat1 << std::endl;
                    std::cout << "eig mat 2:\n" << eigMat2 << std::endl;
                    return false;
                }
        return true;
    }

};

}// namespace TC2LI_SLAM

#endif // GEOMETRIC_TOOLS_H
