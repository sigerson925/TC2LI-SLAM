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

#include "G2oTypesWithLidar.h"
#include "ImuTypes.h"
#include "Converter.h"
namespace TC2LI_SLAM
{
EdgeLidar::EdgeLidar(LidarCovisRes* pLio)
{
    lio = pLio;
    r1 = 1000;
    r2 = 1000;
    is_calc_hess = true;
}
void EdgeLidar::computeError()
{
    for(int i=0;i<_vertices.size();i++)
    {
        const VertexPose* VPi = static_cast<const VertexPose*>(_vertices[i]);
        lio->UpdatePose(i, VPi->estimate().Rcw[0], VPi->estimate().tcw[0]);
    }
    double r = lio->ComputeError();
    _error << std::sqrt(r);
    
    r1 = r2;
    r2 = r;
    if(r1 - r2 < 0) is_calc_hess = false;
    else
        is_calc_hess = true;
}
void EdgeLidar::BuildVoxHess()
{
    lio->BuildVoxHess();
}
void EdgeLidar::linearizeOplus()
{
    for(int i=0;i<_vertices.size();i++)
    {
        const VertexPose* VPi = static_cast<const VertexPose*>(_vertices[i]);
        lio->UpdatePose(i, VPi->estimate().Rcw[0], VPi->estimate().tcw[0]);
    }
    if(is_calc_hess)
        lio->ComputeJandH(JacT, Hessian);
    for(int i=0;i<_jacobianOplus.size();i++)
    {
        _jacobianOplus[i].setZero();
        _jacobianOplus[i] = JacT.block<6, 1>(6*i, 0).transpose();
    }
}
void EdgeLidar::constructQuadraticForm()
{
    if (this->robustKernel()) {
    double error = this->chi2();
    Eigen::Vector3d rho;
    this->robustKernel()->robustify(error, rho);
    Eigen::Matrix<double, 1, 1> omega_r = - _information * _error;
    omega_r *= rho[1];
        computeQuadraticFormLidarRes(this->robustInformation(rho), omega_r);
  } else {
        computeQuadraticFormLidarRes(_information, -_information * _error);
  }
}
void EdgeLidar::computeQuadraticFormLidarRes(const InformationType& omega, const ErrorVector& weightedError)
{
    for (size_t i = 0; i < _vertices.size(); ++i) {
    g2o::OptimizableGraph::Vertex* from = static_cast<g2o::OptimizableGraph::Vertex*>(_vertices[i]);
    bool istatus = !(from->fixed());

    if (istatus) {
        const Eigen::MatrixXd& A = _jacobianOplus[i];

        Eigen::MatrixXd AtO = A.transpose() * omega;
        int fromDim = from->dimension();
        assert(fromDim >= 0);
        Eigen::MatrixXd E(fromDim, fromDim);
        E.setIdentity();
        double info = *omega.data();
        Eigen::Map<Eigen::MatrixXd> fromMap(from->hessianData(), fromDim, fromDim);
        Eigen::Map<Eigen::VectorXd> fromB(from->bData(), fromDim);
#ifdef G2O_OPENMP
      from->lockQuadraticForm();
#endif
        fromMap.noalias() += Hessian.block<6,6>(i,i) * info;
        fromB.noalias() -= A.transpose() * info;

        // compute the off-diagonal blocks ij for all j
        for (size_t j = i+1; j < _vertices.size(); ++j) {
        g2o::OptimizableGraph::Vertex* to = static_cast<g2o::OptimizableGraph::Vertex*>(_vertices[j]);
#ifdef G2O_OPENMP
        to->lockQuadraticForm();
#endif
        bool jstatus = !(to->fixed());
        if (jstatus) {
          const Eigen::MatrixXd& B = _jacobianOplus[j];
          int idx = g2o::internal::computeUpperTriangleIndex(i, j);
          assert(idx < (int)_hessian.size());
          HessianHelper& hhelper = _hessian[idx];
          if (hhelper.transposed) { // we have to write to the block as transposed
            hhelper.matrix.noalias() += Hessian.block<6,6>(j,i) * info;
          } else {
            hhelper.matrix.noalias() += Hessian.block<6,6>(i,j) * info;
          }
        }
#ifdef G2O_OPENMP
        to->unlockQuadraticForm();
#endif
      }

#ifdef G2O_OPENMP
      from->unlockQuadraticForm();
#endif
    }

  }
}

}
