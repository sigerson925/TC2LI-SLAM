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


#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H

#include "Tracking.h"
#include "MapPoint.h"
#include "Atlas.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include<mutex>
#include <unordered_set>


namespace TC2LI_SLAM
{

class Tracking;

class FrameDrawer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    FrameDrawer(Atlas* pAtlas);

    // Update info from the last processed frame.
    void Update(Tracking *pTracker);

    // Draw last processed frame.
    cv::Mat DrawFrame(float imageScale=1.f);
    cv::Mat DrawRightFrame(float imageScale=1.f);

    bool both;

protected:

    void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);

    // Info of the frame to be drawn
    cv::Mat mIm, mImRight;
    int N;
    vector<cv::KeyPoint> mvCurrentKeys,mvCurrentKeysRight;
    vector<bool> mvbMap, mvbVO;
    bool mbOnlyTracking;
    int mnTracked, mnTrackedVO;
    vector<cv::KeyPoint> mvIniKeys;
    vector<int> mvIniMatches;
    int mState;
    std::vector<float> mvCurrentDepth;
    float mThDepth;

    Atlas* mpAtlas;

    std::mutex mMutex;
    vector<pair<cv::Point2f, cv::Point2f> > mvTracks;

    Frame mCurrentFrame;
    vector<MapPoint*> mvpLocalMap;
    vector<cv::KeyPoint> mvMatchedKeys;
    vector<MapPoint*> mvpMatchedMPs;
    vector<cv::KeyPoint> mvOutlierKeys;
    vector<MapPoint*> mvpOutlierMPs;

    map<long unsigned int, cv::Point2f> mmProjectPoints;
    map<long unsigned int, cv::Point2f> mmMatchedInImage;

};

} //namespace TC2LI_SLAM

#endif // FRAMEDRAWER_H
