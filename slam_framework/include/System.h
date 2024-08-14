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


#ifndef SYSTEM_H
#define SYSTEM_H


#include <unistd.h>
#include<stdio.h>
#include<stdlib.h>
#include<string>
#include<thread>
#include<opencv2/core/core.hpp>

#include "Tracking.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Atlas.h"
#include "LocalMapping.h"
#include "KeyFrameDatabase.h"
#include "ORBVocabulary.h"
#include "Viewer.h"
#include "ImuTypes.h"
#include "LidarTypes.h"
#include "Settings.h"
#include "lidar_front_end/LidarFrontEnd.h"

namespace TC2LI_SLAM
{

class Verbose
{
public:
    enum eLevel
    {
        VERBOSITY_QUIET=0,
        VERBOSITY_NORMAL=1,
        VERBOSITY_VERBOSE=2,
        VERBOSITY_VERY_VERBOSE=3,
        VERBOSITY_DEBUG=4
    };

    static eLevel th;

public:
    static void PrintMess(std::string str, eLevel lev)
    {
        if(lev <= th)
            cout << str << endl;
    }

    static void SetTh(eLevel _th)
    {
        th = _th;
    }
};
class RvizViewer;
class FrameDrawer;
class MapDrawer;
class Atlas;
class Tracking;
class LocalMapping;
class Settings;

class System
{
public:
    // Input sensor
    enum eSensor{
        STEREO_LIDAR,
        IMU_STEREO_LIDAR,
    };

    // File type
    enum FileType{
        TEXT_FILE=0,
        BINARY_FILE=1,
    };

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
    System(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true, const int initFr = 0, const string &strSequence = std::string());

    Sophus::SE3f TrackStereoLidar(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp, const vector<IMU::Point>& vImuMeas = vector<IMU::Point>(), string filename="");


    // This stops local mapping thread (map building) and performs only camera tracking.
    void ActivateLocalizationMode();
    // This resumes local mapping thread and performs SLAM again.
    void DeactivateLocalizationMode();

    // Returns true if there have been a big map change (loop closure, global BA)
    // since last call to this function
    bool MapChanged();

    // Reset the system (clear Atlas or the active map)
    void Reset();
    void ResetActiveMap();

    // All threads will be requested to finish.
    // It waits until all threads have finished.
    // This function must be called before saving the trajectory.
    void Shutdown();
    bool isShutDown();

    // Save camera trajectory in the TUM RGB-D dataset format.
    // Only for stereo and RGB-D. This method does not work for monocular.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveTrajectoryTUM(const string &filename);

    // Save keyframe poses in the TUM RGB-D dataset format.
    // This method works for all sensor input.
    // Call first Shutdown()
    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
    void SaveKeyFrameTrajectoryTUM(const string &filename);

    void SaveKeyFrameTrajectoryEuRoC(const string &filename);

    void SaveKeyFrameTrajectoryEuRoC(const string &filename, Map* pMap);

    // Save camera trajectory in the KITTI dataset format.
    // Only for stereo and RGB-D. This method does not work for monocular.
    // Call first Shutdown()
    // See format details at: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
    void SaveTrajectoryKITTI(const string &filename);

    // TODO: Save/Load functions
    // SaveMap(const string &filename);
    // LoadMap(const string &filename);

    void InitRvizViewers(ros::NodeHandle& nh, std::string node_name);
    void PublishMsgs(ros::Time imageTime);

    // Information from most recent processed frame
    // You can call this right after TrackMonocular (or stereo or RGBD)
    int GetTrackingState();
    std::vector<MapPoint*> GetTrackedMapPoints();
    std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

    // For debugging
    double GetTimeFromIMUInit();
    bool isLost();
    bool isFinished();

    void ChangeDataset();

    float GetImageScale();

#ifdef REGISTER_TIMES
    void InsertRectTime(double& time);
    void InsertResizeTime(double& time);
    void InsertTrackTime(double& time);
#endif

private:
    //* added functions
    RvizViewer* mpRvizeViewer;
    vector<MapPoint*> GetAllMapPoints();
    Frame GetCurrentFrame();
    bool IsImuInitialized();
    vector<KeyFrame*> GetAllKeyFrames();
    //! ---------------------------------
    void SaveAtlas(int type);
    bool LoadAtlas(int type);

    string CalculateCheckSum(string filename, int type);

    // Input sensor
    eSensor mSensor;

    // ORB vocabulary used for place recognition and feature matching.
    ORBVocabulary* mpVocabulary;

    // KeyFrame database for place recognition (relocalization and loop detection).
    KeyFrameDatabase* mpKeyFrameDatabase;

    // Map structure that stores the pointers to all KeyFrames and MapPoints.
    //Map* mpMap;
    Atlas* mpAtlas;

    // Tracker. It receives a frame and computes the associated camera pose.
    // It also decides when to insert a new keyframe, create some new MapPoints and
    // performs relocalization if tracking fails.
public:
    Tracking* mpTracker;

private:
    // Local Mapper. It manages the local map and performs local bundle adjustment.
    LocalMapping* mpLocalMapper;

    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;

    // System threads: Local Mapping, Loop Closing, Viewer.
    // The Tracking thread "lives" in the main execution thread that creates the System object.
    std::thread* mptLocalMapping;

    // Reset flag
    std::mutex mMutexReset;
    bool mbReset;
    bool mbResetActiveMap;

    // Change mode flags
    std::mutex mMutexMode;
    bool mbActivateLocalizationMode;
    bool mbDeactivateLocalizationMode;

    // Shutdown flag
    bool mbShutDown;

    // Tracking state
    int mTrackingState;
    std::vector<MapPoint*> mTrackedMapPoints;
    std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
    std::mutex mMutexState;

    //
    string mStrLoadAtlasFromFile;
    string mStrSaveAtlasToFile;

    string mStrVocabularyFilePath;

    Settings* settings_;
};

}// namespace TC2LI_SLAM

#endif // SYSTEM_H
