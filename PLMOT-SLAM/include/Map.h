/**
* This file is part of ORB-LINE-SLAM
*
* Copyright (C) 2020-2021 John Alamanos, National Technical University of Athens.
* Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
* Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
*
* ORB-LINE-SLAM is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
* License as published by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-LINE-SLAM is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
* the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License along with ORB-LINE-SLAM.
* If not, see <http://www.gnu.org/licenses/>.
*/


#ifndef MAP_H
#define MAP_H

#include "MapPoint.h"
#include "KeyFrame.h"
#include "MapLine.h"
#include <set>
#include <pangolin/pangolin.h>
#include <mutex>

#include <boost/serialization/base_object.hpp>


namespace ORB_SLAM3
{

class MapPoint;
class MapLine;
class KeyFrame;
class Atlas;
class KeyFrameDatabase;

class Map
{
    friend class boost::serialization::access;

    template<class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
        ar & mnId;
        ar & mnInitKFid;
        ar & mnMaxKFid;
        ar & mnBigChangeIdx;
        // Set of KeyFrames and MapPoints, in this version the set serializator is not working
        //ar & mspKeyFrames;
        //ar & mspMapPoints;

        ar & mvpBackupKeyFrames;
        ar & mvpBackupMapPoints;

        ar & mvBackupKeyFrameOriginsId;

        ar & mnBackupKFinitialID;
        ar & mnBackupKFlowerID;

        ar & mbImuInitialized;
        ar & mbIsInertial;
        ar & mbIMU_BA1;
        ar & mbIMU_BA2;

        ar & mnInitKFid;
        ar & mnMaxKFid;
        ar & mnLastLoopKFid;
    }

public:
    Map();
    Map(int initKFid);
    ~Map();

    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    void AddMapLine(MapLine* pML);
    void EraseMapPoint(MapPoint* pMP);
    void EraseMapLine(MapLine* pML);
    void AddMapDelaunayLine(cv::Mat pMPDl);
    // add semantic mapoints
    void AddSemanticMapPoint(cv::Mat pMP, std::vector<float> color);
    void AddCameraTrajectory(cv::Mat mMP);
    void AddBound3D(std::vector<float> min_max, string label);
    void AddPersonTrack(cv::Mat center);
    void EraseKeyFrame(KeyFrame* pKF);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
    void SetReferenceMapLines(const std::vector<MapLine*> &vpMLs);
    void InformNewBigChange();
    int GetLastBigChangeIdx();

    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapLine*> GetAllMapLines();
    std::vector<MapPoint*> GetReferenceMapPoints();
    std::vector<MapLine*> GetReferenceMapLines();

    std::vector<cv::Mat> GetAllMapDelaunayLine();
    // 2023.05.23 获取地图中的所有语义地图点
    std::vector<pair<cv::Mat,vector<float>>> GetAllSemanticMapPoints();
    // 2023.05.28 获取地图中相机的光心地图点
    std::vector<cv::Mat> GetAllCameraCenter();
    // 2023.06.07 获取所有点云的包围框
    vector<pair<vector<float>, string>> GetAllBound3D();
    // 2023.06.08 获取地图中行人的轨迹点
    vector<cv::Mat> GetPersonTrack();

    long unsigned int MapPointsInMap();
    long unsigned int MapLinesInMap();
    long unsigned  KeyFramesInMap();

    long unsigned int GetId();

    long unsigned int GetInitKFid();
    void SetInitKFid(long unsigned int initKFif);
    long unsigned int GetMaxKFid();

    KeyFrame* GetOriginKF();

    void SetCurrentMap();
    void SetStoredMap();

    bool HasThumbnail();
    bool IsInUse();

    void SetBad();
    bool IsBad();

    void clear();

    int GetMapChangeIndex();
    void IncreaseChangeIndex();
    int GetLastMapChange();
    void SetLastMapChange(int currentChangeId);

    void SetImuInitialized();
    bool isImuInitialized();

    void RotateMap(const cv::Mat &R);
    void ApplyScaledRotation(const cv::Mat &R, const float s, const bool bScaledVel=false, const cv::Mat t=cv::Mat::zeros(cv::Size(1,3),CV_32F));

    void SetInertialSensor();
    bool IsInertial();
    void SetIniertialBA1();
    void SetIniertialBA2();
    bool GetIniertialBA1();
    bool GetIniertialBA2();

    void PrintEssentialGraph();
    bool CheckEssentialGraph();
    void ChangeId(long unsigned int nId);

    unsigned int GetLowerKFID();

    void PreSave(std::set<GeometricCamera*> &spCams);
    void PostLoad(KeyFrameDatabase* pKFDB, ORBVocabulary* pORBVoc, map<long unsigned int, KeyFrame*>& mpKeyFrameId, map<unsigned int, GeometricCamera*> &mpCams);

    void printReprojectionError(list<KeyFrame*> &lpLocalWindowKFs, KeyFrame* mpCurrentKF, string &name, string &name_folder);

    vector<KeyFrame*> mvpKeyFrameOrigins;
    vector<unsigned long int> mvBackupKeyFrameOriginsId;
    KeyFrame* mpFirstRegionKF;
    std::mutex mMutexMapUpdate;

    // This avoid that two points are created simultaneously in separate threads (id conflict)
    std::mutex mMutexPointCreation;
    std::mutex mMutexLineCreation;

    bool mbFail;

    // Size of the thumbnail (always in power of 2)
    static const int THUMB_WIDTH = 512;
    static const int THUMB_HEIGHT = 512;

    static long unsigned int nNextId;

    // semantic mappoints
    std::vector<pair<cv::Mat, std::vector<float>>> mspSemanticMapPoints;
    // camera track
    std::vector<cv::Mat> mspCameraCenter;
    // bounding box
    std::vector<pair<std::vector<float>, std::string>> mspBound3D;
    pangolin::GlFont *text_font = new pangolin::GlFont("/home/kesai/SLAM_ROS2/src/RGBD/Anonymous-Pro-Bold.ttf",20.0);
    std::vector<cv::Mat> mspPersonTrack;
    std::vector<cv::Mat> mspMapDelaunayLines;

protected:

    long unsigned int mnId;

    std::set<MapPoint*> mspMapPoints;
    std::set<MapLine*> mspMapLines;
    std::set<KeyFrame*> mspKeyFrames;

    std::vector<MapPoint*> mvpBackupMapPoints;
    std::vector<KeyFrame*> mvpBackupKeyFrames;

    KeyFrame* mpKFinitial;
    KeyFrame* mpKFlowerID;

    unsigned long int mnBackupKFinitialID;
    unsigned long int mnBackupKFlowerID;

    std::vector<MapPoint*> mvpReferenceMapPoints;
    std::vector<MapLine*> mvpReferenceMapLines;

    bool mbImuInitialized;

    int mnMapChange;
    int mnMapChangeNotified;

    long unsigned int mnInitKFid;
    long unsigned int mnMaxKFid;
    long unsigned int mnLastLoopKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;


    // View of the map in aerial sight (for the AtlasViewer)
    GLubyte* mThumbnail;

    bool mIsInUse;
    bool mHasTumbnail;
    bool mbBad = false;

    bool mbIsInertial;
    bool mbIMU_BA1;
    bool mbIMU_BA2;

    std::mutex mMutexMap;
};

} //namespace ORB_SLAM3

#endif // MAP_H
