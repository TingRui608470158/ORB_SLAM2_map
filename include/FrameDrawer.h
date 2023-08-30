/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef FRAMEDRAWER_H
#define FRAMEDRAWER_H

#include "Tracking.h"
#include "MapPoint.h"
#include "Map.h"

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include <cmath>

#include<mutex>


namespace ORB_SLAM2
{

class Tracking;
class Viewer;

class Object_data
{
public:
    std::string name; 
    float x_mid ;
    float y_mid ;
    float z_mid ;
    float x_range ;
    float y_range ;
    float z_range ;
};


class FrameDrawer
{
public:
    FrameDrawer(Map* pMap, Tracking *pTracking);

    // Update info from the last processed frame.
    void Update(Tracking *pTracker);

    // Draw last processed frame.
    cv::Mat DrawFrame();
    void SavePoint();
    void objectdetection();
    void DrawBoundingBox(float x_min,float x_max,float y_min,float y_max, cv::Scalar box_color);
    void DrawIcon(int obj_index, float x_min, float x_max, float y_min, float y_max);
    void DrawIcon(int obj_index, int keypoint_index, float upper_y_range);
    void DrawMidPoint(int mid_point, cv::Scalar box_color);


    int GetObjectMidIndex(int obj_index);
    void Init_Object_Data();

    vector<MapPoint*>  All_MapPoint;
    vector<Object_data> Object_Pose;



protected:

    void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);

    // Info of the frame to be drawn
    cv::Mat mIm;
    int N;
    vector<cv::KeyPoint> mvCurrentKeys;
    vector<bool> mvbMap, mvbVO;
    bool mbOnlyTracking;
    int mnTracked, mnTrackedVO;
    vector<cv::KeyPoint> mvIniKeys;
    vector<int> mvIniMatches;
    int mState;
    vector<bool>  bmouse_click;
    vector<cv::Mat> WorldPos_mouseclick;
     
    Map* mpMap;
    Tracking* mpTracker;
    std::mutex mMutex;
};

} //namespace ORB_SLAM

#endif // FRAMEDRAWER_H
