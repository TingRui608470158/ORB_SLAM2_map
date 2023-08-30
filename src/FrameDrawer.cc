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

#include "FrameDrawer.h"
#include "Tracking.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include<mutex>

namespace ORB_SLAM2
{

FrameDrawer::FrameDrawer(Map* pMap, Tracking* pTracking):mpMap(pMap),mpTracker(pTracking)
{
    mState=Tracking::SYSTEM_NOT_READY;
    mIm = cv::Mat(480,640,CV_8UC3, cv::Scalar(0,0,0));

    Init_Object_Data();
}

cv::Mat FrameDrawer::DrawFrame()
{
    cv::Mat im;
    vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
    vector<int> vMatches; // Initialization: correspondeces with reference keypoints
    vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
    vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
    int state; // Tracking state

    //Copy variables within scoped mutex
    {
        unique_lock<mutex> lock(mMutex);
        state=mState;
        if(mState==Tracking::SYSTEM_NOT_READY)
            mState=Tracking::NO_IMAGES_YET;

        mIm.copyTo(im);

        if(mState==Tracking::NOT_INITIALIZED)
        {
            vCurrentKeys = mvCurrentKeys;
            vIniKeys = mvIniKeys;
            vMatches = mvIniMatches;
        }
        else if(mState==Tracking::OK)
        {
            vCurrentKeys = mvCurrentKeys;
            vbVO = mvbVO;
            vbMap = mvbMap;
        }
        else if(mState==Tracking::LOST)
        {
            vCurrentKeys = mvCurrentKeys;
        }
    } // destroy scoped mutex -> release mutex

    if(im.channels()<3) //this should be always true
        cvtColor(im,im,CV_GRAY2BGR);

    //Draw
    if(state==Tracking::NOT_INITIALIZED) //INITIALIZING
    {
        for(unsigned int i=0; i<vMatches.size(); i++)
        {
            if(vMatches[i]>=0)
            {
                cv::line(im,vIniKeys[i].pt,vCurrentKeys[vMatches[i]].pt,
                        cv::Scalar(0,255,0));
            }
        }        
    }
    else if(state==Tracking::OK) //TRACKING
    {
        mnTracked=0;
        mnTrackedVO=0;
        const float r = 5;
        const int n = vCurrentKeys.size();
        bool show_once = false;
        for(int i=0;i<n;i++)
        {
            if(vbVO[i] || vbMap[i])
            {
                // cout<<"mpTracker->bmouse_click[i] = "<<bmouse_click[i]<<endl;
                cv::Point2f pt1,pt2;
                pt1.x=vCurrentKeys[i].pt.x-r;
                pt1.y=vCurrentKeys[i].pt.y-r;
                pt2.x=vCurrentKeys[i].pt.x+r;
                pt2.y=vCurrentKeys[i].pt.y+r;
                
                // This is a match to a MapPoint in the map
                if(vbMap[i] && !bmouse_click[i])
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(0,255,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(0,255,0),-1);
                    mnTracked++;
                }
                else if(vbMap[i] && bmouse_click[i])
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(0,0,255));
                    cv::circle(im,vCurrentKeys[i].pt,4,cv::Scalar(0,0,255),-1);
                    if(!show_once)
                    {
                        show_once = true;
                        float Pose_x = round(WorldPos_mouseclick[i].at<float>(0) * 100) / 100; 
                        float Pose_y = round(WorldPos_mouseclick[i].at<float>(1) * 100) / 100; 
                        float Pose_z = round(WorldPos_mouseclick[i].at<float>(2) * 100) / 100; 
                        ostringstream stream1,stream2,stream3;
                        stream1 << std::fixed << std::setprecision(2) << Pose_x; // 设置输出格式为小数点后两位
                        stream2 << std::fixed << std::setprecision(2) << Pose_y;
                        stream3 << std::fixed << std::setprecision(2) << Pose_z;
                        string Pose_x_str = stream1.str();
                        string Pose_y_str = stream2.str();
                        string Pose_z_str = stream3.str();


                        cout<< "WorldPos_mouseclick[i] = "<< WorldPos_mouseclick[i]<<endl;
                        cout<< "WorldPos_mouseclick[i].size = "<< WorldPos_mouseclick[i].size()<<endl;
                        cout<< "WorldPos_mouseclick[i].x = "<< WorldPos_mouseclick[i].at<float>(0)<<endl;
                        cout<< "WorldPos_mouseclick[i].y = "<< WorldPos_mouseclick[i].at<float>(1)<<endl;
                        cout<< "WorldPos_mouseclick[i].z = "<< WorldPos_mouseclick[i].at<float>(2)<<endl;
                        cout<< "Pose_x = "<< Pose_x <<endl;
                        cout<< "Pose_y = "<< Pose_y <<endl;
                        cout<< "Pose_z = "<< Pose_z <<endl;

                        string WorldPos_text ="("+ Pose_x_str +", "+Pose_y_str+", "+Pose_z_str + ")";
                        cv::Point textPosition(vCurrentKeys[i].pt.x, vCurrentKeys[i].pt.y - 15);
                        cv::putText(im, WorldPos_text, textPosition, cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 2);
                    }
                    mnTracked++;
                }
                else // This is match to a "visual odometry" MapPoint created in the last frame
                {
                    cv::rectangle(im,pt1,pt2,cv::Scalar(255,0,0));
                    cv::circle(im,vCurrentKeys[i].pt,2,cv::Scalar(255,0,0),-1);
                    mnTrackedVO++;
                }
            }
        }
    }

    cv::Mat imWithInfo;
    DrawTextInfo(im,state, imWithInfo);
    
    return imWithInfo;
}

void FrameDrawer::SavePoint()
{
    vector<bool> vbMap;
    vbMap = mvbMap;
    ofstream outputFile("./object_MapPoint.csv",std::ios::app);
    if (!outputFile.is_open()) {
        std::cerr << "Failed to open the file." << std::endl;
        return ;
    }
    for(int i=0;i<N;i++)
    {
        if(vbMap[i] && bmouse_click[i])
        {
            float Pose_x = WorldPos_mouseclick[i].at<float>(0);
            float Pose_y = WorldPos_mouseclick[i].at<float>(1);
            float Pose_z = WorldPos_mouseclick[i].at<float>(2);
            outputFile << Pose_x;
            outputFile << ",";
            outputFile << Pose_y;
            outputFile << ",";
            outputFile << Pose_z;
            outputFile << "\n";
        }
    }
    
}

void FrameDrawer::DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText)
{
    stringstream s;
    if(nState==Tracking::NO_IMAGES_YET)
        s << " WAITING FOR IMAGES";
    else if(nState==Tracking::NOT_INITIALIZED)
        s << " TRYING TO INITIALIZE ";
    else if(nState==Tracking::OK)
    {
        if(!mbOnlyTracking)
            s << "SLAM MODE |  ";
        else
            s << "LOCALIZATION | ";
        int nKFs = mpMap->KeyFramesInMap();
        int nMPs = mpMap->MapPointsInMap();
        s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
        if(mnTrackedVO>0)
            s << ", + VO matches: " << mnTrackedVO;
    }
    else if(nState==Tracking::LOST)
    {
        s << " TRACK LOST. TRYING TO RELOCALIZE ";
    }
    else if(nState==Tracking::SYSTEM_NOT_READY)
    {
        s << " LOADING ORB VOCABULARY. PLEASE WAIT...";
    }

    int baseline=0;
    cv::Size textSize = cv::getTextSize(s.str(),cv::FONT_HERSHEY_PLAIN,1,1,&baseline);

    imText = cv::Mat(im.rows+textSize.height+10,im.cols,im.type());
    im.copyTo(imText.rowRange(0,im.rows).colRange(0,im.cols));
    imText.rowRange(im.rows,imText.rows) = cv::Mat::zeros(textSize.height+10,im.cols,im.type());
    cv::putText(imText,s.str(),cv::Point(5,imText.rows-5),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(255,255,255),1,8);

}

void FrameDrawer::objectdetection()
{
    vector<bool> vbMap;
    float BB_xmin = 0;
    float BB_xmax = 0;
    float BB_ymin = 0;
    float BB_ymax = 0;
    
    cv::Point2f pt1;
    vbMap = mvbMap;
    vector<cv::KeyPoint> vCurrentKeys;   
    vCurrentKeys = mvCurrentKeys;
    cv::Scalar box_blue = cv::Scalar(255, 0, 0);
    cv::Scalar box_red = cv::Scalar(0, 0, 255);
    
    for(int j=0;j<Object_Pose.size(); j++)
    {
        int obj_count = 0;
        for(int i=0;i<N;i++)
        {
            if(vbMap[i])
            {
                MapPoint* pMP = All_MapPoint[i];
                cv::Mat MapPoint_Pose =  pMP->GetWorldPos();
                float Pose_x = MapPoint_Pose.at<float>(0);
                float Pose_y = MapPoint_Pose.at<float>(1);
                float Pose_z = MapPoint_Pose.at<float>(2);

                if( abs(Pose_x-Object_Pose[j].x_mid)<Object_Pose[j].x_range && abs(Pose_y-Object_Pose[j].y_mid)<Object_Pose[j].y_range && abs(Pose_z-Object_Pose[j].z_mid)<Object_Pose[j].z_range)
                {
                    obj_count++;
                    pt1.x=vCurrentKeys[i].pt.x;
                    pt1.y=vCurrentKeys[i].pt.y;     

                    if(BB_xmin == 0 && BB_xmax == 0 && BB_ymin == 0 && BB_ymax == 0 )
                    {
                        BB_xmin = pt1.x;
                        BB_xmax = pt1.x;
                        BB_ymin = pt1.y;
                        BB_ymax = pt1.y;
                    }
                    if(pt1.x < BB_xmin)
                        BB_xmin = pt1.x;
                    else if(pt1.x > BB_xmax)
                        BB_xmax = pt1.x;
                    if(pt1.y < BB_ymin)
                        BB_ymin = pt1.y;
                    else if(pt1.y > BB_ymax)
                        BB_ymax = pt1.y;
                }
            }
        }

        if(obj_count>20)
        {
            
            int mid_point = GetObjectMidIndex(j);
            DrawMidPoint(mid_point,box_red);
            DrawBoundingBox(BB_xmin,BB_xmax,BB_ymin,BB_ymax,box_blue);
            DrawIcon(j, BB_xmin, BB_xmax, BB_ymin, BB_ymax );
            // cout<< Object_Pose[j].name<< endl;
        }
    }

}

void FrameDrawer::DrawBoundingBox(float x_min,float x_max, float y_min, float y_max, cv::Scalar box_color)
{
    if(mIm.channels()<3) 
        cv::cvtColor(mIm,mIm,CV_GRAY2BGR);
    cv::rectangle(mIm, cv::Point(x_min, y_min), cv::Point(x_max, y_max), box_color, 2);
}

void FrameDrawer::DrawMidPoint(int mid_point, cv::Scalar box_color)
{
    if(mIm.channels()<3) 
        cv::cvtColor(mIm,mIm,CV_GRAY2BGR);
    vector<cv::KeyPoint> vCurrentKeys;   
    vCurrentKeys = mvCurrentKeys;
    cv::Point2f pt1;
    pt1.x = mvCurrentKeys[mid_point].pt.x;
    pt1.y = mvCurrentKeys[mid_point].pt.y;
    cv::rectangle(mIm, cv::Point(pt1.x-7, pt1.y-7), cv::Point(pt1.x+7, pt1.y+7), box_color, -1);
}

void FrameDrawer::DrawIcon(int obj_index, int keypoint_index, float upper_y_range)
{
    cv::Mat dst;
    mIm.copyTo(dst);
    // if(dst.channels()<3) 
    //     cv::cvtColor(dst,dst,CV_GRAY2BGR);

    std::string iconPath = "./Datasets/icon/" + Object_Pose[obj_index].name + ".jpg";

    cv::Mat Icon = cv::imread(iconPath,cv::IMREAD_GRAYSCALE);
    if (Icon.empty()) {
        std::cerr << "Could not read icon image: " << iconPath << std::endl;
        return;
    }

    cv::resize(Icon, Icon, cv::Size(30, 30));
    int Icon_width = Icon.cols;
    int Icon_height = Icon.rows;

    vector<cv::KeyPoint> vCurrentKeys;   
    vCurrentKeys = mvCurrentKeys;
    cv::Point2f pt1;
    pt1.x = vCurrentKeys[keypoint_index].pt.x;
    pt1.y = vCurrentKeys[keypoint_index].pt.y;

    int start_x = pt1.x - Icon_width/2;
    int start_y = pt1.y - upper_y_range - Icon_height;

    if (start_x > 0 && start_y > 0 && start_x+Icon_width < dst.cols && start_y+Icon_height < dst.rows) {
        for(int i=0; i<Icon_height; i++)
        {
            for(int j=0; j<Icon_width; j++)
            {
                dst.at<uchar>(start_y+i, start_x+j) = Icon.at<uchar>(i, j);
            }
        }
    }
    // cv::Scalar box_green = cv::Scalar(0, 255, 0);
    
    mIm = dst;

}


void FrameDrawer::DrawIcon(int obj_index, float x_min, float x_max, float y_min, float y_max)
{
    
    cv::Mat dst;
    mIm.copyTo(dst);
    if(dst.channels()<3) 
        cv::cvtColor(dst,dst,CV_GRAY2BGR);

    std::string iconPath = "./Datasets/icon/" + Object_Pose[obj_index].name + ".jpg";

    cv::Mat Icon = cv::imread(iconPath,cv::IMREAD_COLOR);
    if(Icon.channels()<3) 
        cv::cvtColor(Icon,Icon,CV_GRAY2BGR);

    if (Icon.empty()) {
        std::cerr << "Could not read icon image: " << iconPath << std::endl;
        return;
    }

    cv::resize(Icon, Icon, cv::Size(30, 30));

    int obj_x = (x_min + x_max) / 2;
    int obj_y = (y_min + y_max) / 2;
    int obj_y_range = (y_max - y_min) / 2;

    int Icon_width = Icon.cols;
    int Icon_height = Icon.rows;

    int start_x = obj_x;
    int start_y = obj_y - obj_y_range - Icon_height;
    
     if (start_x > 0 && start_y > 0 && start_x+Icon_width < dst.cols && start_y+Icon_height < dst.rows) {
        for(int i=0; i<Icon_height; i++)
        {
            for(int j=0; j<Icon_width; j++)
            {
                for (int channel = 0; channel < 3; channel++) 
                {
                    dst.at<cv::Vec3b>(start_y+i, start_x+j)[channel] = Icon.at<cv::Vec3b>(i, j)[channel];
                }
            }
        }
    }
    mIm = dst;
}
int FrameDrawer::GetObjectMidIndex(int obj_index)
{
    unique_lock<mutex> lock(mMutex);
    vector<bool>  vbMap;
    vbMap = mvbMap;
    
    float mid_point_error,min_error;
    int min_error_index = -1;
    min_error = 10000;
    for(int i=0;i<N;i++) 
    {
        if(vbMap[i])
        {
            MapPoint* pMP = All_MapPoint[i];
            cv::Mat MapPoint_Pose =  pMP->GetWorldPos();
            float Pose_x = MapPoint_Pose.at<float>(0);
            float Pose_y = MapPoint_Pose.at<float>(1);
            float Pose_z = MapPoint_Pose.at<float>(2);

            mid_point_error = sqrt(pow(Object_Pose[obj_index].x_mid-Pose_x, 2) + pow(Object_Pose[obj_index].y_mid-Pose_y, 2) + pow(Object_Pose[obj_index].z_mid-Pose_z, 2));
            if(min_error > mid_point_error)
            {
                min_error = mid_point_error;
                min_error_index = i;
            }
        }
    }
    return min_error_index;
}
void FrameDrawer::Update(Tracking *pTracker)
{
    unique_lock<mutex> lock(mMutex);
    pTracker->mImGray.copyTo(mIm);

    mvCurrentKeys=pTracker->mCurrentFrame.mvKeys;
    N = mvCurrentKeys.size();
    bmouse_click =  pTracker->bmouse_click;
    WorldPos_mouseclick = pTracker->WorldPos_mouseclick;
    
    mvbVO = vector<bool>(N,false);
    mvbMap = vector<bool>(N,false);
    mbOnlyTracking = pTracker->mbOnlyTracking;
    All_MapPoint = pTracker->mCurrentFrame.mvpMapPoints;

    if(pTracker->mLastProcessedState==Tracking::NOT_INITIALIZED)
    {
        mvIniKeys=pTracker->mInitialFrame.mvKeys;
        mvIniMatches=pTracker->mvIniMatches;
    }
    else if(pTracker->mLastProcessedState==Tracking::OK)
    {
        for(int i=0;i<N;i++)
        {
            MapPoint* pMP = pTracker->mCurrentFrame.mvpMapPoints[i];
            if(pMP)
            {
                if(!pTracker->mCurrentFrame.mvbOutlier[i])
                {
                    if(pMP->Observations()>0)
                        mvbMap[i]=true;
                    else
                        mvbVO[i]=true;
                }
            }
        }
    }
    mState=static_cast<int>(pTracker->mLastProcessedState);
}

void FrameDrawer::Init_Object_Data()
{
    Object_data single_object;
    

    single_object.name = "chair";
    single_object.x_mid = -0.41841449999999997;   
    single_object.y_mid = 0.289473;
    single_object.z_mid = 0.0779965;
    single_object.x_range = 0.12300349999999999;
    single_object.y_range = 0.146677;
    single_object.z_range = 0.1133625;
    Object_Pose.push_back(single_object);

    single_object.name = "phone";
    single_object.x_mid = 0.30693400000000004;   
    single_object.y_mid = 0.22725800000000002;
    single_object.z_mid = -0.06361825;
    single_object.x_range = 0.032477000000000006;
    single_object.y_range = 0.017233000000000012;
    single_object.z_range = 0.02981035;
    Object_Pose.push_back(single_object);
}

} //namespace ORB_SLAM
