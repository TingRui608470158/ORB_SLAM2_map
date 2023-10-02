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


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<opencv2/core/core.hpp>

#include<System.h>

using namespace std;
using namespace cv;

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps);

int main(int argc, char **argv)
{
    std::cout<<"cv::getBuildInformation()=" << cv::getBuildInformation() << std::endl;
    string Map_name = "map.bin";

    if(argc != 5)
    {
        cerr << endl << "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_image_folder path_to_times_file" << endl;
        return 1;
    }

//check
    //check Map if exist
    fstream test_file(Map_name);
    bool load_map = (test_file.good())? true:false;
    test_file.close();
    if(load_map)
        cout<<"Main: Map exist"<<endl;
    else
        cout<<"Main: Map not exist"<<endl;
    
    //check camera if exist
    cv::VideoCapture cap(0);
    bool is_camera = (cap.isOpened())? true:false;
    if(is_camera)
        cout<<"Main: camera exist"<<endl;
    else
        cout<<"Main: camera not exist"<<endl;

// create Slam system
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true,load_map);

// load map 
    if(load_map)
    {
        cout<<"Main: Start Load Map "<<endl;
        SLAM.LoadMap(Map_name);
    }

// start slam
    
    int frameWidth = static_cast<int>(cap.get(CV_CAP_PROP_FRAME_WIDTH));
    int frameHeight = static_cast<int>(cap.get(CV_CAP_PROP_FRAME_HEIGHT));
    double frameRate = 30.0; // 设置帧率为30帧每秒
    cout<<"frameWidth = "<<frameWidth<<endl;
    cout<<"frameHeight = "<<frameHeight<<endl;
    cout<<"frameRate = "<<frameRate<<endl;
    cv::VideoWriter videoWriter("output_video.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), cap.get(CV_CAP_PROP_FPS), cv::Size(frameWidth, frameHeight),false);
    if (!videoWriter.isOpened()) {
        std::cerr << "Error: Unable to create VideoWriter." << std::endl;
        return -1;
    }
    
    if(is_camera)   //use camera
    {
        Mat frame;
        Mat gray;
        Mat slam_tcw;
        bool bwrite_video = false;
        // Track
        while (char(waitKey(1)) != 'q') {
            bool ret = cap.read(frame); // or cap >> frame;
            if (!ret) {
                cout << "Can't receive frame (stream end?). Exiting ...\n";
                break;
            }
        
            cvtColor(frame, gray, COLOR_BGR2GRAY);
            auto t_now = std::chrono::system_clock::now();
            auto timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(t_now.time_since_epoch()).count();
            slam_tcw = SLAM.TrackMonocular(gray,timestamp);

            if(!slam_tcw.empty())
            {
                bwrite_video = true;
            }
            
            if(bwrite_video)
            {
                 printf("videoWriter\n");
                 videoWriter<<gray;
            }
        }
    }else    // use dataset
    {
        // Retrieve paths to images
        vector<string> vstrImageFilenames;
        vector<double> vTimestamps;
        LoadImages(string(argv[3]), string(argv[4]), vstrImageFilenames, vTimestamps);

        int nImages = vstrImageFilenames.size();

        if(nImages<=0)
        {
            cerr << "ERROR: Failed to load images" << endl;
            return 1;
        }

        // Create SLAM system. It initializes all system threads and gets ready to process frames.
        // ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);

        // Vector for tracking time statistics
        vector<float> vTimesTrack;
        vTimesTrack.resize(nImages);
        cout << "Start processing sequence ..." << endl;
        cout << "Images in the sequence: " << nImages << endl << endl;

        // Main loop
        cv::Mat im;
        for(int ni=0; ni<nImages; ni++)
        {
            // Read image from file
            im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
            double tframe = vTimestamps[ni];

            if(im.empty())
            {
                cerr << endl << "Failed to load image at: "
                    <<  vstrImageFilenames[ni] << endl;
                return 1;
            }

            std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

            // Pass the image to the SLAM system
            SLAM.TrackMonocular(im,tframe);

            std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

            double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

            vTimesTrack[ni]=ttrack;

            // Wait to load the next frame
            double T=0;
            if(ni<nImages-1)
                T = vTimestamps[ni+1]-tframe;
            else if(ni>0)
                T = tframe-vTimestamps[ni-1];

            if(ttrack<T)
                std::this_thread::sleep_for(std::chrono::microseconds(static_cast<size_t>((T-ttrack)*1e6)));
        }


        // Tracking time statistics
        sort(vTimesTrack.begin(),vTimesTrack.end());
        float totaltime = 0;
        for(int ni=0; ni<nImages; ni++)
        {
            totaltime+=vTimesTrack[ni];
        }
        cout << "-------" << endl << endl;
        cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
        cout << "mean tracking time: " << totaltime/nImages << endl;
    }

    // Save map
    if(!load_map)
        SLAM.SaveMap("./map.bin");

    // Stop all threads
    SLAM.Shutdown();
    cap.release();
    videoWriter.release();
    
    
    
    
    
    
    
    
    
    
    
//     // Retrieve paths to images
//     vector<string> vstrImageFilenames;
//     vector<double> vTimestamps;
//     LoadImages(string(argv[3]), string(argv[4]), vstrImageFilenames, vTimestamps);

//     int nImages = vstrImageFilenames.size();

//     if(nImages<=0)
//     {
//         cerr << "ERROR: Failed to load images" << endl;
//         return 1;
//     }

//     // Create SLAM system. It initializes all system threads and gets ready to process frames.
//     ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::MONOCULAR,true);
//     SLAM.SaveMap("./map.bin");
//     // Vector for tracking time statistics
//     vector<float> vTimesTrack;
//     vTimesTrack.resize(nImages);

//     cout << endl << "-------" << endl;
//     cout << "Start processing sequence ..." << endl;
//     cout << "Images in the sequence: " << nImages << endl << endl;

//     // Main loop
//     cv::Mat im;
//     for(int ni=0; ni<nImages; ni++)
//     {
//         // Read image from file
//         im = cv::imread(vstrImageFilenames[ni],CV_LOAD_IMAGE_UNCHANGED);
//         double tframe = vTimestamps[ni];

//         if(im.empty())
//         {
//             cerr << endl << "Failed to load image at: "
//                  <<  vstrImageFilenames[ni] << endl;
//             return 1;
//         }

// #ifdef COMPILEDWITHC11
//         std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
// #else
//         std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
// #endif

//         // Pass the image to the SLAM system
//         SLAM.TrackMonocular(im,tframe);

// #ifdef COMPILEDWITHC11
//         std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
// #else
//         std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
// #endif

//         double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

//         vTimesTrack[ni]=ttrack;

//         // Wait to load the next frame
//         double T=0;
//         if(ni<nImages-1)
//             T = vTimestamps[ni+1]-tframe;
//         else if(ni>0)
//             T = tframe-vTimestamps[ni-1];

//         if(ttrack<T)
//             usleep((T-ttrack)*1e6);
//     }

//     SLAM.SaveMap("./map.bin");

//     // Stop all threads
//     SLAM.Shutdown();

    // Tracking time statistics
    // sort(vTimesTrack.begin(),vTimesTrack.end());
    // float totaltime = 0;
    // for(int ni=0; ni<nImages; ni++)
    // {
    //     totaltime+=vTimesTrack[ni];
    // }
    // cout << "-------" << endl << endl;
    // cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    // cout << "mean tracking time: " << totaltime/nImages << endl;

    // Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");

    return 0;
}

void LoadImages(const string &strImagePath, const string &strPathTimes,
                vector<string> &vstrImages, vector<double> &vTimeStamps)
{
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    vTimeStamps.reserve(5000);
    vstrImages.reserve(5000);
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            vstrImages.push_back(strImagePath + "/" + ss.str() + ".png");
            double t;
            ss >> t;
            vTimeStamps.push_back(t/1e9);

        }
    }
}
