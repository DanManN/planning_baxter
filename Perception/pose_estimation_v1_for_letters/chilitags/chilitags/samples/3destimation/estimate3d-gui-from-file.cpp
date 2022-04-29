/*******************************************************************************
*   Copyright 2013-2014 EPFL                                                   *
*   Copyright 2013-2014 Quentin Bonnard                                        *
*                                                                              *
*   This file is part of chilitags.                                            *
*                                                                              *
*   Chilitags is free software: you can redistribute it and/or modify          *
*   it under the terms of the Lesser GNU General Public License as             *
*   published by the Free Software Foundation, either version 3 of the         *
*   License, or (at your option) any later version.                            *
*                                                                              *
*   Chilitags is distributed in the hope that it will be useful,               *
*   but WITHOUT ANY WARRANTY; without even the implied warranty of             *
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the              *
*   GNU Lesser General Public License for more details.                        *
*                                                                              *
*   You should have received a copy of the GNU Lesser General Public License   *
*   along with Chilitags.  If not, see <http://www.gnu.org/licenses/>.         *
*******************************************************************************/

#include <iostream>

#include <chilitags.hpp>
#include <typeinfo>
#include <opencv2/core/core.hpp> // for cv::Mat
#include <opencv2/core/core_c.h> // CV_AA
#include <opencv2/highgui/highgui.hpp> // for camera capture
#include <opencv2/imgproc/imgproc.hpp> // for camera capture

using namespace std;
using namespace cv;

int main(int argc, char* argv[])
{
    cout
        << "Usage: "<< argv[0]
        << " [-c <tag configuration (YAML)>] [-i <camera calibration (YAML)>]\n";

    cv::Mat inputImage = cv::imread(argv[1]);

    // cv::imshow("temp", inputImage);
    // cv::waitKey();

    const char* intrinsicsFilename = 0;
    const char* configFilename = 0;

    for( int i = 2; i < argc; i++ )
    {
        if( strcmp(argv[i], "-c") == 0 )
            configFilename = argv[++i];
        else if( strcmp(argv[i], "-i") == 0 )
            intrinsicsFilename = argv[++i];
    }

    /*****************************/
    /*    Init camera capture    */
    /*****************************/
    // int cameraIndex = 0;
    // cv::VideoCapture capture(cameraIndex);
    // if (!capture.isOpened())
    // {
    //     cerr << "Unable to initialise video capture.\n";
    //     return 1;
    // }

    /******************************/
    /* Setting up pose estimation */
    /******************************/
#ifdef OPENCV3
    float inputWidth  = inputImage.cols;
    float inputHeight = inputImage.rows;
#else
    float inputWidth  = inputImage.cols;
    float inputHeight = inputImage.rows;
#endif

    cout<<inputWidth<<endl;
    cout<<inputHeight<<endl;

    chilitags::Chilitags3D chilitags3D(Size(inputWidth, inputHeight));

    if (configFilename) chilitags3D.readTagConfiguration(configFilename);

    if (intrinsicsFilename) {
        Size calibratedImageSize = chilitags3D.readCalibration(intrinsicsFilename);
// #ifdef OPENCV3
//         capture.set(cv::CAP_PROP_FRAME_WIDTH, calibratedImageSize.width);
//         capture.set(cv::CAP_PROP_FRAME_HEIGHT, calibratedImageSize.height);
// #else
//         capture.set(CV_CAP_PROP_FRAME_WIDTH, calibratedImageSize.width);
//         capture.set(CV_CAP_PROP_FRAME_HEIGHT, calibratedImageSize.height);
// #endif
    }

    cv::Mat projectionMat = cv::Mat::zeros(4,4,CV_32F);
    chilitags3D.getCameraMatrix().copyTo(projectionMat(cv::Rect(0,0,3,3)));
    cv::Matx44f projection = projectionMat;
    projection(3,2) = 1;

    /*****************************/
    /*             Go!           */
    /*****************************/
    cv::namedWindow("Pose Estimation");

    
    cv::Mat outputImage = inputImage.clone();

    for (auto& kv : chilitags3D.estimate(inputImage)) {

        static const float DEFAULT_SIZE = 20.f;
        static const cv::Vec4f UNITS[4] {
            {0.f, 0.f, 0.f, 1.f},
            {DEFAULT_SIZE, 0.f, 0.f, 1.f},
            {0.f, DEFAULT_SIZE, 0.f, 1.f},
            {0.f, 0.f, DEFAULT_SIZE, 1.f},
        };

        cv::Matx44f transformation = kv.second;
        cv::Vec4f referential[4] = {
            projection*transformation*UNITS[0],
            projection*transformation*UNITS[1],
            projection*transformation*UNITS[2],
            projection*transformation*UNITS[3],
        };

        std::vector<cv::Point2f> t2DPoints;
        for (auto homogenousPoint : referential)
            t2DPoints.push_back(cv::Point2f(
                                    homogenousPoint[0]/homogenousPoint[3],
                                    homogenousPoint[1]/homogenousPoint[3]));

        static const int SHIFT = 16;
        static const float PRECISION = 1<<SHIFT;
        static const std::string AXIS_NAMES[3] = { "x", "y", "z" };
        static const cv::Scalar AXIS_COLORS[3] = {
            {0,0,255},{0,255,0},{255,0,0},
        };
        for (int i : {1,2,3}) {
            cv::line(
                outputImage,
                PRECISION*t2DPoints[0],
                PRECISION*t2DPoints[i],
                AXIS_COLORS[i-1],
#ifdef OPENCV3
                1, cv::LINE_AA, SHIFT);
#else
                1, CV_AA, SHIFT);
#endif
            cv::putText(outputImage, AXIS_NAMES[i-1], t2DPoints[i],
                        cv::FONT_HERSHEY_SIMPLEX, 0.5f, AXIS_COLORS[i-1]);
        }

        cv::putText(outputImage, kv.first, t2DPoints[0],
                    cv::FONT_HERSHEY_SIMPLEX, 0.5f, cv::Scalar(255,255,255));
        int l = kv.first.length();
        cout<<kv.first.substr(4, l)<< " " << t2DPoints[0].x << " " << t2DPoints[0].y 
        << " " << t2DPoints[1].x << " " << t2DPoints[1].y << " " << t2DPoints[2].x 
        << " " << t2DPoints[2].y << " " << t2DPoints[3].x << " " << t2DPoints[3].y  <<endl;
        // cout<<t2DPoints<<endl;
    }


    // cv::imshow("Pose Estimation", outputImage);
    // cv::waitKey();
    
    // cv::imwrite("output.png", outputImage);

    cv::destroyWindow("Pose Estimation");

    return 0;
}

