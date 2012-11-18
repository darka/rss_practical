#ifndef VISION_HPP
#define VISION_HPP

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <algorithm>
#include <cstring>
#include <vector>
#include <cmath>
#include "Controller.hpp"

#define PI 3.14159265

enum BASE_TYPE { BASE_QUEEN, BASE_GREY };

struct BoxDetectionResult
{
        BoxDetectionResult() : detected(false), too_far(false), centered(false) {}
        bool detected;
        bool too_far;
        bool centered;
};

struct BoxModel
{
        BoxModel() : area(0), width(0) {}
        std::pair<int, int> position;
        int area;
        int width;
};



class Vision{

public:
    Vision();
    ~Vision();
    void update();
    void cleanupAfterUpdate();

    inline static bool inRange( unsigned char red,
                                unsigned char green,
                                unsigned char blue,
                                int           range );

    void static segment_floor( IplImage* src,
                               IplImage* dst,
                               int*      odv );

    inline void static interpolatedCoordinates( int &min_x,
                                                int &min_y,
                                                int &max_x,
                                                int &max_y,
                                                int width,
                                                int height );


    inline static bool boxIsCentered( int image_center_x,
                                      int image_center_y,
                                      int box_center_x,
                                      int box_center_y );

    BoxDetectionResult detectBoxes();

    void initSift();

    bool detectFeatures( int        min_x,
                         int        min_y,
                         int        max_x,
                         int        max_y,
                         IplImage*  frameHD );

    inline static bool checkForMatch( size_t                        point_i,
                                      std::vector<cv::KeyPoint>&    other_points,
                                      cv::Mat&                      descriptors_image,
                                      cv::Mat&                      descriptors_camera );

    inline static double distSquared( size_t    point_a,
                                      size_t    point_b,
                                      cv::Mat&  descriptors_image,
                                      cv::Mat&  descriptors_camera );

    inline static IplImage* small_size_camera_image( IplImage* orig );

    inline static IplImage* grabFrame();

    static void* cameraThread( void* Param );

    static void* baseThread(void* Param);

    void static enableBaseDetection() { runBaseDetection = true; }
    bool static baseDetectionEnabled() { return runBaseDetection; }
    void static disableBaseDetection() { runBaseDetection = false; }

    bool static canReleaseBox() { return releaseBox; }

    static CvCapture* capture;
    static BASE_TYPE  baseType;
    static IplImage* orig;
    static IplImage* orig_small;
    static IplImage* detected_floor;
    static bool origReady;
    BoxModel boxModel;

    static const int CAMERA_WIDTH = 140;
    static const int CAMERA_HEIGHT = 80;
    static const int REAL_WIDTH = 432;
    static const int REAL_HEIGHT = 240;

    int  saved_angle;
    bool correct_box;
    static int  baseCenterX;
    static int  baseCenterY;
    static bool runBaseDetection;
    static bool releaseBox;
    static const int  contourMinSize = 150;
    static const int  contourMaxSize = 700;
    static const int  lowThreshold = 68;
    static const int  highThreshold = 110;
    static const int  minRatio = 500;
    static const int  maxRatio = 1500;
    static const int  a = 18;
    static const int  b = 17;
    static const int  c = 28;
    static const int  d = 11;
    static const int  e = 4;
    static const int  PolygonBase = 15;
    static const int  W1 = 2;
    static const int  W2 = 10;
    static const int  W3 = 0;
    static const int  W4 = 9;
    static const int  W5 = 2;

    std::vector< std::vector<cv::KeyPoint>* >   sift_keypoints;
    std::vector< int >                          keypoint_match_count;
    std::vector< cv::Mat >                      sift_descriptors;
    cv::SiftFeatureDetector                     detector;
    cv::SiftDescriptorExtractor                 extractor;
    std::vector< std::string>                   image_names;
    std::vector< std::string>                   base_image_names;
    std::vector< std::vector<cv::KeyPoint>* >   base_sift_keypoints;
    std::vector< cv::Mat >                      base_sift_descriptors;

    int odv[Vision::CAMERA_WIDTH];
    int boxVec[Vision::CAMERA_WIDTH];

    static const bool windowsEnabled = true;
};

#endif // VISION_HPP
