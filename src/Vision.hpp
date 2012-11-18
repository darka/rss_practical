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
        ~BoxDetectionResult();
        bool detected;
        bool too_far;
        bool centered;
};


class Vision{

public:
    Vision();
    ~Vision();

    inline static bool inRange( unsigned char red,
                                unsigned char green,
                                unsigned char blue,
                                int           range );

    inline static bool inRangeHsv( unsigned char hue,
                                   unsigned char sat,
                                   unsigned char val,
                                   int           range );


    void static segment_floor( IplImage* src,
                               IplImage* dst,
                               int*      odv );

    void static segment_base( IplImage*  src,
                              IplImage*  dst );

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

    BoxDetectionResult detectBoxes( IplImage* frame,
                                    IplImage* frameHD,
                                    int*      boxVec );

    void static initSift();

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







private:

    CvCapture* capture;
    BASE_TYPE  baseType;
    IplImage*  orig = NULL;
    IplImage*  orig_small = NULL;

    const int CAMERA_WIDTH = 140;
    const int CAMERA_HEIGHT = 80;
    const int REAL_WIDTH = 432;
    const int REAL_HEIGHT = 240;

    int  saved_angle = -1;
    bool correct_box = false;
    int  contourMinSize = 150;
    int  contourMaxSize = 700;
    int  baseCenterX = 0;
    int  baseCenterY = 0;
    int  lowThreshold = 68;
    int  highThreshold = 110;
    int  minRatio = 500;
    int  maxRatio = 1500;
    int  a = 18;
    int  b = 17;
    int  c = 28;
    int  d = 11;
    int  e = 4;
    int  PolygonBase = 15;

    std::vector< std::vector<cv::KeyPoint>* >   sift_keypoints;
    std::vector< int >                          keypoint_match_count;
    std::vector< cv::Mat >                      sift_descriptors;
    cv::SiftFeatureDetector                     detector;
    cv::SiftDescriptorExtractor                 extractor;
    std::vector< std::string>                   image_names;
    std::vector< std::string>                   base_image_names;
    std::vector< std::vector<cv::KeyPoint>* >   base_sift_keypoints;
    std::vector< cv::Mat >                      base_sift_descriptors;

};

#endif // VISION_HPP
