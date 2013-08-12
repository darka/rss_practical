#include "Vision.hpp"
#include <algorithm>
#include <stdio.h>
#include <unistd.h>

using namespace cv;

CvCapture* Vision::capture = NULL;
BASE_TYPE Vision::baseType = BASE_NONE;
IplImage* Vision::orig = NULL;
IplImage* Vision::orig_small = NULL;
IplImage* Vision::detected_floor = NULL;
bool Vision::origReady = false;
bool Vision::runBaseDetection = false;
bool Vision::releaseBox = false;
int Vision::baseCenterX = 0;
int Vision::baseCenterY = 0;
char Vision::base = 'q'; // hardcoded base
IplImage* Vision::queen_base = cvLoadImage("base1_.png");
IplImage* Vision::purple_base = cvLoadImage("base2_.png");

CvHistogram* Vision::hist_hue_ground = NULL;
CvHistogram* Vision::hist_sat_ground = NULL;
CvHistogram* Vision::hist_val_ground = NULL;

const char* Vision::hist_window_1 = "hist1";
const char* Vision::hist_window_2 = "hist2";
const char* Vision::hist_window_3 = "hist3";


Vision::Vision()
: saved_angle(-1)
, correct_box(false)
{

    initSift();
    calcHistGround();

    // Create thread object
    pthread_t thread1;
    pthread_t thread2;

    // Create data
    int id1 = 1;
    int id2 = 2;

    // Start thread


    if (windowsEnabled)
    {
        cvNamedWindow( "mywindow3", CV_WINDOW_NORMAL );
        cvNamedWindow( "mywindow6", CV_WINDOW_NORMAL );
        cvNamedWindow( "mywindow7", CV_WINDOW_NORMAL );
        cvNamedWindow( "mywindow8", CV_WINDOW_AUTOSIZE );
        cvNamedWindow( "mywindow9", CV_WINDOW_AUTOSIZE );
        cvNamedWindow( "mywindow10", CV_WINDOW_NORMAL );                

        cvNamedWindow("Histogram", CV_WINDOW_NORMAL);
        cvNamedWindow("Histogram2", CV_WINDOW_NORMAL);

        cvNamedWindow(hist_window_1, CV_WINDOW_NORMAL);
        cvNamedWindow(hist_window_2, CV_WINDOW_NORMAL);
        cvNamedWindow(hist_window_3, CV_WINDOW_NORMAL);

        cvMoveWindow("mywindow", 0, 20);
        cvMoveWindow("mywindow3", 800, 20);
        cvMoveWindow("mywindow5", 800, 320);
        cvMoveWindow("mywindow6", 0, 620);
        cvMoveWindow("mywindow8", 800, 620);
        cvMoveWindow("mywindow9", 600, 620);
        cvMoveWindow("mywindow10", 600, 620);
        namedWindow("calcHist Demo", CV_WINDOW_AUTOSIZE );
    }
    pthread_create(&thread1, NULL, cameraThread, (void*)&id1);
    pthread_create(&thread2, NULL, baseThread, (void*)&id2);

    while (!origReady) 
    { 
        std::cout << "waiting for camera thread...\n"; 
    }

}

Vision::~Vision()
{
    cvReleaseCapture( &capture );
    if (windowsEnabled) 
    {
        cvDestroyWindow( "Histogram" );
        cvDestroyWindow( "Histogram2" );
        cvDestroyWindow( "mywindow" );
        cvDestroyWindow( "mywindow2" );
        cvDestroyWindow( "mywindow3" );
        cvDestroyWindow( "mywindow4" );
        cvDestroyWindow( "mywindow5" );
        cvDestroyWindow( "mywindow6" );
        cvDestroyWindow( "mywindow8" );
        cvDestroyWindow( "mywindow9" );
        cvDestroyWindow( "mywindow10" );
    }
}


int Vision::calcHistGround()
{
    /* Always check if the program can find a file */


    IplImage* img = cvLoadImage("ground.png");
    if( !img )
        return -1;
    IplImage* img_hsv = cvCreateImage( cvGetSize(img), img->depth, img->nChannels );
    IplImage* channel = cvCreateImage( cvGetSize(img), 8, 1 );
    cvCvtColor(img,img_hsv,CV_BGR2HSV);
    IplImage *hist_img = cvCreateImage(cvSize(CAMERA_WIDTH,CAMERA_HEIGHT), 8, 3);
    cvSet( hist_img, cvScalarAll(255), 0 );

    int hist_size = 256;      
    float range[]={0,256};
    float* ranges[] = { range };

    float max_value = 0.0;
    float max = 0.0;
    float w_scale = 0.0;


    /* Create a 1-D Arrays to hold the histograms */
    hist_hue_ground = cvCreateHist(1, &hist_size, CV_HIST_ARRAY, ranges, 1);
    hist_sat_ground = cvCreateHist(1, &hist_size, CV_HIST_ARRAY, ranges, 1);
    hist_val_ground = cvCreateHist(1, &hist_size, CV_HIST_ARRAY, ranges, 1);
    /* Set image to obtain RED as Channel of Interest (COI) */
    cvSetImageCOI(img,1);
    cvCopy(img,channel);
    cvResetImageROI(img);
    /* Calculate histogram of the Image and store it in the array */
    cvCalcHist( &channel, hist_hue_ground, 0, NULL );
    cvNormalizeHist(hist_hue_ground, 1.0);
    /* Calculate and Plot the histograms Green and Blue channels as well */
    /* Green channel */
    cvSetImageCOI(img,2);
    cvCopy(img,channel);
    cvResetImageROI(img);
    cvCalcHist( &channel, hist_sat_ground, 0, NULL );
    cvNormalizeHist(hist_sat_ground, 1.0);

    /* Blue channel */
    cvSetImageCOI(img,3);
    cvCopy(img,channel);
    cvResetImageROI(img);
    cvCalcHist( &channel, hist_val_ground, 0, NULL );
    cvNormalizeHist(hist_val_ground, 1.0);

    cvReleaseImage( &img_hsv );        
    cvReleaseImage( &img );    
    cvReleaseImage( &hist_img );
    cvReleaseImage( &channel );
    return 0;
}

double Vision::matchBase(cv::Mat const& src_test)
{
    Mat hsv_base;
    Mat hsv_test;
    Mat hsv_test_down;

    IplImage* src_base;    
    if (base == 'p')
        src_base = purple_base;
    else
        src_base = queen_base;

    /// Convert to HSV
    cvtColor( src_base, hsv_base, CV_BGR2HSV );
    cvtColor( src_test, hsv_test, CV_BGR2HSV );

    // Using 30 bins for hue and 32 for saturation
    int h_bins = 50; int s_bins = 60;
    int histSize[] = { h_bins, s_bins };

    // hue varies from 0 to 256, saturation from 0 to 180
    float h_ranges[] = { 0, 256 };
    float s_ranges[] = { 0, 180 };

    const float* ranges[] = { h_ranges, s_ranges };

    // Use the o-th and 1-st channels
    int channels[] = { 0, 1 };

    // Histograms
    MatND hist_base;
    MatND hist_test;

    // Calculate the histograms for the HSV images
    calcHist( &hsv_base, 1, channels, Mat(), hist_base, 2, histSize, ranges, true, false );
    normalize( hist_base, hist_base, 0, 1, NORM_MINMAX, -1, Mat() );

    calcHist( &hsv_test, 1, channels, Mat(), hist_test, 2, histSize, ranges, true, false );
    normalize( hist_test, hist_test, 0, 1, NORM_MINMAX, -1, Mat() );

    for( int i = 0; i < 4; i++ )
    { 
        int compare_method = i;
        double base_base = compareHist( hist_base, hist_base, compare_method );
        double base_test = compareHist( hist_test, hist_base, compare_method );

        printf( " Method [%d]  : %f %f \n", i, base_base, base_test);
    }

    return compareHist( hist_test, hist_base, 0 );

    // Apply the histogram comparison methods

    int hist_w = 512; int hist_h = 400;
    int bin_w = cvRound( (double) hist_w/h_bins );
    Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );
    for( int i = 1; i < h_bins; i++ )
    {
        line( histImage, Point( bin_w*(i-1), hist_h - cvRound(hist_base.at<float>(i-1)) ) ,
                Point( bin_w*(i), hist_h - cvRound(hist_base.at<float>(i)) ),
                Scalar( 255, 0, 0), 2, 8, 0  );
    }

    imshow("calcHist Demo", histImage );
    printf( "Done \n" );
}

void Vision::update()
{
    IplImage* prepared = cvCreateImage(cvSize(orig_small->width,orig_small->height),IPL_DEPTH_8U, orig_small->nChannels);
    cvSmooth(orig_small, prepared, CV_GAUSSIAN, 3, 3);
    cvDilate(prepared, prepared, NULL, 5);

    if ( !prepared ) {
        std::cerr << "ERROR: prepared is NULL \n";
        return;
    }

    for (size_t i = 0; i < CAMERA_WIDTH; ++i)
    {
        odv[i] = 0;
    }

    detected_floor = cvCreateImage(cvSize(prepared->width, prepared->height),IPL_DEPTH_8U, prepared->nChannels); 

    // Segment floor from image and write obstacle distances to odv vector
    segment_floor(prepared, detected_floor, odv);
    cvReleaseImage(&prepared);
}

void Vision::cleanupAfterUpdate()
{
    cvReleaseImage(&detected_floor);
}

bool Vision::inRange(unsigned char red,
        unsigned char green,
        unsigned char blue,
        int           range)
{
    // CALCULATES IF RGB VALUE OF PIXEL IS IN SPECIFIED RANGE
    //
    // INPUT:   RGB-VALUE OF PIXEL AND RANGEhist_test
    // OUTPUT:  TRUE OR FALSE

    const char diffRange = 14;
    const char blackness = 70;
    if (red <= blackness || green <= blackness || blue <= blackness)
        return false;
    if ( max(max((max(red, blue) - min(red, blue)),
                    (max(green, blue) - min(green, blue))),
                (max(green, red) - min(green, red))) >= diffRange )
        return false;
    return true;
}


inline void Vision::interpolatedCoordinates( int &min_x,
        int &min_y,
        int &max_x,
        int &max_y,
        int width,
        int height )
{
    // CONVERTS SMALL IMAGE COORDINATES TO LARGE IMAGE COORDINATES

    min_x = ((float)min_x) / width * REAL_WIDTH;
    max_x = ((float)max_x) / width * REAL_WIDTH;
    min_y = ((float)min_y) / height * REAL_HEIGHT;
    max_y = ((float)max_y) / height * REAL_HEIGHT;
}


inline bool Vision::boxIsCentered( int image_center_x,
        int image_center_y,
        int box_center_x,
        int box_center_y )
{
    // DETECT IF BOX IS CENTERED, IN ORDER TO DETERMINE IF BOX CAN BE
    // APPROACHED OR IF ROBOT NEEDS TO CHANGE DIRECTION TO FACE BOX CENTERED

    double center_error;

    if (REAL_HEIGHT - box_center_y <= 2)
    {
        center_error = 2;
    }
    else
    {
        center_error  = REAL_HEIGHT - box_center_y;
    }

    center_error = center_error / 40.0;
    center_error = REAL_HEIGHT / center_error;
    return ((image_center_x - center_error) <= box_center_x && box_center_x <= (image_center_x + center_error));
}





void Vision::segment_floor(IplImage* src, IplImage* dst, int* odv)
{
    const int floor_range = 13;

    for (int i = 0; i < src->height; i++)
    {
        for (int k = 0; k < src->width; k += 1)
        {
            int j = k * dst->nChannels;
            unsigned char red = src->imageData[i * src->widthStep + j + 2];
            unsigned char green = src->imageData[i * src->widthStep + j + 1];
            unsigned char blue = src->imageData[i * src->widthStep + j];

            if (!inRange(red, green, blue, floor_range))
            {

                const unsigned char value = 0;

                ((uchar *)(dst->imageData + i * dst->widthStep))[j] = blue;
                ((uchar *)(dst->imageData + i * dst->widthStep))[j+1] = green;
                ((uchar *)(dst->imageData + i * dst->widthStep))[j+2] = red;
            }
            else
            {
                const unsigned char value = 255;

                ((uchar *)(dst->imageData + i * dst->widthStep))[j] = value;
                ((uchar *)(dst->imageData + i * dst->widthStep))[j+1] = value;
                ((uchar *)(dst->imageData + i * dst->widthStep))[j+2] = value;
            }
        }
    }

    std::memset(odv, 0, sizeof(int));


    for (int i = 0; i < dst->height; i++)
    {
        for (int k = 0; k < dst->width; k += 1)
        {
            int j = k * dst->nChannels;
            unsigned char red = dst->imageData[i * dst->widthStep + j + 2];
            unsigned char green = dst->imageData[i * dst->widthStep + j + 1];
            unsigned char blue = dst->imageData[i * dst->widthStep + j];
            if (red == 255 && green == 255 && blue == 255)
            {

            }
            else
            {
                if (odv[k] < i)
                {
                    odv[k] = i;
                }
            }

        }
    }

    for (size_t i = 0; i < CAMERA_WIDTH; ++i)
    {
        odv[i] = dst->height - odv[i];
    }

    if (windowsEnabled) cvShowImage( "mywindow3", dst );
}


BoxDetectionResult Vision::detectBoxes()
{

    IplImage* frame = orig_small;
    IplImage* frameHD = orig;
    const int BoxROIError = 65;

    BoxDetectionResult ret;
    IplImage* gray2;
    gray2 = cvCreateImage(cvSize(frame->width, frame->height), IPL_DEPTH_8U, 1);
    cvCvtColor(frame,gray2,CV_BGR2GRAY);

    IplImage* gray2Dilated = cvCreateImage(cvSize(frame->width, frame->height), IPL_DEPTH_8U, 1);
    cvCopy(gray2, gray2Dilated);

    CvScalar avg;
    CvScalar avgStd;
    cvAvgSdv(gray2, &avg, &avgStd, NULL);

    cvThreshold(gray2, gray2, (int)avg.val[0] - a* (int)(avgStd.val[0]/b), 255, CV_THRESH_BINARY_INV);

    cvThreshold(gray2Dilated, gray2Dilated, (int)avg.val[0] - d* (int)(avgStd.val[0]/e), 255, CV_THRESH_BINARY_INV);
    cvDilate(gray2Dilated, gray2Dilated, NULL, 2);

    cv::Mat gray_CvMat(gray2, false);

    cv::Mat grayDilated_CvMat(gray2Dilated, false);

    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;



    vector<vector<Point> > contoursDilated;
    vector<Vec4i> hierarchyDilated;

    findContours(gray_CvMat, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    findContours(grayDilated_CvMat, contoursDilated, hierarchyDilated, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    for (vector<vector<Point> >::iterator i = contoursDilated.begin(); i != contoursDilated.end(); ++i)
    {
        contours.push_back(*i);
    }
    for (vector<Vec4i>::iterator i = hierarchyDilated.begin(); i != hierarchyDilated.end(); ++i)
    {
        hierarchy.push_back(*i);
    }


    Mat drawing = Mat::zeros( gray_CvMat.size(), CV_8UC1 );
    vector<vector<Point> > squares;
    vector<Point> approx;

    for (size_t i = 0; i < contours.size(); ++i)
    {
        approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*(1.0/c), true);
        squares.push_back(approx);
    }


    int idx = 0;

    std::vector< std::pair<int, int> > centers;
    for(size_t idx = 0; idx < squares.size(); ++idx)
    {
        bool notRemoved = true;

        Mat m(squares[idx]);
        double area = contourArea(m);
        cv::RotatedRect box = cv::minAreaRect(m);

        cv::Point2f rBPts[4];
        box.points(rBPts);
        int min_x = rBPts[0].x;
        int max_x = rBPts[0].x;
        int min_y = rBPts[0].y;
        int max_y = rBPts[0].y;

        for(int i = 0; i < 3 ; i++){

            if(rBPts[i+1].y > max_y){
                max_y = std::min((int)rBPts[i+1].y, frame->height);
            }

            if(rBPts[i+1].y < min_y){
                min_y = std::max(0, (int)rBPts[i+1].y);
            }
            if(rBPts[i+1].x > max_x){
                max_x = std::min((int)rBPts[i+1].x, frame->width);
            }
            if(rBPts[i+1].x < min_x){
                min_x = std::max((int)rBPts[i+1].x, 0);
            }

        }

        int width = max_x - min_x;
        int height = max_y - min_y;
        float ratio = (float)width / height;
        float boundingBoxArea = width * height;

        // TODO: fix problems with detecting multiple boxes
        if(contourMaxSize > area && area > contourMinSize && (maxRatio/1000.f) > ratio && ratio > (minRatio/1000.f) && boundingBoxArea < 2*area)
        {
            drawContours(drawing, squares, idx, Scalar(255, 0, 0), CV_FILLED, 8, hierarchy);

            int min_x_ = min_x;
            int min_y_ = min_y;
            int max_x_ = max_x;
            int max_y_ = max_y;

            int center_x = ((min_x_ + max_x_) / 2);
            int center_y = ((min_y_ + max_y_) / 2);

            if (boxModel.area < area)
            {
                boxModel.area = area;
                boxModel.width = width;
                boxModel.position = std::make_pair(center_x, center_y);
            }

            const int centerThreshold = 13;
            for (std::vector< std::pair<int, int> >::iterator i = centers.begin();
                    i != centers.end(); ++i)
            {
                if( (std::fabs(center_x - i->first) <= centerThreshold) &&
                        (std::fabs(center_y - i->second) <= centerThreshold))
                {
                    notRemoved = false;
                    continue;
                }
            }

            if(!notRemoved) continue;

            centers.push_back(std::make_pair(center_x, center_y));

            interpolatedCoordinates(min_x, min_y, max_x, max_y, drawing.cols, drawing.rows);

            ret.detected = true;
            std::cout << "**** box detected with area: " << area << '\n';
            ret.too_far = (area < 250);
            if (ret.too_far)
            {
                std::cout << "**** Box is too far to approach.\n";
            }

            int box_center_x = ((min_x + max_x) / 2);
            int box_center_y = ((min_y + max_y) / 2);
            int image_center_x = REAL_WIDTH / 2;
            int image_center_y = REAL_HEIGHT / 2;

            std::cout << "box at: " << box_center_x << ", center at: " << (REAL_WIDTH / 2) << '\n';
            ret.centered = boxIsCentered(image_center_x, image_center_y, box_center_x, box_center_y);

            if (!ret.centered)
            {
                std::cout << "**** Box is not centered.\n";
            }
            if (ret.centered && !ret.too_far)
            {
                std::cout << "**** Box can be approached.\n";
                correct_box = detectFeatures(min_x, min_y, max_x, max_y, frameHD);
                if (correct_box)
                    std::cout << "**** approaching\n";
                else
                    std::cout << "**** but not enough keypoints\n";
            }

        }
    }


    Mat& drawingScaled = drawing;
    imshow("mywindow9", drawing);

    std::memset(boxVec, 0, sizeof(int)*drawingScaled.cols);

    for (int i = 0; i < drawingScaled.rows; i++)
    {
        for (int k = 0; k < drawingScaled.cols; k += 1)
        {
            unsigned char pixel = drawingScaled.at<char>(i, k);
            if (pixel == 255)
            {
                if (boxVec[k] < i)
                {
                    boxVec[k] = i;
                }

            }

        }

    }

    cvReleaseImage(&gray2);
    return ret;
}



void Vision::initSift()
{

    image_names.push_back("walle.png");
    keypoint_match_count.push_back(20);
    image_names.push_back("ferrari.png");
    keypoint_match_count.push_back(7);
    image_names.push_back("celebes.png");
    keypoint_match_count.push_back(11);
    image_names.push_back("fry.png");
    keypoint_match_count.push_back(5);
    image_names.push_back("mario.png");
    keypoint_match_count.push_back(11);
    image_names.push_back("terminator.png");
    keypoint_match_count.push_back(11);
    image_names.push_back("iron.png");
    keypoint_match_count.push_back(11);
    image_names.push_back("starry.png");
    keypoint_match_count.push_back(11);

    base_image_names.push_back("base1.png");
    base_image_names.push_back("base2.png");

    for (size_t i = 0; i < image_names.size(); ++i)
    {
        std::cout << "Generating descriptors for: " << image_names[i] << '\n';
        cv::Mat input = cv::imread(image_names[i], 0);
        std::vector<cv::KeyPoint>* keypoints_image = new std::vector<cv::KeyPoint>();
        detector.detect(input, *keypoints_image);
        cv::Mat descriptors_image;
        extractor.compute(input, *keypoints_image, descriptors_image);

        sift_keypoints.push_back(keypoints_image);
        sift_descriptors.push_back(descriptors_image);
        std::cout << "Keypoints: " << sift_keypoints[i]->size() << '\n';
    }
}

bool Vision::detectFeatures(int min_x, int min_y, int max_x, int max_y, IplImage* frameHD)
{
    bool ret = false;

    std::vector<cv::KeyPoint> keypoints_camera;


    cv::Mat cameraMat(frameHD);

    // Setup a rectangle to define your region of interest
    int width = max_x - min_x;
    int height = max_y - min_y;

    if(min_x >= REAL_WIDTH)
    {
        min_x = REAL_WIDTH;
    }

    if(max_x >= REAL_WIDTH)
    {
        max_x = REAL_WIDTH;
    }

    if(min_y >= REAL_HEIGHT)
    {
        min_y = REAL_HEIGHT;
    }

    if(max_y >= REAL_HEIGHT)
    {
        max_y = REAL_HEIGHT;
    }

    if(min_y + height >= REAL_HEIGHT)
    {
        height = REAL_HEIGHT - min_y;
    }

    if(min_x + width >= REAL_WIDTH)
    {
        width = REAL_WIDTH - min_x;
    }


    cv::Rect myROI(min_x, min_y, width, height);


    // Crop the full image to that image contained by the rectangle myROI
    // Note that this doesn't copy the data
    cv::Mat croppedImage = cameraMat(myROI);
    cv::Mat croppedImageGray;
    cvtColor(croppedImage, croppedImageGray, CV_RGB2GRAY);
    detector.detect(croppedImageGray, keypoints_camera);
    cv::Mat descriptors_camera;
    extractor.compute(croppedImageGray, keypoints_camera, descriptors_camera);


    if (keypoints_camera.size() > 10)
    {
        for (size_t image_iter = 0; image_iter != sift_keypoints.size(); ++image_iter)
        {
            unsigned int count = 0;
            for (size_t i = 0; i < sift_keypoints[image_iter]->size(); ++i) {

                if(checkForMatch(i, keypoints_camera, sift_descriptors[image_iter], descriptors_camera))
                {
                    count++;
                }
            }
            std::cout << image_names[image_iter] << "   keypoints: " << count << '\n';

            if (count >= keypoint_match_count[image_iter])
            {
                ret = true;
                break;
            }
        }
    }


    keypoints_camera.clear();
    return ret;

}

inline bool Vision::checkForMatch(size_t point_i, std::vector<cv::KeyPoint>& other_points, cv::Mat& descriptors_image, cv::Mat& descriptors_camera)
{
    double dsq, distsq1 = 100000000, distsq2 = 100000000;

    // Find the two closest matches, and put their squared distances in
    // distsq1 and distsq2.
    size_t minkey;
    for (size_t i = 0; i < other_points.size(); ++i) {
        dsq = distSquared(point_i, i, descriptors_image, descriptors_camera);

        if (dsq < distsq1) {
            distsq2 = distsq1;
            distsq1 = dsq;
            minkey = i;
        }
        else if (dsq < distsq2) {
            distsq2 = dsq;
        }
    }
    // Check whether closest distance is less than 0.6 of second.
    if (10 * 10 * distsq1 <= 7 * 7 * distsq2)
    {
        return true;
    }
    else
        return false;
}


inline double Vision::distSquared(size_t point_a, size_t point_b, cv::Mat& descriptors_image, cv::Mat& descriptors_camera)
{
    int i = 0;
    double distsq = 0.0;

    for (i = 0; i < 128; i++) {
        float dif = descriptors_image.at<float>(point_a, i) - descriptors_camera.at<float>(point_b, i);
        distsq += dif * dif;
    }

    return distsq;
}

inline IplImage* Vision::small_size_camera_image(IplImage* orig)
{
    IplImage*  orig_small = cvCreateImage( cvSize(CAMERA_WIDTH, CAMERA_HEIGHT), orig->depth, orig->nChannels);
    cvResize(orig, orig_small);
    return orig_small;
}

inline IplImage* Vision::grabFrame()
{
    cvGrabFrame(capture);
    cvGrabFrame(capture);
    return cvRetrieveFrame( capture );
}

void* Vision::cameraThread(void* Param)
{
    // Retrieve data
    int id = *((int*)Param);
    // Perform some action
    // Count down from 10 using different speed depending on data
    capture = cvCaptureFromCAM( CV_CAP_ANY );

    if ( !capture ) {
        std::cerr << "ERROR: capture is NULL \n";
    }

    cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_WIDTH, REAL_WIDTH );
    cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_HEIGHT, REAL_HEIGHT );
    while (true)
    {
        orig = grabFrame();
        if (orig == NULL)
            std::cout << "Camera frame empty\n";
        else
        {        
            orig_small = small_size_camera_image(orig);
            origReady = true;
        }
        if ( (cvWaitKey(10) & 255) == 27 ) break;
    }

    pthread_exit(NULL);
}


void* Vision::baseThread(void* Param)
{
    // Retrieve data
    int id = *((int*)Param);
    // Perform some action
    // Count down from 10 using different speed depending on data
    while (orig_small == NULL) { usleep(100); }
    while (true)
    {
        if (runBaseDetection)
        {
            /// Canny detector
            Mat detected_edges;
            Mat orig_small_copy(orig_small, true);
            cvtColor( orig_small_copy, detected_edges, CV_BGR2GRAY );
            cv::Mat sharpened;
            cv::GaussianBlur(detected_edges, sharpened, cv::Size(0, 0), W1);
            cv::addWeighted(detected_edges, W2/10.0, sharpened, -W3/10.0, W4, sharpened);


            Canny( sharpened, sharpened, lowThreshold, highThreshold, 3, true );
            dilate(sharpened, sharpened, getStructuringElement( MORPH_RECT,
                        Size( 3, 3 ),
                        Point( 2, 2 ) ));
            Mat dst;
            /// Using Canny's output as a mask, we display our result
            dst = Scalar::all(0);

            Mat(orig_small).copyTo( dst, sharpened);
            vector<vector<Point> > contours;
            vector<Vec4i> hierarchy;
            findContours( sharpened, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
            vector<Point> approx;
            vector<vector<Point> > bases;

            for( int i = 0; i< contours.size(); i++ )
            {
                approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*(1.0/PolygonBase), true);
                bases.push_back(approx);
            }

            /// Draw contours
            Mat contourDrawing = Mat::zeros( detected_edges.size(), CV_8UC1 );
            for( int i = 0; i< bases.size(); i++ )
            {
                Mat m(bases[i]);
                double area = contourArea(m);
                if(area > 500 && area < 2000)
                {
                    Scalar color = Scalar( 255, 255, 255 );
                    drawContours( contourDrawing, bases, i, color, CV_FILLED, 8, hierarchy, 0, Point(0,0) );
                }
            }

            /// Show in a window

            contours.clear();
            hierarchy.clear();
            approx.clear();
            bases.clear();
            findContours( contourDrawing, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
            for( int i = 0; i< contours.size(); i++ )
            {
                approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*(1.0/PolygonBase), true);
                bases.push_back(approx);
            }
            Mat contourDrawing2 = Mat::zeros( detected_edges.size(), CV_8UC1 );


            for( int i = 0; i < bases.size(); i++ )
            {
                Scalar color = Scalar( 255, 255, 255 );
                drawContours( contourDrawing2, bases, i, color, CV_FILLED, 8, hierarchy, 0, Point(0,0) );
                baseCenterX = 0;
                baseCenterY = 0;
                for (size_t j = 0; j < bases[i].size(); j++)
                {
                    baseCenterX += bases[i][j].x;
                    baseCenterY += bases[i][j].y;
                }
                baseCenterX /= bases[i].size();
                baseCenterY /= bases[i].size();
                std::cout << "center: (" << baseCenterX << ", " << baseCenterY << ")\n";

                int c1 = 0;
                int c2 = 0;
                int c3 = 0;

                int centerSize = 10;

                for(int i = baseCenterX - centerSize / 2; i < baseCenterX + centerSize / 2; i++)
                {
                    for(int j = baseCenterY - centerSize / 2; j < baseCenterY + centerSize / 2; j++)
                    {
                        c1 += orig_small_copy.at<Vec3b>( j , i )[0];
                        c2 += orig_small_copy.at<Vec3b>( j , i )[1];
                        c3 += orig_small_copy.at<Vec3b>( j , i )[2];
                        orig_small_copy.at<Vec3b>( j , i )[0] = 255;
                        orig_small_copy.at<Vec3b>( j , i )[1] = 255;
                        orig_small_copy.at<Vec3b>( j , i )[2] = 255;
                    }
                }
                orig_small_copy.at<Vec3b>( 1 , 1 )[0] = 0;
                orig_small_copy.at<Vec3b>( 1 , 1 )[1] = 0;
                orig_small_copy.at<Vec3b>( 1 , 1 )[2] = 255;
                orig_small_copy.at<Vec3b>( 2 , 1 )[0] = 0;
                orig_small_copy.at<Vec3b>( 2 , 1 )[1] = 0;
                orig_small_copy.at<Vec3b>( 2 , 1 )[2] = 255;
                orig_small_copy.at<Vec3b>( 3 , 1 )[0] = 0;
                orig_small_copy.at<Vec3b>( 3 , 1 )[1] = 0;
                orig_small_copy.at<Vec3b>( 3 , 1 )[2] = 255;

                std::cout << c1 << "," << c2 << "," << c3 << std::endl;

                c1 /= centerSize*centerSize;
                c2 /= centerSize*centerSize;
                c3 /= centerSize*centerSize;

                std::cout << c1 << "," << c2 << "," << c3 << std::endl;



                baseType = BASE_NONE;
                if (160 <= c3 && c3 <= 195 &&
                        130 <= c2 && c2 <= 165 &&
                        100 <= c1 && c1 <= 130)
                {
                    baseType = BASE_QUEEN;
                }
                else if (120 <= c3 && c3 <= 152 &&
                        120 <= c2 && c2 <= 152 &&
                        120 <= c1 && c1 <= 152)
                {
                    baseType = BASE_PURPLE;
                }

                BASE_TYPE histType = BASE_NONE;
                if ((orig) && matchBase(orig) > 0.85)
                {
                    if (base == 'q')
                        histType = BASE_QUEEN;
                    else if (base == 'p')
                        histType = BASE_PURPLE;
                }       
                if ( baseType != BASE_NONE && 
                        ((base == 'q' && baseType == BASE_QUEEN) || 
                         (base == 'p' && baseType == BASE_PURPLE)) )
                {
                    releaseBox = true;
                    std::cout << "Detected: " << baseType << ", " << histType << '\n';
                }

            }
            if (windowsEnabled) imshow( "mywindow8", contourDrawing2 );
        }
        else
        {
            releaseBox = false;
            usleep(10);
        }

    }

    pthread_exit(NULL);
}

