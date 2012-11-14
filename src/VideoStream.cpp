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


using namespace cv;
RNG rng(12345);
enum BASE_TYPE { BASE_QUEEN, BASE_GREY };
/*
const int CAMERA_WIDTH = 181;
const int CAMERA_HEIGHT = 96;
const int REAL_WIDTH = 544;
const int REAL_HEIGHT = 288;
*/
const int CAMERA_WIDTH = 140;
const int CAMERA_HEIGHT = 80;
const int REAL_WIDTH = 432;
const int REAL_HEIGHT = 240;
bool movementEnabled = true;
bool windowsEnabled = true;
bool canReleaseBox = false; 
BASE_TYPE baseType;
/*
int a = 4;
int b = 5;
int c = 15;
*/
int saved_angle = -1;
bool correct_box = false;
int contourMinSize = 150;
int contourMaxSize = 700;
int baseCenterX = 0;
int baseCenterY = 0;
int lowThreshold = 68;
int highThreshold = 110;
int minRatio = 500;
int maxRatio = 1500;
int a = 18;
int b = 17;
int c = 28;
int d = 11;
int e = 4;
int PolygonBase = 15;

int minHue = 0;
int maxHue = 0;
int minSat = 0;
int maxSat = 0;
int minVal = 0;
int maxVal = 0;

std::vector< std::vector<cv::KeyPoint>* > sift_keypoints;
std::vector< int > keypoint_match_count;
std::vector< cv::Mat > sift_descriptors;

cv::SiftFeatureDetector detector;
cv::SiftDescriptorExtractor extractor;
std::vector<std::string> image_names;     

std::vector<std::string> base_image_names;  
std::vector< std::vector<cv::KeyPoint>* > base_sift_keypoints;
std::vector< cv::Mat > base_sift_descriptors;

bool checkForMatch(size_t point_i, std::vector<cv::KeyPoint>& other_points, cv::Mat& descriptors_image, cv::Mat& descriptors_camera);
double distSquared(size_t point_a, size_t point_b, cv::Mat& descriptors_image, cv::Mat& descriptors_camera);
bool detectFeatures(int min_x, int min_y, int max_x, int max_y, IplImage* frameHD);
        
Controller ctrl;
CvCapture* capture;
int stoppedForPicturesCounter = 0;
int sawBoxCounter = 3;
bool boxDetected;
bool hasBox = false;

struct BoxModel
{
        BoxModel() : area(0), width(0) {}
        std::pair<int, int> position;
        int width;
        int area;
};

BoxModel boxModel;

int boxModelAliveCounter = 3;

inline bool inRange(unsigned char red, unsigned char green, unsigned char blue, int range)
{
        const char diffRange = 14;
        const char blackness = 70;
        if (red <= blackness || green <= blackness || blue <= blackness)
                return false;
        if ( max(max((max(red, blue) - min(red, blue)),
                 (max(green, blue) - min(green, blue))),
                 (max(green, red) - min(green, red))) >= diffRange )
             return false;
        return true;

}        cv::Mat captureHsv;

inline bool inRangeHsv(unsigned char hue, unsigned char sat, unsigned char val, int range)
{
        const char diffRange = 14;
        const char blackness = 70;
        /*
        if (red <= blackness || green <= blackness || blue <= blackness)
                return false;
        if ( max(max((max(red, blue) - min(red, blue)),
                 (max(green, blue) - min(green, blue))),
                 (max(green, red) - min(green, red))) >= diffRange )
             return false;*/
        //std::cout << ((int)hue) << ' ' << ((int)sat) << ' ' << ((int)val) << '\n';
        if (maxHue >= hue && hue >= minHue && maxSat >= sat && sat >= minSat && maxVal >= val && val >= minVal)
                return true;     
        return false;

}

void segment_floor(IplImage* src, IplImage* dst, int* odv)
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
        
        //if (windowsEnabled) cvShowImage( "mywindow3", dst ); 	
}

void segment_base(IplImage* src, IplImage* dst/*, int* odv*/)
{
        const int floor_range = 13;
         cvDilate(src, src, NULL, 1);
        for (int i = 0; i < src->height; i++)
        {
                for (int k = 0; k < src->width; k += 1)
                {
                        int j = k * dst->nChannels;
                        unsigned char red = src->imageData[i * src->widthStep + j + 2];
                        unsigned char green = src->imageData[i * src->widthStep + j + 1];
                        unsigned char blue = src->imageData[i * src->widthStep + j];
                        
                        if (!inRangeHsv(red, green, blue, floor_range))
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
        
        //std::memset(odv, 0, sizeof(int));
/*
        
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
*/        
        //if (windowsEnabled) cvShowImage( "mywindow9", dst ); 	
}


void moveBackConsideringFreeSpace(IplImage* img, int* odv, Controller& ctrl)
{
        std::cout <<"Moving backwards...\n";
        int leftSpace = 0;
        for (size_t i = 0; i < img->width / 2; ++i)
        {
             leftSpace += odv[i];
        }
        int rightSpace = 0;
        for (size_t i = img->width / 2; i < img->width; ++i)
        {
             rightSpace += odv[i];
        }
        double leftSpaceAverage = leftSpace / (img->width / 2.0);
        double rightSpaceAverage = rightSpace / (img->width / 2.0);
        ctrl.moveBackward();
        usleep(400000);
        if (leftSpaceAverage < rightSpaceAverage)
        {
                ctrl.moveBackwardRight();
        }
        else
        {
                ctrl.moveBackwardLeft();
        }
        usleep(300000);
}


inline void interpolatedCoordinates(int &min_x, int &min_y, int &max_x, int &max_y, int width, int height)
{

    min_x = ((float)min_x) / width * REAL_WIDTH;
    max_x = ((float)max_x) / width * REAL_WIDTH;
    min_y = ((float)min_y) / height * REAL_HEIGHT;
    max_y = ((float)max_y) / height * REAL_HEIGHT;

}


int whiskerIgnoreCounter = 0;

struct BoxDetectionResult
{
        BoxDetectionResult() : detected(false), too_far(false), centered(false) {}
        bool detected;
        bool too_far;
        bool centered;
};

BoxDetectionResult detectBoxes(IplImage* frame, IplImage* frameHD, int* boxVec)
{
        const int BoxROIError = 65;
        //CvRect rect = cvRect(boxModel.position.first - boxModel.width / 2 - BoxROIError, 0, boxModel.width + BoxROIError, frame->height);
        //cvSetImageROI(frame, rect);
        
        IplImage* frameCutout;
        /*if (boxModel.area != 0)
        {
                frameCutout = cvCreateImage(cvSize(frame->width, frame->height), frame->depth, frame->nChannels);
                cvCopy(frame, frameCutout);
         
                //cvRectangle(frameCutout, cvPoint(0, 0), cvPoint(std::max(boxModel.position.first - boxModel.width / 2 - BoxROIError, 0), frame->height), cvScalar(255, 255, 255), CV_FILLED);
                //cvRectangle(frameCutout, cvPoint(std::min(boxModel.position.first + boxModel.width / 2 + BoxROIError, frame->width), 0), cvPoint(frame->width, frame->height), cvScalar(255, 255, 255), CV_FILLED);
                std::cout << "hello hello!\n";        
        }
        else
        {*/
                frameCutout = frame;
        //}
        if (windowsEnabled) cvShowImage("mywindow6", frameCutout);  
                        
        BoxDetectionResult ret;
        IplImage* gray2;
        gray2 = cvCreateImage(cvSize(frameCutout->width, frameCutout->height), IPL_DEPTH_8U, 1);
        cvCvtColor(frameCutout,gray2,CV_BGR2GRAY);
        
        IplImage* gray2Dilated = cvCreateImage(cvSize(frameCutout->width, frameCutout->height), IPL_DEPTH_8U, 1);
        cvCopy(gray2, gray2Dilated);
        //cvNot(gray2, gray2Inv);
        
        
        CvScalar avg;
        CvScalar avgStd;
        cvAvgSdv(gray2, &avg, &avgStd, NULL);
        
        cvThreshold(gray2, gray2, (int)avg.val[0] - a* (int)(avgStd.val[0]/b), 255, CV_THRESH_BINARY_INV);
        
        cvThreshold(gray2Dilated, gray2Dilated, (int)avg.val[0] - d* (int)(avgStd.val[0]/e), 255, CV_THRESH_BINARY_INV);
        cvDilate(gray2Dilated, gray2Dilated, NULL, 2);
        
        
                
        /*CvScalar avgInv;
        CvScalar avgStdInv;
        cvAvgSdv(gray2Inv, &avgInv, &avgStdInv, NULL);
        
        */
        //cvShowImage("mywindow8", gray2Dilated);
        
        cv::Mat gray_CvMat(gray2, false);
        
        /*cv::Mat holes=gray_CvMat.clone();
        cv::floodFill(holes,cv::Point2i(0,0),cv::Scalar(1));
        for(int i=0;i<gray_CvMat.rows*gray_CvMat.cols;i++)
        {
        if(holes.data[i]==0)
            gray_CvMat.data[i]=1;
        }*/
        
        cv::Mat grayDilated_CvMat(gray2Dilated, false);
        
        /*cv::Mat holesDilated=grayDilated_CvMat.clone();
        cv::floodFill(holesDilated,cv::Point2i(0,0),cv::Scalar(1));
        */
        
        // TODO: instead of OR'ing these two images, OR the contours detected on them
        /*for(int i=0;i<gray_CvMat.rows*gray_CvMat.cols;i++)
        {
        if(grayDilated_CvMat.data[i]==255)
            gray_CvMat.data[i]=255;
        }*/
       
        //imshow("mywindow4", gray_CvMat);
        //imshow("mywindow8",  grayDilated_CvMat);
        
              
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
        //if (!squares.empty())
        //{
        std::vector< std::pair<int, int> > centers;
        //std::cout << "amount of contours: " << squares.size() << '\n';
        //for( ; idx >= 0; idx = hierarchy[idx][0] )
        for(size_t idx = 0; idx < squares.size(); ++idx)
        {
                bool notRemoved = true;
                /*std::cout << "-----\n";
                for (vector<Point>::iterator i = squares[idx].begin(); i != squares[idx].end(); ++i)
                {
                        std::cout << " < " << (*i) << '\n';
                }*/
                
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
                //      std::cout << "ratio: " << ratio << '\n';
                float boundingBoxArea = width * height;
        
                
                // TODO: fix problems with detecting multiple boxes
                if(contourMaxSize > area && area > contourMinSize && (maxRatio/1000.f) > ratio && ratio > (minRatio/1000.f) && boundingBoxArea < 2*area)
                {
                        drawContours(drawing, squares, idx, Scalar(255, 0, 0), CV_FILLED, 8, hierarchy);
                        //ctrl.stop();
                        
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
        
                        
                        if (stoppedForPicturesCounter == 0)
                                stoppedForPicturesCounter = 5;
        



                        
                        //std::cout << "center: " << center_x << ", " << center_y << '\n';
                        const int centerThreshold = 13;
                        for (std::vector< std::pair<int, int> >::iterator i = centers.begin();
                                i != centers.end(); ++i)
                        {
                                if( (std::fabs(center_x - i->first) <= centerThreshold) &&
                                        (std::fabs(center_y - i->second) <= centerThreshold))
                                {
                                        //std::cout << "Removing box at " << center_x << ", " << center_y << '\n';
                                        notRemoved = false;
                                        continue;
                                }    
                        }
                        
                        if(!notRemoved) continue;
                                
                        centers.push_back(std::make_pair(center_x, center_y));
                        
                        interpolatedCoordinates(min_x, min_y, max_x, max_y, drawing.cols, drawing.rows);
                        //ctrl.stop();
                        correct_box = detectFeatures(min_x, min_y, max_x, max_y, frameHD);
                        ret.detected = true;
                        boxDetected = ret.detected;

                        
                        const int center_error = 100;
                        if (ret.detected)
                        {
                                std::cout << "**** box detected with area: " << area << '\n';
                                ret.too_far = (area < 350);
                                if (ret.too_far)
                                {
                                        std::cout << "**** Box is too far to approach.\n";
                                }
                                
                                int box_center_x = ((min_x + max_x) / 2);
                                int box_center_y = ((min_y + max_y) / 2);
                                int image_center_x = REAL_WIDTH / 2;
                                int image_center_y = REAL_HEIGHT / 2;
                                //std::cout << "**** box at " << box_center_x << ", " << box_center_y << "; image center at " << image_center_x << ", " << image_center_y << "\n";

                                ret.centered = ((image_center_x - center_error) <= box_center_x && box_center_x <= (image_center_x + center_error));
                                std::cout << "box at: " << box_center_x << ", center at: " << (REAL_WIDTH / 2) << '\n';
                                                                                
                                /*
                                ret.centered = ((image_center_x - center_error) < box_center_x && box_center_x < (image_center_x + center_error)) &&
                                                ((image_center_y - center_error) < box_center_y && box_center_y < (image_center_y + center_error));
                                */
                                if (!ret.centered)
                                {
                                        std::cout << "**** Box is not centered.\n";
                                }
                                if (ret.centered && !ret.too_far)
                                {
                                        std::cout << "**** Box can be approached.\n";
                                }
                        }
                
                }
        }
        //}
        
        Mat& drawingScaled = drawing;
        //if (windowsEnabled) imshow("mywindow2", drawingScaled);
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

inline std::pair<size_t, size_t> longestLine(int* boxVec, size_t size)
{
        size_t beginMax = 0;
        size_t distanceMax = 0;
        size_t endMax = 0;
                
        size_t begin = 0;
        size_t distance = 0;
        size_t end = 0;
        
        bool current = false;
        
        for (size_t i = 0; i < size; ++i)
        {
                if (current && i == size - 1 && boxVec[i])
                {
                        end = i;
                        if (distance > distanceMax)
                        {
                                beginMax = begin;
                                endMax = end;
                                distanceMax = distance;
                        }
                }
                if (boxVec[i])
                {
                        if (current)
                        {
                                distance++;
                                end = i;
                        }
                        else
                        {
                                distance = 1;
                                begin = i;
                                end = begin;
                                current = true;
                        }
                }

                else
                {
                        if (current)
                        {
                                current = false;
                                if (distance > distanceMax)
                                {
                                        beginMax = begin;
                                        endMax = end;
                                        distanceMax = distance;
                                }
                        }
                        
                }
        }
        std::pair<size_t, size_t> ret;
        ret.first = beginMax;
        ret.second = endMax;
        return ret;
}


inline void stopAndRotate(Controller& ctrl)
{
        std::cout << "stopping and rotating...\n";
        ctrl.stop(); 
        usleep(1000000);
        ctrl.rotateOnSpot();
        usleep(1000000);
}

inline void grabBox(Controller& ctrl)
{
        std::cout << "-- Grabbing box\n";
        ctrl.openServo();
        ctrl.turn(0); 
        usleep(3000000);
        ctrl.closeServo();
        usleep(1000000);
        ctrl.stop();
        hasBox = true;
}

inline void dropBox(Controller& ctrl, double angle)
{
        std::cout << "-- Dropping box\n";
        hasBox = false;
        ctrl.stop();
        usleep(5000000);
        ctrl.turn(angle);
        ctrl.turn(0); 
        usleep(30000 * (CAMERA_HEIGHT - baseCenterY));
        ctrl.openServo();
        ctrl.stop();
        ctrl.moveBackward();
        usleep(1000000);
        ctrl.rotateOnSpot();
        usleep(500000);
        ctrl.stop();
        usleep(5000000);
}


bool lastMoveWasTowardsBox = false;
void run(bool* moveable, int* odv, IplImage* img, Controller& ctrl, IplImage* normalCapture, IplImage* hdCapture, int* boxVec)
{
        std::vector<cv::KeyPoint> keypoints_camera;   
        
        IplImage* captureHsv = cvCreateImage( cvSize(normalCapture->width, normalCapture->height), IPL_DEPTH_8U, 3);
        cvCvtColor(normalCapture, captureHsv, CV_RGB2HSV);
        IplImage* captureHsvSeg = cvCreateImage( cvSize(normalCapture->width, normalCapture->height), IPL_DEPTH_8U, 3);
        segment_base(captureHsv, captureHsvSeg);
        //imshow("mywindow9", captureHsv);
        
        

        //detector.detect(captureGray, keypoints_camera);
        //cv::Mat descriptors_camera;
        //extractor.compute(captureGray, keypoints_camera, descriptors_camera);
/*        
        for (size_t base_image_iter = 0; base_image_iter != base_sift_keypoints.size(); ++base_image_iter)
        {
                unsigned int count = 0;
                for (size_t i = 0; i < base_sift_keypoints[base_image_iter]->size(); ++i) {

                        if(checkForMatch(i, keypoints_camera, base_sift_descriptors[base_image_iter], descriptors_camera))
                        {
                                count++;
                        }
                }
                std::cout << base_image_names[base_image_iter] << "   keypoints: " << count << '\n';  
        }        
*/
        const int freeSpaceThreshold = 20; 
        const int IRThreshold = 210;
      
        // write the moveable vector
        for (size_t i = 0; i < img->width; ++i)
        {
                if (odv[i] <= freeSpaceThreshold)
                {
                        moveable[i] = false;
                }
                else
                {
                        moveable[i] = true;
                }
        }

        // box detection
        BoxDetectionResult boxDetectionResult;
        if (!hasBox)
        {
                boxDetectionResult = detectBoxes(normalCapture, hdCapture, boxVec);
        }
        
        // can we grab the box?
        
        if (movementEnabled && !hasBox && boxDetectionResult.detected && !boxDetectionResult.too_far && boxDetectionResult.centered && correct_box)
        {
                ctrl.stop();
                usleep(500000);
                grabBox(ctrl);
                return;
        }
        
        if (hasBox && canReleaseBox)
        {
                int x_distance = baseCenterX - (normalCapture->width / 2);
                int y_distance = normalCapture->width - baseCenterY;
                double tan = (double)y_distance / (double)x_distance;
		double angle = std::atan(tan);
		angle = angle * 180 / PI;
		dropBox(ctrl, angle);
        }
        
        std::pair<size_t, size_t> boxLine = longestLine(boxVec, img->width);
                
        // are we so close to the box that we only need to rotate so we face it
        /*
        if (!hasBox && boxDetectionResult.detected && !boxDetectionResult.centered)
        {
                
                ctrl.stop();
                usleep(100000);
                size_t line_center = (boxLine.first + boxLine.second) / 2;
                bool rotateLeft = line_center < CAMERA_WIDTH / 2;
                //std::cout << "box center: " << line_center << "; camera center: " << (CAMERA_WIDTH / 2) << '\n';
                if (rotateLeft)
                {
                        // rotate left
                        std::cout << "---- moving on the spot left\n";
                        ctrl.rotateOnSpotLeft();
                        if (boxModel.area != 0) {
                                boxModel.width += 20;
                                boxModel.position.first = boxModel.position.first - 20; 
                        }        
                }
                else
                {
                        // rotate right
                        std::cout << "---- moving on the spot right\n";
                        ctrl.rotateOnSpotRight();
                        if (boxModel.area != 0) {
                                boxModel.width += 20;
                                boxModel.position.first = boxModel.position.first + 20;
                        }        
                }
                usleep(300000);
                ctrl.stop();IRThreshold
                usleep(100000);
                return;
        }*/
        
        // determine if should move towards box        
        bool moveTowardsBox = false;
        int minBoxDistance = img->height;
        if (!hasBox && boxDetectionResult.detected/* && boxDetectionResult.too_far*/) // only move towards box if it is too far
        {
                //std::cout << "box line: " << boxLine.first << ", " << boxLine.second << '\n';
                if (boxLine.second - boxLine.first == 0) 
                {
                        moveTowardsBox = false;
                }
                else
                {
                        moveTowardsBox = true;
                        for (size_t i = boxLine.first; i < boxLine.second; ++i)
                        {
                                minBoxDistance = min(minBoxDistance, boxVec[i]);
                        }

                }
        }    
        if (moveTowardsBox)
        {
                std::cout << "Will move towards box.\n";
        }   
        
        // calculate the line, the centre of which we should move towards
        size_t beginMax = 0;
        size_t distanceMax = 0;
        size_t endMax = 0;
                
        size_t begin = 0;
        size_t distance = 0;
        size_t end = 0;
        
        bool current = false;
        
        for (size_t i = 0; i < img->width; ++i)
        {
                if (current && i == img->width - 1 && moveable[i])
                {
                        end = i;
                        if (distance > distanceMax)
                        {
                                beginMax = begin;
                                endMax = end;
                                distanceMax = distance;
                        }
                }
                if (moveable[i])
                {
                        if (current)
                        {
                                distance++;
                                end = i;
                        }
                        else
                        {
                                distance = 1;
                                begin = i;
                                end = begin;
                                current = true;
                        }
                }

                else
                {
                        if (current)
                        {
                                current = false;
                                if (distance > distanceMax)
                                {
                                        beginMax = begin;
                                        endMax = end;
                                        distanceMax = distance;
                                }
                        }
                        
                }
        }
        //cvLine(img, cvPoint((int)beginMax, 100), cvPoint((int)endMax, 100), CV_RGB(0,0,255));

        //cvShowImage( "mywindow7", img );
        

        
    
        // centre of the line
        double movement_angle = 0;
        int goal_dist = 0;
        int lowestY = 0;
        
        //moveTowardsBox = true; 
        
        if (moveTowardsBox)
        {
                lowestY = minBoxDistance;
                goal_dist = (boxLine.first + boxLine.second) / 2;
                std::cout << "moving towards box\n";
        }
        else
        {
                // calculate the minimum distance between the ground and the obstacles spanned by the line
                lowestY = img->height;
                for (size_t i = beginMax; i <= endMax; ++i)
                {
                        if (odv[i] < lowestY)
                                lowestY = odv[i];
                }
                goal_dist = (beginMax + endMax) / 2;
        
                // signed distance between centre of the line and centre of the image
                
        }

        int bottom_dist = goal_dist - (img->width / 2);
        if (bottom_dist != 0) 
        {

		        double tan = (double)lowestY / (double)bottom_dist;
		        movement_angle = std::atan(tan);
		        movement_angle = movement_angle * 180 / PI;
		        
		        if (movement_angle > 0)
		        { 
				movement_angle = 90 - movement_angle;
			}
		        if (movement_angle < 0)
		        { 
		        	movement_angle = -90 - movement_angle;
        		}
        }
        
        int leftIR = ctrl.getIRLeftValue();
        int rightIR = ctrl.getIRRightValue();
        
        bool moveBack = false;
        if (!moveTowardsBox)
        {
                if (distanceMax < 10)
                {
                        moveBack = true;
                        std::cout << "Moving back: camera facing wall\n";
                }
        }
        //std::cout << "left IR " << leftIR << '/' << IRThreshold << '\n';
        if (leftIR > IRThreshold)
        {
                moveBack = true;
                
        }
        if (rightIR > IRThreshold)
        {
                moveBack = true;
        }
        //std::cout << "right IR " << rightIR << '/' << IRThreshold << '\n';

        /*if (Controller::whiskersTouched)
        {
                std::cout << "Moving back: whiskers touched\n";
                Controller::whiskersTouched = false;
                if (whiskerIgnoreCounter == 0)
                {
                        moveBack = true;
                        whiskerIgnoreCounter = 5;
                }        
                else        
                {
                        whiskerIgnoreCounter--;
                }                
        }*/

        
        if (moveBack)
        {
                if (movementEnabled) 
                        moveBackConsideringFreeSpace(img, odv, ctrl);
                return;
        }
        
        if(distanceMax < 20){
                if (movement_angle < 0)
                {
                	movement_angle = -90;
                }
                else
                {       
                	movement_angle = 90;
        		}
        }

        //std::cout << "counter: " << stoppedForPicturesCounter << ", angle: " << movement_angle << '\n';
        if (stoppedForPicturesCounter <= 1 || moveTowardsBox)
        {
        	if ((lastMoveWasTowardsBox) && !moveTowardsBox)
        	{
        	        //std::cout << "last move was towards box, no box found this time, will wait\n";    
        	}
        	else
        	{
        	        if (movementEnabled) ctrl.turnAt(movement_angle);
        	        //saved_angle = -movement_angle;
        	}
        	        
        	if (moveTowardsBox)
        	{
        	        //std::cout << "\nMOVING TOWARDS BOX DIRECTION: " << movement_angle << '\n';
        	        usleep(100000);
        	        lastMoveWasTowardsBox = true;
        	}       
        	else
        	{
        	        lastMoveWasTowardsBox = false;
        	}
        	
                   
        if(sawBoxCounter == 3)
        {
                sawBoxCounter = 0;
                boxModel.area = 0;
                boxModel.width = 0;
        }        
        else
        {
                sawBoxCounter++;
        }                        
        
        
        	//
        	//ctrl.stop(); 
        cvLine(normalCapture, cvPoint(boxLine.first, 40), cvPoint(boxLine.second, 40), cvScalar(255, 0, 0));
        //if (windowsEnabled) cvShowImage("window4", normalCapture);
        }        
}

void initSift()
{
        
        image_names.push_back("walle.png");
        //keypoint_match_count.push_back(0);
        keypoint_match_count.push_back(11);
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
        image_names.push_back("thor.png");
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
                
                //imshow("mywindow6", input);
        } 
        /*
        for (size_t i = 0; i < base_image_names.size(); ++i)
        {
                std::cout << "Generating descriptors for: " << base_image_names[i] << '\n';
                cv::Mat input = cv::imread(base_image_names[i], 0);
                std::vector<cv::KeyPoint>* keypoints_image = new std::vector<cv::KeyPoint>();
                detector.detect(input, *keypoints_image);                
                cv::Mat descriptors_image;
                extractor.compute(input, *keypoints_image, descriptors_image);                

                base_sift_keypoints.push_back(keypoints_image);
                base_sift_descriptors.push_back(descriptors_image);
                std::cout << "Keypoints: " << base_sift_keypoints[i]->size() << '\n';
                
                //imshow("mywindow6", input);
        } */
        
}

unsigned int matchedImageCounter = 0;
bool detectFeatures(int min_x, int min_y, int max_x, int max_y, IplImage* frameHD)
{
        bool ret = false;
        // Add results to image and save.
        //cv::Mat output;


        //cvNamedWindow( "mywindow", CV_WINDOW_AUTOSIZE );
        //cvNamedWindow( "mywindow2", CV_WINDOW_AUTOSIZE );
/*        cvReleaseCapture(&capture);   
        usleep(10);     
        
        
        capture = cvCaptureFromCAM( CV_CAP_ANY );
        
        cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_WIDTH, 640 );
        cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_HEIGHT, 480 );
*/
        std::vector<cv::KeyPoint> keypoints_camera;   


        /*IplImage* camera = NULL; 
        
        while ( camera == NULL ) {
                camera = cvQueryFrame( capture );
        }*/
        
        cv::Mat cameraMat(frameHD);
        //cv::Mat cameraMatGray;
        //cvtColor( cameraMat, cameraMatGray, CV_RGB2GRAY );

        //cv::Mat interestingImage(frame); 

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
        
        
        //std::cout<<min_x << "," << max_x << "," << min_y << "," << max_y << "," << width << "," << height << std::endl;
        cv::Rect myROI(min_x, min_y, width, height);


        // Crop the full image to that image contained by the rectangle myROI
        // Note that this doesn't copy the data
        cv::Mat croppedImage = cameraMat(myROI);
        cv::Mat croppedImageGray;
        cvtColor(croppedImage, croppedImageGray, CV_RGB2GRAY);
        detector.detect(croppedImageGray, keypoints_camera);
        cv::Mat descriptors_camera;
        extractor.compute(croppedImageGray, keypoints_camera, descriptors_camera);
        
        //if (windowsEnabled) imshow("mywindow5", croppedImageGray);

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
                                /*cv::Mat outputCam;
                                cv::drawKeypoints(croppedImageGray, keypoints_camera, outputCam);
                                imshow("mywindow5", outputCam);
                                std::stringstream ss;
                                ss << "matched";
                                ss << matchedImageCounter;
                                ss << ".png";
                                cv::imwrite(ss.str().c_str(), outputCam);*/
                                matchedImageCounter++;
                                break;
                        }
                }
        }
        

        keypoints_camera.clear();
        return ret;

}

//cv::KeyPoint* checkForMatch(cv::KeyPoint& point, std::vector<cv::KeyPoint>& other_points )
inline bool checkForMatch(size_t point_i, std::vector<cv::KeyPoint>& other_points, cv::Mat& descriptors_image, cv::Mat& descriptors_camera)
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


inline double distSquared(size_t point_a, size_t point_b, cv::Mat& descriptors_image, cv::Mat& descriptors_camera)
{
    int i = 0;
    double distsq = 0.0;

    for (i = 0; i < 128; i++) {
      float dif = descriptors_image.at<float>(point_a, i) - descriptors_camera.at<float>(point_b, i);
      distsq += dif * dif;
    }
    
    return distsq;
}

inline IplImage* small_size_camera_image(IplImage* orig)
{
        IplImage*  orig_small = cvCreateImage( cvSize(CAMERA_WIDTH, CAMERA_HEIGHT), orig->depth, orig->nChannels);
        cvResize(orig, orig_small);
        return orig_small;
}
        
inline IplImage* grabFrame()
{

        /*cvGrabFrame(capture);
        cvGrabFrame(capture);
        cvGrabFrame(capture);
        cvGrabFrame(capture);        
        cvGrabFrame(capture);        
        cvRetrieveFrame( capture );
        cvGrabFrame(capture);        
        cvGrabFrame(capture);        
        cvGrabFrame(capture);        
        cvGrabFrame(capture);    */    
        cvGrabFrame(capture);                
        
        return cvRetrieveFrame( capture );
}

IplImage* orig = NULL;
bool origReady = false;
void* cameraThread(void* Param)
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
                origReady = true;
                //if (windowsEnabled) imshow("mywindow8", orig);
        }        
        
	pthread_exit(NULL);
}
int W1 = 2;
int W2 = 10;
int W3 = 0;
int W4 = 9;
int W5 = 2;
IplImage* orig_small = NULL;
void* baseThread(void* Param)
{
	// Retrieve data
	int id = *((int*)Param);
	// Perform some action
	// Count down from 10 using different speed depending on data
        while (orig_small == NULL) { usleep(1); }
        while (true)
        {
                if (hasBox)
                {
                /// Canny detector
                        std::cout << "whoosh\n";
                        Mat detected_edges;
                        Mat orig_small_copy(orig_small, true);
                        cvtColor( orig_small, detected_edges, CV_BGR2GRAY );
                        cv::Mat sharpened;
                        cv::GaussianBlur(detected_edges, sharpened, cv::Size(0, 0), W1);
                        cv::addWeighted(detected_edges, W2/10.0, sharpened, -W3/10.0, W4, sharpened);
                        imshow( "mywindow3", detected_edges );  

                                                        
                        Canny( sharpened, sharpened, lowThreshold, highThreshold, 3, true );
                        dilate(sharpened, sharpened, getStructuringElement( MORPH_RECT,
                                                        Size( 3, 3 ),
                                                        Point( 2, 2) ));
                        imshow( "mywindow7", sharpened );                                       
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
                                        std::cout << area << "\n";
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
                        

                        for( int i = 0; i< bases.size(); i++ )
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
                                
                                for(int i = baseCenterX - 5; i < baseCenterX+5; i++)
                                {
                                        for(int j = baseCenterY - 5; j < baseCenterY + 5; j++)
                                        {
                                                c1 += orig_small_copy.at<Vec3b>( j , i )[0];
                                                c2 += orig_small_copy.at<Vec3b>( j , i )[1];
                                                c3 += orig_small_copy.at<Vec3b>( j , i )[2];     
                                                orig_small_copy.at<Vec3b>( j , i )[0] = 255;
                                                orig_small_copy.at<Vec3b>( j , i )[1] = 255;
                                                orig_small_copy.at<Vec3b>( j , i )[2] = 255;
                                                //contourDrawing2.at<char>(i,j) = 0;
                                                //std::cout << i << "," << j << "\n";
                                                                            
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
                                
                                c1 /= 100;
                                c2 /= 100;
                                c3 /= 100;
                                
                                std::cout << c1 << "," << c2 << "," << c3 << std::endl;
                                
                                if (160 <= c3 && c3 <= 195 &&
                                    130 <= c2 && c2 <= 165 &&
                                    100 <= c1 && c1 <= 130)
                                {
                                        canReleaseBox = true;                                        
                                        baseType = BASE_QUEEN;
                                } 
                                else if (108 <= c3 && c3 <= 152 &&
                                         108 <= c2 && c2 <= 152 &&
                                         108 <= c1 && c1 <= 152)
                                {
                                        canReleaseBox = true;                                        
                                        baseType = BASE_GREY;
                                } 
                        }
                        imshow( "mywindow8", contourDrawing2 );  
                        imshow( "mywindow10", orig_small_copy );
               }
               else
               {
                        canReleaseBox = false;
               }
           
        }        

	pthread_exit(NULL);
}


int main(int argc, char** argv) {

        initSift();
        
        bool moveable[CAMERA_WIDTH];
        for (size_t i = 0; i < CAMERA_WIDTH; ++i)
        {
          moveable[i] = false;
        }
      

        
	printf("Starting threads...\n");
	// Create thread object
	pthread_t thread1;
	pthread_t thread2;
	// Create data
	int id1 = 1;
	int id2 = 2;
	// Start thread
	pthread_create(&thread1, NULL, cameraThread, (void*)&id1);
	pthread_create(&thread2, NULL, baseThread, (void*)&id2);

        if (windowsEnabled) 
        {
                // Create a window in which the captured images will be presented
                //cvNamedWindow( "mywindow", CV_WINDOW_AUTOSIZE );
                //cvNamedWindow( "mywindow2", CV_WINDOW_AUTOSIZE );
                cvNamedWindow( "mywindow3", CV_WINDOW_NORMAL );
                //cvNamedWindow( "mywindow4", CV_WINDOW_AUTOSIZE );
                //cvNamedWindow( "mywindow5", CV_WINDOW_AUTOSIZE );
                cvNamedWindow( "mywindow6", CV_WINDOW_NORMAL );
                cvNamedWindow( "mywindow7", CV_WINDOW_NORMAL );
                cvNamedWindow( "mywindow8", CV_WINDOW_NORMAL );
                //cvNamedWindow( "mywindow9", CV_WINDOW_AUTOSIZE );
                cvNamedWindow( "mywindow10", CV_WINDOW_NORMAL );                
                
                cvMoveWindow("mywindow", 0, 20);
                //cvMoveWindow("mywindow2", 400, 20);
                cvMoveWindow("mywindow3", 800, 20);
                //cvMoveWindow("mywindow3", 0, 320);
                //cvMoveWindow("mywindow4", 400, 320);
                cvMoveWindow("mywindow5", 800, 320);
                cvMoveWindow("mywindow6", 0, 620);
                cvMoveWindow("mywindow8", 800, 620);
                //cvMoveWindow("mywindow9", 600, 620);
                cvMoveWindow("mywindow10", 600, 620);
                


                /*cvCreateTrackbar( "a", "mywindow2", &a, 40, NULL );
                cvCreateTrackbar( "b", "mywindow2", &b, 40, NULL );
                cvCreateTrackbar( "c", "mywindow2", &c, 100, NULL );
                cvCreateTrackbar( "d", "mywindow4", &d, 40, NULL );
                cvCreateTrackbar( "e", "mywindow4", &e, 40, NULL );
                cvCreateTrackbar( "minRatio", "mywindow4", &minRatio, 2000, NULL );
                cvCreateTrackbar( "maxRatio", "mywindow4", &maxRatio, 2000, NULL );
                cvCreateTrackbar( "min_size", "mywindow4", &contourMinSize, 1000, NULL );
                cvCreateTrackbar( "max_size", "mywindow4", &contourMaxSize, 1000, NULL );
                cvCreateTrackbar( "minHue", "mywindow8", &minHue, 255, NULL );
                cvCreateTrackbar( "maxHue", "mywindow8", &maxHue, 255, NULL );
                cvCreateTrackbar( "minSat", "mywindow8", &minSat, 255, NULL );
                cvCreateTrackbar( "maxSat", "mywindow8", &maxSat, 255, NULL );
                cvCreateTrackbar( "minVal", "mywindow8", &minVal, 255, NULL );
                cvCreateTrackbar( "maxVal", "mywindow8", &maxVal, 255, NULL );*/
                cvCreateTrackbar( "lowT", "mywindow10", &lowThreshold, 400, NULL );
                cvCreateTrackbar( "highT", "mywindow10", &highThreshold, 400, NULL );
                cvCreateTrackbar( "p", "mywindow10", &PolygonBase, 100, NULL );
                cvCreateTrackbar( "W1", "mywindow3", &W1, 100, NULL );
                cvCreateTrackbar( "W2", "mywindow3", &W2, 100, NULL );
                cvCreateTrackbar( "W3", "mywindow3", &W3, 100, NULL );
                cvCreateTrackbar( "W4", "mywindow3", &W4, 100, NULL );
                //cvCreateTrackbar( "W5", "mywindow3", &W5, 100, NULL );
        }
        int odv[CAMERA_WIDTH];
        int boxVec[CAMERA_WIDTH];
        
	while (!origReady) { std::cout << "waiting for camera thread...\n"; };
        while ( 1 ) {
                std::cout << "has box? " << hasBox << '\n';
                if(boxDetected){
                        //::cout<<"BOXDETECTED: " << boxDetected << std::endl;
                        ctrl.stop();
                        usleep(100000);
                }
                
                
                //std::cout << orig->width << 'x' << orig->height << '\n';
                /*if (saved_angle != -1)
                {
                        ctrl.turn(saved_angle);
                        usleep(100000);
                }*/
                cvShowImage("mywindow", orig);
                orig_small = small_size_camera_image(orig);
                
  /// Reduce noise with a kernel 3x3
  //blur( src_gray, detected_edges, Size(3,3) );


  
                //cvShowImage("mywindow3", orig_small);
                
                // preprocessed frame for navigation
                IplImage* prepared = cvCreateImage(cvSize(orig_small->width,orig_small->height),IPL_DEPTH_8U, orig_small->nChannels);
                cvSmooth(orig_small, prepared, CV_GAUSSIAN, 3, 3);
                cvDilate(prepared, prepared, NULL, 5);
                                 
                if ( !prepared ) {
            		std::cerr << "ERROR: capture is NULL \n";
                    break;
                }
                
                for (size_t i = 0; i < CAMERA_WIDTH; ++i)
                {
                        odv[i] = 0;
                }
                
                IplImage* detected_floor = cvCreateImage(cvSize(prepared->width, prepared->height),IPL_DEPTH_8U, prepared->nChannels); 
                  
                // Segment floor from image and write obstacle distances to odv vector
                segment_floor(prepared, detected_floor, odv);
                
                // Navigate on free space
                boxDetected = false;
                run(moveable, odv, detected_floor, ctrl, orig_small, orig, boxVec);

                cvReleaseImage(&prepared);
                cvReleaseImage(&detected_floor);
                correct_box = false;
                // Do not release the frame!
                //If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
                //remove higher bits using AND operator
                if ( (cvWaitKey(10) & 255) == 27 ) break;
                stoppedForPicturesCounter = stoppedForPicturesCounter > 0 ? stoppedForPicturesCounter - 1 : 0;
        }

        // Release the capture device housekeeping
        cvReleaseCapture( &capture );
        if (windowsEnabled) 
        {
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
       /* cvDestroyWindow( "mywindow7" );
        */

        return 0;
}
