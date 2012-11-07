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

const int CAMERA_WIDTH = 181;
const int CAMERA_HEIGHT = 96;
const int REAL_WIDTH = 544;
const int REAL_HEIGHT = 288;

/*
int a = 4;
int b = 5;
int c = 15;
*/
int contourMinSize = 80;
int a = 18;
int b = 17;
int c = 40;
int d = 29;
int e = 10;

bool checkForMatch(size_t point_i, std::vector<cv::KeyPoint>& other_points, cv::Mat& descriptors_image, cv::Mat& descriptors_camera);
double distSquared(size_t point_a, size_t point_b, cv::Mat& descriptors_image, cv::Mat& descriptors_camera);
bool detectFeatures(int min_x, int min_y, int max_x, int max_y, IplImage* frameHD);
        
Controller ctrl;
CvCapture* capture;
int stoppedForPicturesCounter = 0;
bool hasBox = false;

bool inRange(unsigned char red, unsigned char green, unsigned char blue, int range)
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
        
        cvShowImage( "mywindow", dst ); 	
}


void moveBackConsideringFreeSpace(IplImage* img, int* odv, Controller& ctrl)
{
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


void interpolatedCoordinates(int &min_x, int &min_y, int &max_x, int &max_y, int width, int height)
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
        BoxDetectionResult ret;
        IplImage* gray2;
        gray2 = cvCreateImage(cvSize(frame->width, frame->height), IPL_DEPTH_8U, 1);
        cvCvtColor(frame,gray2,CV_BGR2GRAY);
        
        IplImage* gray2Dilated = cvCreateImage(cvSize(frame->width, frame->height), IPL_DEPTH_8U, 1);
        cvCopy(gray2, gray2Dilated);
        //cvNot(gray2, gray2Inv);
        
        
        CvScalar avg;
        CvScalar avgStd;
        cvAvgSdv(gray2, &avg, &avgStd, NULL);
        
        cvThreshold(gray2, gray2, (int)avg.val[0] - a* (int)(avgStd.val[0]/b), 255, CV_THRESH_BINARY_INV);
        
        cvThreshold(gray2Dilated, gray2Dilated, (int)avg.val[0] - d* (int)(avgStd.val[0]/e), 255, CV_THRESH_BINARY_INV);
        cvDilate(gray2Dilated, gray2Dilated, NULL, 2);
        
        //cvShowImage("mywindow4", gray2);
                
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
       
        imshow("mywindow4", gray_CvMat);
        imshow("mywindow8",  grayDilated_CvMat);
        
              
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        
        findContours(grayDilated_CvMat, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
        Mat drawing = Mat::zeros( gray_CvMat.size(), CV_8UC1 );    
        vector<vector<Point> > squares;
        vector<Point> approx;
        
        for (size_t i = 0; i < contours.size(); ++i)
        {
                approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*(1.0/c), true);
                squares.push_back(approx);
        }


        int idx = 0;
        if (!squares.empty())
        {
                for( ; idx >= 0; idx = hierarchy[idx][0] )
                {
                        std::cout << "-----\n";
                        for (vector<Point>::iterator i = squares[idx].begin(); i != squares[idx].end(); ++i)
                        {
                                std::cout << " < " << (*i) << '\n';
                        }
                        
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
                        if(1500 > area && area > contourMinSize && 1.6f > ratio && ratio > 0.4f && boundingBoxArea < 2*area)
                        {
                                drawContours(drawing, squares, idx, Scalar(255, 0, 0), CV_FILLED, 8, hierarchy);

                                if (stoppedForPicturesCounter == 0)
                                        stoppedForPicturesCounter = 5;
                

                                int min_x_ = min_x;
                                int min_y_ = min_y;
                                int max_x_ = max_x;
                                int max_y_ = max_y;
                                interpolatedCoordinates(min_x, min_y, max_x, max_y, drawing.cols, drawing.rows);
                                ctrl.stop();
                                detectFeatures(min_x, min_y, max_x, max_y, frameHD);
                                ret.detected = true;

                                
                                const int center_error = 50;
                                if (ret.detected)
                                {
                                        std::cout << "**** box detected with area: " << area << '\n';
                                        ret.too_far = (area < 500);
                                        if (ret.too_far)
                                        {
                                                std::cout << "**** Box is too far to approach.\n";
                                        }
                                        
                                        int box_center_x = ((min_x + max_x) / 2);
                                        int box_center_y = ((min_y + max_y) / 2);
                                        int image_center_x = REAL_WIDTH / 2;
                                        int image_center_y = REAL_HEIGHT / 2;
                                        std::cout << "**** box at " << box_center_x << ", " << box_center_y << "; image center at " << image_center_x << ", " << image_center_y << "\n";

                                        ret.centered = ((image_center_x - center_error) < box_center_x && box_center_x < (image_center_x + center_error));
                                                                                       
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
        }
        
        Mat& drawingScaled = drawing;
        imshow("mywindow2", drawingScaled);
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

std::pair<size_t, size_t> longestLine(int* boxVec, size_t size)
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


void stopAndRotate(Controller& ctrl)
{
        std::cout << "stopping and rotating...\n";
        ctrl.stop(); 
        usleep(1000000);
        ctrl.rotateOnSpot();
        usleep(1000000);
}

void grabBox(Controller& ctrl)
{
        std::cout << "-- Grabbing box\n";
        ctrl.openServo();
        ctrl.turn(0); 
        usleep(1800000);
        ctrl.closeServo();
        usleep(1000000);
        ctrl.stop();
        hasBox = true;
}

void run(bool* moveable, int* odv, IplImage* img, Controller& ctrl, IplImage* normalCapture, IplImage* hdCapture, int* boxVec)
{
        const int freeSpaceThreshold = 70; 
        const int IRThreshold = 390;
      
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
        BoxDetectionResult boxDetectionResult = detectBoxes(normalCapture, hdCapture, boxVec);
        
        // can we grab the box?
        if (!hasBox && boxDetectionResult.detected && !boxDetectionResult.too_far && boxDetectionResult.centered)
        {
                ctrl.stop();
                usleep(500000);
                //grabBox(ctrl);
                return;
        }

        std::pair<size_t, size_t> boxLine = longestLine(boxVec, img->width);
                
        // are we so close to the box that we only need to rotate so we face it?
        if (!hasBox && boxDetectionResult.detected && !boxDetectionResult.too_far && !boxDetectionResult.centered)
        {
                
                ctrl.stop();
                usleep(100000);
                size_t line_center = (boxLine.first + boxLine.second) / 2;
                bool rotateLeft = line_center < CAMERA_WIDTH / 2;
                std::cout << "box center: " << line_center << "; camera center: " << (CAMERA_WIDTH / 2) << '\n';
                if (rotateLeft)
                {
                        // rotate left
                        std::cout << "---- moving on the spot left\n";
                        //ctrl.rotateOnSpotLeft();
                }
                else
                {
                        // rotate right
                        std::cout << "---- moving on the spot right\n";
                        //ctrl.rotateOnSpotRight();
                }
                usleep(100000);
                ctrl.stop();
                usleep(500000);
                return;
        }
        
        // determine if should move towards box        
        bool moveTowardsBox = false;
        int minBoxDistance = img->height;
        if (!hasBox && boxDetectionResult.detected && boxDetectionResult.too_far) // only move towards box if it is too far
        {
                std::cout << "box line: " << boxLine.first << ", " << boxLine.second << '\n';
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
        cvLine(img, cvPoint((int)beginMax, 100), cvPoint((int)endMax, 100), CV_RGB(0,0,255));

        cvShowImage( "mywindow7", img );
        

        
    
        // centre of the line
        double movement_angle = 0;
        int goal_dist = 0;
        int lowestY = 0;
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
        if (distanceMax < 10)
        {
                moveBack = true;
                std::cout << "Moving back: camera facing wall\n";
        }
        if (leftIR > IRThreshold)
        {
                moveBack = true;
                std::cout << "Moving back: left IR\n";
        }
        if (rightIR > IRThreshold)
        {
                moveBack = true;
                std::cout << "Moving back: right IR\n";
        }

        
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
                //moveBackConsideringFreeSpace(img, odv, ctrl);
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

        std::cout << "counter: " << stoppedForPicturesCounter << ", angle: " << movement_angle << '\n';
        if (stoppedForPicturesCounter <= 1)
        {
        	//ctrl.turn(movement_angle);
        	if (moveTowardsBox)
        	        usleep(500000);
        }        
}

std::vector< std::vector<cv::KeyPoint>* > sift_keypoints;
std::vector< cv::Mat > sift_descriptors;
cv::Mat sift_images;
cv::SiftFeatureDetector detector;
cv::SiftDescriptorExtractor extractor;
std::vector<std::string> image_names;     

void initSift()
{

        /*image_names.push_back("walle.png");
        image_names.push_back("ferrari.png");
        image_names.push_back("celebes.png");
        image_names.push_back("fry.png");
        image_names.push_back("mario.png");
        image_names.push_back("terminator.png");*/
        /*image_names.push_back("iron.png");*/
        image_names.push_back("starry.png");
        /*image_names.push_back("thor.png");*/


        for (size_t i = 0; i < image_names.size(); ++i)
        {
                std::cout << "Generating descriptors for: " << image_names[i] << '\n';
                cv::Mat input = cv::imread(image_names[i], 0);
                std::vector<cv::KeyPoint>* keypoints_image = new std::vector<cv::KeyPoint>();
                detector.detect(input, *keypoints_image);                
                cv::Mat descriptors_image;
                extractor.compute(input, *keypoints_image, descriptors_image);                
                sift_images = input;
                sift_keypoints.push_back(keypoints_image);
                sift_descriptors.push_back(descriptors_image);
                imshow("mywindow6", input);
        } 
        
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
                        
                        if (count >= 11) 
                        {
                                ret = true;
                                cv::Mat outputCam;
                                cv::drawKeypoints(croppedImageGray, keypoints_camera, outputCam);
                                imshow("mywindow5", outputCam);
                                std::stringstream ss;
                                ss << "matched";
                                ss << matchedImageCounter;
                                ss << ".png";
                                cv::imwrite(ss.str().c_str(), outputCam);
                                matchedImageCounter++;
                                break;
                        }
                }
        }
        

        keypoints_camera.clear();
        return ret;

}

//cv::KeyPoint* checkForMatch(cv::KeyPoint& point, std::vector<cv::KeyPoint>& other_points )
bool checkForMatch(size_t point_i, std::vector<cv::KeyPoint>& other_points, cv::Mat& descriptors_image, cv::Mat& descriptors_camera)
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


double distSquared(size_t point_a, size_t point_b, cv::Mat& descriptors_image, cv::Mat& descriptors_camera)
{
    int i = 0;
    double distsq = 0.0;

    for (i = 0; i < 128; i++) {
      float dif = descriptors_image.at<float>(point_a, i) - descriptors_camera.at<float>(point_b, i);
      distsq += dif * dif;
    }
    
    return distsq;
}

int main(int argc, char** argv) {

        initSift();
        
        bool moveable[CAMERA_WIDTH];
        for (size_t i = 0; i < CAMERA_WIDTH; ++i)
        {
          moveable[i] = false;
        }
      
        capture = cvCaptureFromCAM( CV_CAP_ANY );  
        cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_WIDTH, REAL_WIDTH );
        cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_HEIGHT, REAL_HEIGHT );
        
        if ( !capture ) {
                std::cerr << "ERROR: capture is NULL \n";
                return -1;
        }
        
        // Create a window in which the captured images will be presented
        cvNamedWindow( "mywindow", CV_WINDOW_AUTOSIZE );
        cvNamedWindow( "mywindow2", CV_WINDOW_AUTOSIZE );
        cvNamedWindow( "mywindow3", CV_WINDOW_AUTOSIZE );
        cvNamedWindow( "mywindow4", CV_WINDOW_AUTOSIZE );
        cvNamedWindow( "mywindow5", CV_WINDOW_AUTOSIZE );
        cvNamedWindow( "mywindow6", CV_WINDOW_AUTOSIZE );
        cvNamedWindow( "mywindow7", CV_WINDOW_AUTOSIZE );
        cvNamedWindow( "mywindow8", CV_WINDOW_AUTOSIZE );
        
        cvMoveWindow("mywindow", 0, 20);
        cvMoveWindow("mywindow2", 400, 20);
        cvMoveWindow("mywindow3", 800, 20);
        cvMoveWindow("mywindow3", 0, 320);
        cvMoveWindow("mywindow4", 400, 320);
        cvMoveWindow("mywindow5", 800, 320);
        cvMoveWindow("mywindow6", 0, 620);
        cvMoveWindow("mywindow7", 400, 620);
        cvMoveWindow("mywindow8", 800, 620);


        cvCreateTrackbar( "a", "mywindow2", &a, 40, NULL );
        cvCreateTrackbar( "b", "mywindow2", &b, 40, NULL );
        cvCreateTrackbar( "c", "mywindow2", &c, 100, NULL );
        cvCreateTrackbar( "d", "mywindow8", &d, 40, NULL );
        cvCreateTrackbar( "e", "mywindow8", &e, 40, NULL );
        cvCreateTrackbar( "size", "mywindow8", &contourMinSize, 1000, NULL );

        int odv[CAMERA_WIDTH];
        int boxVec[CAMERA_WIDTH];
        
        
        while ( 1 ) {
        
                IplImage* orig = NULL;
           
                cvGrabFrame(capture);
                cvGrabFrame(capture);
                
                orig = cvRetrieveFrame( capture );
                IplImage* orig_small = cvCreateImage( cvSize(CAMERA_WIDTH, CAMERA_HEIGHT), orig->depth, orig->nChannels);
                
                // resize image from camera for real time navigations
                cvResize(orig, orig_small);
                
                cvShowImage("mywindow3", orig_small);
                
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
                run(moveable, odv, detected_floor, ctrl, orig_small, orig, boxVec);

                cvReleaseImage(&prepared);
                cvReleaseImage(&detected_floor);
                
                // Do not release the frame!
                //If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
                //remove higher bits using AND operator
                if ( (cvWaitKey(10) & 255) == 27 ) break;
                stoppedForPicturesCounter = stoppedForPicturesCounter > 0 ? stoppedForPicturesCounter - 1 : 0;
        }

        // Release the capture device housekeeping
        cvReleaseCapture( &capture );
        cvDestroyWindow( "mywindow" );
        cvDestroyWindow( "mywindow2" );
        cvDestroyWindow( "mywindow3" );
        cvDestroyWindow( "mywindow4" );
        cvDestroyWindow( "mywindow5" );
        cvDestroyWindow( "mywindow6" );
        cvDestroyWindow( "mywindow7" );
        cvDestroyWindow( "mywindow8" );

        return 0;
}
