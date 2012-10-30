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
//#include "controller.hpp"

using namespace cv;
const int CAMERA_WIDTH = 256;
const int CAMERA_HEIGHT = 192;
//const int CAMERA_WIDTH = 180;
//const int CAMERA_HEIGHT = 135;
//const int CAMERA_WIDTH = 320;
//const int CAMERA_HEIGHT = 240;
const int CENTER_OFFSET = 0;

bool checkForMatch(size_t point_i, std::vector<cv::KeyPoint>& other_points, cv::Mat& descriptors_image, cv::Mat& descriptors_camera);
double distSquared(size_t point_a, size_t point_b, cv::Mat& descriptors_image, cv::Mat& descriptors_camera);
bool detectFeatures(int min_x, int min_y, int max_x, int max_y, IplImage* frameHD);
        
Controller ctrl;
std::vector<cv::KeyPoint> matching;  
CvCapture* capture;
int stoppedForPicturesCounter = 0;

int calcHist(IplImage* img, int* colors)
{
        /* Always check if the program can find a file */
        if( !img )
        return -1;

        IplImage* channel = cvCreateImage( cvGetSize(img), 8, 1 );
        IplImage *hist_img = cvCreateImage(cvSize(CAMERA_WIDTH,CAMERA_HEIGHT), 8, 3);
        cvSet( hist_img, cvScalarAll(255), 0 );

        CvHistogram *hist_red;
        CvHistogram *hist_green;
        CvHistogram *hist_blue;

        int hist_size = 256;      
        float range[]={0,256};
        float* ranges[] = { range };

        float max_value = 0.0;
        float max = 0.0;
        float w_scale = 0.0;

        /* Create a 1-D Arrays to hold the histograms */
        hist_red = cvCreateHist(1, &hist_size, CV_HIST_ARRAY, ranges, 1);
        hist_green = cvCreateHist(1, &hist_size, CV_HIST_ARRAY, ranges, 1);
        hist_blue = cvCreateHist(1, &hist_size, CV_HIST_ARRAY, ranges, 1);

        /* Set image to obtain RED as Channel of Interest (COI) */
        cvSetImageCOI(img,3);
        cvCopy(img,channel);
        cvResetImageROI(img);

        /* Calculate histogram of the Image and store it in the array */
        cvCalcHist( &channel, hist_red, 0, NULL );

        /* Calculate and Plot the histograms Green and Blue channels as well */
        /* Green channel */
        cvSetImageCOI(img,2);
        cvCopy(img,channel);
        cvResetImageROI(img);

        cvCalcHist( &channel, hist_green, 0, NULL );

        /* Blue channel */
        cvSetImageCOI(img,1);
        cvCopy(img,channel);
        cvResetImageROI(img);

        cvCalcHist( &channel, hist_blue, 0, NULL );


        /* Find the minimum and maximum values of the histograms */
        cvGetMinMaxHistValue( hist_red, 0, &max_value, 0, 0 );
        cvGetMinMaxHistValue( hist_green, 0, &max, 0, 0 );

        max_value = (max > max_value) ? max : max_value;

        cvGetMinMaxHistValue( hist_blue, 0, &max, 0, 0 );

        max_value = (max > max_value) ? max : max_value;    
        // The variable max_value has the maximum of the three histograms

        /* Using the maximum value, Scale/Squeeze the histogram (to fit the image) */
        cvScale( hist_red->bins, hist_red->bins, ((float)hist_img->height)/max_value, 0 );
        cvScale( hist_green->bins, hist_green->bins, ((float)hist_img->height)/max_value, 0 );
        cvScale( hist_blue->bins, hist_blue->bins, ((float)hist_img->height)/max_value, 0 );

        printf("Scale: %4.2f pixels per 100 units\n", max_value*100/((float)hist_img->height));                         
           //A scale to estimate the number of pixels

        /* Scale/Squeeze the histogram range to image width */
        w_scale = ((float)hist_img->width)/hist_size;
        
        int Rmax = 0;//hist_img->height+1;
        int Gmax = 0;//hist_img->height+1;
        int Bmax = 0;//hist_img->height+1;
        
        int rm = 0;
        int gm = 0;
        int bm = 0;

        /* Plot the Histograms */
        for( int i = 1; i < hist_size; i++ )
        {
                if (Rmax < cvRound(cvGetReal1D(hist_red->bins,i)) ){
                        rm = i;
                        Rmax = cvRound(cvGetReal1D(hist_red->bins,i));
                        //std::cout<< "rmax " << cvRound(cvGetReal1D(hist_red->bins,i)) << " i: " << i <<std::endl;
                }
                
                if (Gmax < cvRound(cvGetReal1D(hist_green->bins,i)) ){
                        gm = i;
                        Gmax = cvRound(cvGetReal1D(hist_green->bins,i));
                        //std::cout<< "gmax " << cvRound(cvGetReal1D(hist_green->bins,i)) << " i: " << i <<std::endl;
                }
                
                if (Bmax < cvRound(cvGetReal1D(hist_blue->bins,i)) ){
                        bm = i;
                        Bmax = cvRound(cvGetReal1D(hist_blue->bins,i));
                        //std::cout<< "bmax " << cvRound(cvGetReal1D(hist_blue->bins,i)) << " i: " << i <<std::endl;
                }
        
                cvRectangle( hist_img, cvPoint((int)i*w_scale , hist_img->height),
                cvPoint((int)(i+1)*w_scale, hist_img->height - cvRound(cvGetReal1D(hist_red->bins,i))),
                CV_RGB(255,0,0), -1, 8, 0 );
                
                //std::cout << "x: " << ((int)(i+1)*w_scale) << ", y: " << (hist_img->height - cvRound(cvGetReal1D(hist_red->bins,i))) <<'\n';
                
                cvRectangle( hist_img, cvPoint((int)i*w_scale , hist_img->height),
                cvPoint((int)(i+1)*w_scale, hist_img->height - cvRound(cvGetReal1D(hist_green->bins,i))),
                CV_RGB(0,255,0), -1, 8, 0 );
                
                cvRectangle( hist_img, cvPoint((int)i*w_scale , hist_img->height),
                cvPoint((int)(i+1)*w_scale, hist_img->height - cvRound(cvGetReal1D(hist_blue->bins,i))),
                CV_RGB(0,0,255), -1, 8, 0 );
        }

        /*
        cvNamedWindow( "Image", 1 );
        cvShowImage( "Image",img);

        cvNamedWindow("Histogram", 1);
        cvShowImage( "Histogram", hist_img);

        cvWaitKey(0);

        cvDestroyWindow( "Image" );
        cvDestroyWindow( "Histogram" );
        cvReleaseImage( &img );
        */
        //std::cout << "max r: " << Rmax << " g: " << Gmax << " b: " << Bmax << '\n';
        //std::cout << "max r: " << rm << " g: " << gm << " b: " << bm << '\n';
        
        cvReleaseImage( &hist_img );
        cvReleaseImage( &channel );
        colors[0] = rm;
        colors[1] = gm;
        colors[2] = bm;
        return 0;
}

bool inRange(unsigned char red, unsigned char green, unsigned char blue, int range, int* colors)
{
/*
  return !((red <= colors[0] - range || red >= colors[0] + range) ||
         (green <= colors[1] - range || green >= colors[1] + range) ||
         (blue <= colors[2] - range || blue >= colors[2] + range));
*/


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

bool isGround(IplImage* src, IplImage* dst, int* colors, int* odv)
{
        //cvShowImage( "mywindow", src );
        //cvWaitKey(0);
        const int range = 13;
        IplImage* grid = dst;
        //IplImage* grid = cvCreateImage(cvSize(src->width,src->height), IPL_DEPTH_8U, 1); 
        
        for (int i = 0; i < src->height; i++)
        {
                for (int k = 0; k < src->width; k += 1)
                {
                        int j = k * grid->nChannels;
                        unsigned char red = src->imageData[i * src->widthStep + j + 2];
                        unsigned char green = src->imageData[i * src->widthStep + j + 1];
                        unsigned char blue = src->imageData[i * src->widthStep + j];
                        //std::cout << ">>> r: " << ((int)red) << " g: " << ((int)green) << " b: " << ((int)blue) << ' ';
                        if (!inRange(red, green, blue, range, colors))
                        {

                                const unsigned char value = 0;

                                ((uchar *)(grid->imageData + i * grid->widthStep))[j] = blue;
                                ((uchar *)(grid->imageData + i * grid->widthStep))[j+1] = green;
                                ((uchar *)(grid->imageData + i * grid->widthStep))[j+2] = red;
                        }
                        else
                        {
                                const unsigned char value = 255;

                                ((uchar *)(grid->imageData + i * grid->widthStep))[j] = value;
                                ((uchar *)(grid->imageData + i * grid->widthStep))[j+1] = value;
                                ((uchar *)(grid->imageData + i * grid->widthStep))[j+2] = value;
                                //std::cout << "IN RANGE!\n";
                        }
                        //cvShowImage( "mywindow2", grid );                       
                        //sleep(1);
                }
        }
        
        std::memset(odv, 0, sizeof(int));

        
        //std::cout << "Drawing image: \n";
        for (int i = 0; i < grid->height; i++)
        {
                for (int k = 0; k < grid->width; k += 1)
                {
                        int j = k * grid->nChannels;
                        unsigned char red = grid->imageData[i * grid->widthStep + j + 2];
                        unsigned char green = grid->imageData[i * grid->widthStep + j + 1];
                        unsigned char blue = grid->imageData[i * grid->widthStep + j];
                        if (red == 255 && green == 255 && blue == 255)
                        {
                                //std::cout << '1';

                        } else 
                        { 
                                //std::cout << '0'; 
                                if (odv[k] < i)
                                {
                                        odv[k] = i;
                                }  
                       }
                      
                }
                //std::cout << '\n';
        }
        
        /*
        std::cout << "odv: ";
        for (size_t i = 0; i < CAMERA_WIDTH; ++i)
        {
          std::cout << odv[i] << ' ';
        }
        std::cout << '\n';
        */
        for (size_t i = 0; i < CAMERA_WIDTH; ++i)
        {
          odv[i] = grid->height - odv[i];
        }
        
        //std::cout << "origin: " << src->origin << '\n';

        
        cvShowImage( "mywindow", grid );
        //cvWaitKey(0);
        return true;
}


bool moveBackConsideringFreeSpace(IplImage* img, int* odv, Controller& ctrl)
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
        usleep(500);
        if (leftSpaceAverage < rightSpaceAverage)
        {
                ctrl.moveBackwardRight();
        }
        else
        {
                ctrl.moveBackwardLeft();
        }
}

double variance(vector<Point>& vec)
{
        double meanX = 0;
        double meanY = 0;
        for (size_t i = 0; i < vec.size(); ++i)
        {
                Point& p = vec[i];
                meanX += p.x;
                meanY += p.y;
        }
        meanX /= vec.size();
        meanY /= vec.size();
        double meanXs = 0;
        double meanYs = 0;
        for (size_t i = 0; i < vec.size(); ++i)
        {
                Point& p = vec[i];
                meanXs += p.x*p.x;
                meanYs += p.y*p.y;
        }
        meanXs /= vec.size();
        meanYs /= vec.size();
        
        double retX = meanXs - meanX * meanX;
        double retY = meanYs - meanY * meanY;
        //std::cout << "var: " << retX << ", " << retY << '\n';
        return 0;
} 

void interpolatedCoordinates(int &min_x, int &min_y, int &max_x, int &max_y, int width, int height)
{
    //std::cout << "INTERPOLATED:\n";
    //std::cout << (((float)min_x) / ((float)width)) << ", " << max_x << ", " << min_y << ", " << max_y << ", " << width << ", " << height << '\n';    
    min_x = ((float)min_x) / width * 640;
    max_x = ((float)max_x) / width * 640;
    min_y = ((float)min_y) / height * 480;
    max_y = ((float)max_y) / height * 480;
    //std::cout << min_x_ << ", " << max_x_ << ", " << min_y_ << ", " << max_y_ << '\n';

}

/*int a = 100;
int b = 100;
int c = 1;
int d = 3;*/
bool detectBoxes(IplImage* frame, IplImage* frameHD, int* boxVec)
{
        bool ret = false;

        IplImage* gray2;
        gray2 = cvCreateImage(cvSize(frame->width, frame->height), IPL_DEPTH_8U, 1);
   
        //  cvtColor( image, gray0, CV_BGR2GRAY );
        cvCvtColor(frame,gray2,CV_BGR2GRAY);

        
        cv::Mat grad;
        cv::Mat grad_x, grad_y;
        cv::Mat abs_grad_x, abs_grad_y;

        CvScalar avg;
        CvScalar avgStd;
        cvAvgSdv(gray2, &avg, &avgStd, NULL);

        //cvSmooth(gray2, gray2, CV_GAUSSIAN, 3, 3);
        cvThreshold(gray2, gray2, (int)avg.val[0]-7*(int)(avgStd.val[0]/8), 255, CV_THRESH_BINARY_INV);

        //cvErode(gray2, gray2, NULL,1); 

        //cvDilate(gray2, gray2, NULL, 2);
        
        
        cv::Mat gray0(gray2, false);
        
        int scale = 1;
        int delta = 0;
        int ddepth = CV_16S; 
        /// Gradient X
        //Scharr( src_gray, grad_x, ddepth, 1, 0, scale, delta, BORDER_DEFAULT );
        Sobel( gray0, grad_x, ddepth, 1, 0, 3, scale, delta, BORDER_DEFAULT );
        convertScaleAbs( grad_x, abs_grad_x );

        /// Gradient Y
        //Scharr( src_gray, grad_y, ddepth, 0, 1, scale, delta, BORDER_DEFAULT );
        Sobel( gray0, grad_y, ddepth, 0, 1, 3, scale, delta, BORDER_DEFAULT );
        convertScaleAbs( grad_y, abs_grad_y );

        /// Total Gradient (approximate)
        addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, grad );
        //imshow( "mywindow", grad );


        // karlphillip: dilate the image so this technique can detect the white square,
        cv::Mat gray1;
        // 71 68
        Canny( gray0, gray1, 226, 300, 5, true );
        vector<vector<Point> > contours;
        vector<Vec4i> hierarchy;
        
        //imshow("preBoxDetection", gray0);
        //imshow("preBoxDetectionsssssss", grad);
        

        //findContours(gray1, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
        findContours(gray0, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
        Mat drawing = Mat::zeros( gray1.size(), CV_8UC1 );    

        vector<vector<Point> > squares;

        vector<Point> approx;
        for (size_t i = 0; i < contours.size(); ++i)
        {
                approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*(1.0/20), true);
                squares.push_back(approx);
        }


        Size drawingSize = drawing.size();

        int idx = 0;
        for( ; idx >= 0; idx = hierarchy[idx][0] )
        {
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






                //std::cout << idx << ": " << area << ", " << ratio << ", " << boundingBoxArea <<  '\n';                
                //variance(squares[idx]);
                int edgeProximityThreshold = 2;
                bool tooCloseToEdge = false;
                for (size_t i = 0; i < squares[idx].size(); ++i)
                {
                        Point& p = squares[idx][i];
                        if (p.x <= edgeProximityThreshold || drawingSize.width - p.x <= edgeProximityThreshold)
                        {
                                tooCloseToEdge = true;
                                break;
                        }        
                        if (p.y <= edgeProximityThreshold/* || drawingSize.height - p.y <= edgeProximityThreshold*/)
                        {
                                tooCloseToEdge = true;
                                break;
                        }        
                }
                
//                if (tooCloseToEdge) continue;
                
                if(2000 > area && area > 150 && 1.6f > ratio && ratio > 0.4f && boundingBoxArea < 2*area)
                {
                        drawContours(drawing, squares, idx, Scalar(255, 0, 0), CV_FILLED, 8, hierarchy);
                        
                        // Transform it into the C++ cv::Mat format
                        //cv::Mat interestingImage(frame); 

                        // Setup a rectangle to define your region of interest
                        //cv::Rect myROI(min_x, min_y, width, height);

                        // Crop the full image to that image contained by the rectangle myROI
                        // Note that this doesn't copy the data
                        //cv::Mat croppedImage = interestingImage(myROI);
                        //imshow("mywindow5", croppedImage);
                        if (stoppedForPicturesCounter == 0)
                                stoppedForPicturesCounter = 5;
                        std::cout<<min_x << "," << max_x << "," << min_y << "," << max_y << "," << width << "," << height << std::endl;
                        interpolatedCoordinates(min_x, min_y, max_x, max_y, drawing.cols, drawing.rows);
                        std::cout<<min_x << "," << max_x << "," << min_y << "," << max_y << "," << width << "," << height << std::endl;
                        std::cout << "HD: " << frameHD->width << ", " << frameHD->height << '\n';
                        ctrl.stop();
                        ret = detectFeatures(min_x, min_y, max_x, max_y, frameHD);
                
                }
        }
        
        Mat& drawingScaled = drawing;
        //Mat drawingScaled = Mat::zeros( Size(CAMERA_WIDTH, CAMERA_HEIGHT), CV_8UC1 ); 
        //cv::resize(drawing, drawingScaled, Size(CAMERA_WIDTH, CAMERA_HEIGHT), 0, 0, INTER_NEAREST);
        imshow("mywindow2", drawingScaled);
        std::memset(boxVec, 0, sizeof(int));
        
        //std::cout << "Drawing image: \n";


        //int boxVecTemp[drawing.cols];
        std::memset(boxVec, 0, sizeof(int)*drawingScaled.cols);
        //std::cout << "rc: " << drawingScaled.rows << ", " << drawingScaled.cols << '\n';
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
                                //std::cout << 1;
                        } 
                        //else { std::cout << 0; }
                }
                //std::cout << '\n';

        }
        
        
        //std::cout << '\n';

        //std::cout << "box: ";
        for (size_t i = 0; i < drawingScaled.cols; ++i)
        {
                //if (boxVec[i] > 0) 
                //    boxVec[i] = 1;
                //std::cout << boxVec[i];    
        }
        //std::cout << '\n';
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
        usleep(1000);
        ctrl.rotateOnSpot();
        usleep(4000);
}

struct BoxHistory
{
        BoxHistory()
        {
                std::memset(history, 0, sizeof(bool)*boxHistorySize);
                boxHistoryIndex = 0;
        }
        void update(bool value)
        {
                history[boxHistoryIndex] = value;
                boxHistoryIndex++;
                boxHistoryIndex %= boxHistorySize;
        }
        
        bool movingTowardsBox()
        {
                
                size_t movingTowardsBoxSum = 0;
                for (size_t i = 0; i < boxHistorySize; ++i)
                {
                        if (history[i]) movingTowardsBoxSum += 1;
                }
                //std::cout << "--- Box detected over the last " << movingTowardsBoxSum << " frames.\n";
                if (movingTowardsBoxSum >= movingTowardsBoxThreshold)
                {
                        //std::cout << "MOVING TOWARDS BOX!!\n";
                        return true;
                }
                else 
                {
                        return false;  
                }
        }
        static size_t const boxHistorySize = 20;
        bool history[boxHistorySize]; 
        size_t boxHistoryIndex;
        static size_t const movingTowardsBoxThreshold = 12;
};

void run(bool* moveable, int* odv, IplImage* img, Controller& ctrl, BoxHistory& boxHistory, IplImage* normalCapture, IplImage* hdCapture, int* boxVec)
{
        const int freeSpaceThreshold = 50;
        const int leftThreshold = 25;
        const int middleThreshold = 40;
        const int rightThreshold = 35;
        
        const int IRThreshold = 300;
        const int WhiskerThreshold = 500;
        
        
        

        
        

        for (size_t i = 0; i < img->width; ++i)
        {
                if (odv[i] <= freeSpaceThreshold)
                {
                        moveable[i] = false;
                        //std::cout << '0';
                }
                else
                {
                        moveable[i] = true;
                        //std::cout << '1';
                }
        }
        //std::cout << '\n';

        // BOX DETECTION!!
        bool detectedCorrectBox = detectBoxes(normalCapture, hdCapture, boxVec);        
        std::pair<size_t, size_t> boxLine = longestLine(boxVec, img->width);
        bool moveTowardsBox;
        int minBoxDistance = img->height;
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
        
        //std::cout << "boxline: " << boxLine.first << ' '<< boxLine.second << '\n';

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
        //std::cout << "begin: " << beginMax << " end: " << endMax << '\n';
        //cvShowImage( "mywindow2", img );
        
        int lowestY = img->height;
        for (size_t i = beginMax; i <= endMax; ++i)
        {
                if (odv[i] < lowestY)
                        lowestY = odv[i];
        }
        
        
        int goal_dist = (beginMax + endMax) / 2;
        int bottom_dist = goal_dist - (img->width / 2) + CENTER_OFFSET;
       
        //std::cout << "lowestY: " << lowestY << ", bottom_dist: " << bottom_dist << ", img->width: " << img->width << '\n';
        double result = 0;
        if (moveTowardsBox)
        {
                boxHistory.update(true);
                int line_mid = (boxLine.first + boxLine.second) / 2;
                int bottom_box_dist = line_mid - (img->width / 2) + CENTER_OFFSET;
                double tanbox = (double)minBoxDistance / (double)bottom_box_dist;
        
                result = std::atan(tanbox);
                //std::cout << "BOX LOCATED!!! The arc tangent is: " <<  (result * 180 / PI) << '\n';
                result = result * 180 / PI;
        }
        else if (bottom_dist > 0) 
        {
                if (boxHistory.movingTowardsBox())
                {
                        result = 0;
                }
                else
                {
                        boxHistory.update(false);
                        double tan = (double)lowestY / (double)bottom_dist;
                
                        result = std::atan(tan);
                        //std::cout << "The arc tangent is: " <<  (result * 180 / PI) << '\n';
                        result = result * 180 / PI;
                }        
        }
        
        int leftIR = ctrl.getIRLeftValue();
        int rightIR = ctrl.getIRRightValue();
        int leftWhisker = ctrl.getWhiskerLeftValue();
        int rightWhisker = ctrl.getWhiskerRightValue();
        
        //std::cout << "W: " << leftWhisker << " | " << rightWhisker << ", IR: " << leftIR << " | " << rightIR << '\n';
        
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
        if (leftWhisker > WhiskerThreshold)
        {
                if (boxHistory.movingTowardsBox())
                {
                        stopAndRotate(ctrl);
                }
                else
                {
                        std::cout << "Moving back: left whisker\n";
                        moveBack = true;
                }
                        
                
        }
        if (rightWhisker > WhiskerThreshold)
        {
                if (boxHistory.movingTowardsBox())
                {
                        stopAndRotate(ctrl);
                }
                else
                {
                        moveBack = true;
                        std::cout << "Moving back: right whisker\n";
                }        
        }
        
        if (detectedCorrectBox)
        {
                stopAndRotate(ctrl);
                return;
        }
        
        if (moveBack)
        {
                moveBackConsideringFreeSpace(img, odv, ctrl);
                boxHistory.update(false);
                return;
                
        }/* else {ctrl.stop();}*/
        
        if(distanceMax < 60){
           result = 90;
        }
        std::cout << ">>> Angle: " << result << '\n';
        
        if (stoppedForPicturesCounter == 0)
                ctrl.turn(result);
}

std::vector< std::vector<cv::KeyPoint>* > sift_keypoints;
std::vector< cv::Mat > sift_descriptors;
cv::Mat sift_images;
cv::SiftFeatureDetector detector;
cv::SiftDescriptorExtractor extractor;
std::vector<std::string> image_names;     

void initSift()
{

        image_names.push_back("walle.png");
        /*image_names.push_back("ferrari.png");
        image_names.push_back("celebes.png");
        image_names.push_back("fry.png");
        image_names.push_back("mario.png");
        image_names.push_back("terminator.png");
        image_names.push_back("iron.png");
        image_names.push_back("starry.png");
        image_names.push_back("thor.png");*/


        for (size_t i = 0; i < image_names.size(); ++i)
        {
                //std::cout << "hey! :D\n";
                cv::Mat input = cv::imread(image_names[i], 0); //Load as grayscale                
                //std::cout << "hey! :D\n";
                std::vector<cv::KeyPoint>* keypoints_image = new std::vector<cv::KeyPoint>();
                detector.detect(input, *keypoints_image);
                
                cv::Mat descriptors_image;
                extractor.compute(input, *keypoints_image, descriptors_image);
                
                sift_images = input;
                sift_keypoints.push_back(keypoints_image);
                sift_descriptors.push_back(descriptors_image);
                //std::cout <<  "walle: " Controller ctrl;<< input << '\n';
                imshow("mywindow6", input);
        } 
        
        //std::cout << "KEYPOINTS:\n";    
        for (size_t i = 0 ; i < sift_keypoints[0]->size(); i++)
        {
                //std::cout << (*(sift_keypoints[0]))[i].pt << '\n';
        }

}

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
        
        if(min_x >= 640)
        {
            min_x = 640;   
        }

        if(max_x >= 640)
        {
            max_x = 640;   
        }
        
        if(min_y >= 480)
        {
            min_y = 480;   
        }
        
        if(max_y >= 480)
        {
            max_y = 480;   
        }
        
        if(min_y + height >= 480)
        {
            height = 480 - min_y;   
        }
        
        if(min_x + width >= 640)
        {
            width = 640 - min_x;   
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
        
        cv::Mat outputCam;
        cv::drawKeypoints(croppedImageGray, keypoints_camera, outputCam);
        imshow("mywindow5", outputCam);
        
        //std::cout << "KEYPOINTS:\n";    
        for (size_t i = 0 ; i < sift_keypoints[0]->size(); i++)
        {
                //std::cout << (*(sift_keypoints[0]))[i].pt << '\n';
        }
        /*
        cv::Mat walleOut;
        cv::drawKeypoints(sift_images, sift_keypoints[0], walleOut);
        imshow("mywindow6", walleOut);*/
        
        
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
                        break;
                }
        }
        

        

        //cv::Mat outputCam(croppedImage.size(), croppedImage.channels());
        //cv::drawKeypoints(cameraMat, matching, outputCam);
        //cv::imwrite("ck.png", outputCam);
        //imshow("mywindow2", outputCam);
        //imshow("mywindow", outputCam);
        //cvShowImage( "mywindow2", camera );
        
        //cvReleaseImage(&camera);

        // Do not release the frame!
        //If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
        //remove higher bits using AND operator
        //if ( (cvWaitKey(10) & 255) == 27 ) break;
        
        keypoints_camera.clear();
        //keypoints_camera.clear();
        //sleep(1);
        matching.clear();
        return ret;


        //cv::drawKeypoints(input, keypoints_image, output);


/*
        cvReleaseCapture( &capture );
        
        usleep(10);     
        
        capture = cvCaptureFromCAM( CV_CAP_ANY );
        
        cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH );
        cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT );
*/
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
        matching.push_back(other_points[minkey]);
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
        
     
        int temp = 20;

        capture = cvCaptureFromCAM( CV_CAP_ANY );
        
//hdCapture
        
        cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_WIDTH, 640 );
        cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_HEIGHT, 480 );

        //CvCapture* hdCapture = cvCaptureFromCAM( CV_CAP_ANY );
        //cvSetCaptureProperty( hdCapture, CV_CAP_PROP_FRAME_WIDTH, 352 );
        //cvSetCaptureProperty( hdCapture, CV_CAP_PROP_FRAME_HEIGHT, 288 );
        
        usleep(10);
        
        if ( !capture ) {
                fprintf( stderr, "ERROR: capture is NULL \n" );
                getchar();
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
        //cvNamedWindow( "preBoxDetection", CV_WINDOW_AUTOSIZE );
        //cvNamedWindow( "preBoxDetectionsssssss", CV_WINDOW_AUTOSIZE );
        // Show the image captured from the camera in the window and repeat
        
        int odv[CAMERA_WIDTH];
        int boxVec[CAMERA_WIDTH];
        for (size_t i = 0; i < CAMERA_WIDTH; ++i)
        {
          odv[i] = 0;
        }
        BoxHistory boxHistory;
        //cvCreateTrackbar( "SwitchA", "mywindow2", &a, 300, NULL );
        //cvCreateTrackbar( "SwitchB", "mywindow2", &b, 300, NULL );
        //cvCreateTrackbar( "SwitchC", "mywindow2", &c, 20, NULL );
      //  cvCreateTrackbar( "SwitchD", "mywindow2", &d, 300, NULL );
        while ( 1 ) {
                // Get one frame
                //cvQueryFrame( capture );
                IplImage* orig = NULL;
           
                cvGrabFrame(capture);
                cvGrabFrame(capture);
                orig = cvRetrieveFrame( capture );
                IplImage* orig_small = cvCreateImage( cvSize(CAMERA_WIDTH, CAMERA_HEIGHT), orig->depth, orig->nChannels);
                //std::cout << "0HD: " << orig->width << ", " << orig->height << '\n';
                cvResize(orig, orig_small);
                //std::cout << "1HD: " << orig->width << ", " << orig->height << '\n';
                //IplImage* orig = cvCreateImage(cvSize(orig->width,orig->height),orig->depth, orig->nChannels); 
                //cvCopy( realOrig, orig, NULL );
                
                IplImage* frame = cvCreateImage(cvSize(orig_small->width,orig_small->height),IPL_DEPTH_8U, orig_small->nChannels);                 
                IplImage* dst = cvCreateImage(cvSize(frame->width,frame->height),IPL_DEPTH_8U, frame->nChannels); 
                
                
                //IplImage *gray = cvCreateImage( cvSize( frame->width, frame->height ), IPL_DEPTH_8U, 1 );
                //cvCvtColor( frame, gray, CV_RGB2GRAY );
                //cvShowImage( "mywindow3", gray );
                
                //cvCvtColor(frame, frame, CV_BGR2HSV);
                cvSmooth(orig_small, frame, CV_GAUSSIAN, 3, 3);
                cvDilate(frame, frame, NULL, 5);
                
                
                //cvShowImage( "mywindow", frame );
                
                
                //cvCopy( frame, dst, NULL );
                
                if ( !frame ) {
                        fprintf( stderr, "ERROR: frame is null...\n" );
                        getchar();
                        break;
                }
                cvShowImage( "mywindow4", orig_small );
                cvShowImage( "mywindow3", frame );
                int colors[3] = {0, 0, 0};
                
                if(temp == 0){
                        calcHist(frame, colors);
                }
                else
                {
                        temp -= 1;
                }
                for (size_t i = 0; i < CAMERA_WIDTH; ++i)
                {
                        odv[i] = 0;
                }
                isGround(frame, dst, colors, odv);
                //double angle = getFreeSpaceAngle(moveable, odv, dst);
                
                //std::cout << "3HD: " << orig->width << ", " << orig->height << '\n';
                run(moveable, odv, dst, ctrl, boxHistory, orig_small, orig, boxVec);
                //std::cout << "ANGLE: " << angle << '\n';
                //ctrl.turn(angle);
                //std::cout << "max r: " << colors[0] << " g: " << colors[1] << " b: " << colors[2] << '\n';
                cvReleaseImage(&frame);
                cvReleaseImage(&dst);
                //cvReleaseImage(&orig);
                
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
        //cvDestroyWindow( "preBoxDetection" );
        return 0;
}
