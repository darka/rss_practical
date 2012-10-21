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

const int CAMERA_WIDTH = 180;
const int CAMERA_HEIGHT = 135;
const int CENTER_OFFSET = 0;

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

        
        std::cout << "Drawing image: \n";
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
                                std::cout << '1';

                        } else 
                        { 
                                std::cout << '0'; 
                                if (odv[k] < i)
                                {
                                        odv[k] = i;
                                }  
                       }
                      
                }
                std::cout << '\n';
        }
        std::cout << "Done!\n";
        std::cout << "odv: ";
        for (size_t i = 0; i < CAMERA_WIDTH; ++i)
        {
          std::cout << odv[i] << ' ';
        }
        std::cout << '\n';
        for (size_t i = 0; i < CAMERA_WIDTH; ++i)
        {
          odv[i] = grid->height - odv[i];
        }
        
        std::cout << "origin: " << src->origin << '\n';

        
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

void run(bool* moveable, int* odv, IplImage* img, Controller& ctrl)
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
                        std::cout << '0';
                }
                else
                {
                        moveable[i] = true;
                        std::cout << '1';
                }

        }
        std::cout << '\n';        

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
        std::cout << "begin: " << beginMax << " end: " << endMax << '\n';
        //cvShowImage( "mywindow2", img );
        
        int lowestY = img->height;
        for (size_t i = beginMax; i <= endMax; ++i)
        {
                if (odv[i] < lowestY)
                        lowestY = odv[i];
        }
        
        
        int goal_dist = (beginMax + endMax) / 2;
        int bottom_dist = goal_dist - (img->width / 2) + CENTER_OFFSET;
       
        std::cout << "lowestY: " << lowestY << ", bottom_dist: " << bottom_dist << ", img->width: " << img->width << '\n';
        double result = 0;
        if (bottom_dist > 0) {
                double tan = (double)lowestY / (double)bottom_dist;
        
                result = std::atan(tan);
                std::cout << "The arc tangent is: " <<  (result * 180 / PI) << '\n';
                result = result * 180 / PI;
        }
        
        int leftIR = ctrl.getIRLeftValue();
        int rightIR = ctrl.getIRRightValue();
        int leftWhisker = ctrl.getWhiskerLeftValue();
        int rightWhisker = ctrl.getWhiskerRightValue();
        
        std::cout << "W: " << leftWhisker << " | " << rightWhisker << ", IR: " << leftIR << " | " << rightIR << '\n';
        
        if (distanceMax < 10 || leftIR > IRThreshold || rightIR > IRThreshold || leftWhisker > WhiskerThreshold || rightWhisker > WhiskerThreshold) {
                moveBackConsideringFreeSpace(img, odv, ctrl);
                return;
                
        }/* else {ctrl.stop();}*/
        
        if(distanceMax < 60){
           result = 90;
        }
        std::cout << ">>> Angle: " << result << '\n';
        
        ctrl.turn(result);
}

int main(int argc, char** argv) {

        Controller ctrl;
        bool moveable[CAMERA_WIDTH];
        for (size_t i = 0; i < CAMERA_WIDTH; ++i)
        {
          moveable[i] = false;
        }
        
        
        
        int temp = 20;

        CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
        

        
        cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH );
        cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT );
        
        if ( !capture ) {
                fprintf( stderr, "ERROR: capture is NULL \n" );
                getchar();
                return -1;
        }
        // Create a window in which the captured images will be presented
        cvNamedWindow( "mywindow", CV_WINDOW_AUTOSIZE );
        //cvNamedWindow( "mywindow2", CV_WINDOW_AUTOSIZE );
        //cvNamedWindow( "mywindow3", CV_WINDOW_AUTOSIZE );
        // Show the image captured from the camera in the window and repeat
        
        int odv[CAMERA_WIDTH];
        for (size_t i = 0; i < CAMERA_WIDTH; ++i)
        {
          odv[i] = 0;
        }
        
        while ( 1 ) {
                // Get one frame
                //cvQueryFrame( capture );
                IplImage* frame = cvQueryFrame( capture );
                
                IplImage* dst = cvCreateImage(cvSize(frame->width,frame->height),IPL_DEPTH_8U, frame->nChannels); 
                
                
                //IplImage *gray = cvCreateImage( cvSize( frame->width, frame->height ), IPL_DEPTH_8U, 1 );
                //cvCvtColor( frame, gray, CV_RGB2GRAY );
                //cvShowImage( "mywindow3", gray );
                
                //cvCvtColor(frame, frame, CV_BGR2HSV);
                cvSmooth(frame, frame, CV_GAUSSIAN, 3, 3);
                cvDilate(frame, frame, NULL, 5);
                
                
                //cvShowImage( "mywindow", frame );
                
                
                //cvCopy( frame, dst, NULL );
                
                if ( !frame ) {
                        fprintf( stderr, "ERROR: frame is null...\n" );
                        getchar();
                        break;
                }
                //cvShowImage( "mywindow", frame );
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
                run(moveable, odv, dst, ctrl);
                //std::cout << "ANGLE: " << angle << '\n';
                //ctrl.turn(angle);
                //std::cout << "max r: " << colors[0] << " g: " << colors[1] << " b: " << colors[2] << '\n';
                cvReleaseImage(&dst);

                // Do not release the frame!
                //If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
                //remove higher bits using AND operator
                if ( (cvWaitKey(10) & 255) == 27 ) break;
        }
        // Release the capture device housekeeping
        cvReleaseCapture( &capture );
        //cvDestroyWindow( "mywindow" );
        //cvDestroyWindow( "mywindow2" );
        return 0;
}
