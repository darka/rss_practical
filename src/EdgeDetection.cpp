#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <sstream>
#include <iostream>
#include <stdlib.h>
#include <stdio.h>

using namespace cv;


const int CAMERA_WIDTH = 640;
const int CAMERA_HEIGHT = 480;
int main( int argc, char** argv )
{

        //CvCapture* capture2 = cvCaptureFromCAM( CV_CAP_ANY );
        CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );

        std::cout << "testa\n";        
        cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH );
        cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT );
        //cvSetCaptureProperty( capture2, CV_CAP_PROP_FRAME_WIDTH, 180 );
        //cvSetCaptureProperty( capture2, CV_CAP_PROP_FRAME_HEIGHT, 135 );
        //CvCapture* hdCapture = cvCaptureFromCAM( CV_CAP_ANY );
        //cvSetCaptureProperty( hdCapture, CV_CAP_PROP_FRAME_WIDTH, 352 );
        //cvSetCaptureProperty( hdCapture, CV_CAP_PROP_FRAME_HEIGHT, 288 );
        
        if ( !capture ) {
                fprintf( stderr, "ERROR: capture is NULL \n" );
                getchar();
                return -1;
        }
        // Create a window in which the captured images will be presented
        cvNamedWindow( "mywindow", CV_WINDOW_AUTOSIZE );
        cvNamedWindow( "mywindow2", CV_WINDOW_AUTOSIZE );
        std::cout << "test0\n";
        IplImage* frame = cvQueryFrame(capture);
        //IplImage* frame2 = cvQueryFrame(capture2);
        std::string filename_base = "picture";
        std::cout << "test1\n";
        //cvShowImage( "mywindow", frame );
//        cvShowImage( "mywindow2", frame2 );

        for (int i = 0; i < 20; ++i)
        {
                frame = cvQueryFrame(capture);
                //frame = cvCreateImage(cvSize(tmpFrame->width,tmpFrame->height),tmpFrame->depth, tmpFrame->nChannels); 
                //cvCopy(tmpFrame, frame);
                cvShowImage( "mywindow", frame );
                std::cout <<"hello!!!\n";
                if ( (cvWaitKey(10) & 255) == 27 ) break;
        }
        
        cvReleaseCapture(&capture);
        usleep(10);
        capture = cvCaptureFromCAM( CV_CAP_ANY );
        cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_WIDTH, 180 );
        cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_HEIGHT, 135 );   
        
        for (int i = 0; i < 120; ++i)
        {       
                
                  
                IplImage* frame2 = cvQueryFrame(capture);

                cvShowImage( "mywindow2", frame2 );
                if ( (cvWaitKey(10) & 255) == 27 ) break;
                
        }
        cvReleaseCapture(&capture);
        usleep(10);
        std::cout << "6\n";
        capture = cvCaptureFromCAM( CV_CAP_ANY );
        cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH );
        cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT );
        for (int i = 0; i < 20; ++i)
        {
                frame = cvQueryFrame(capture);
                //frame = cvCreateImage(cvSize(tmpFrame->width,tmpFrame->height),tmpFrame->depth, tmpFrame->nChannels); 
                //cvCopy(tmpFrame, frame);
                cvShowImage( "mywindow", frame );
                std::cout <<"hello!!!\n";
                if ( (cvWaitKey(10) & 255) == 27 ) break;
        }
        cvReleaseCapture(&capture);
        //c
  
  }
