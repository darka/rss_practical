#include "Controller.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

CvCapture* startCamera()
{
        CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
        cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_WIDTH, 256 );
        cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_HEIGHT, 192 );
        
        // Create a window in which the captured images will be presented
        
        // Show the image captured from the camera in the window and repeat
        return capture;
}

int main()
{

        Controller c;
        
        // Basic movement test

        /*c.moveForward();
        sleep(2);

        c.turnLeft();
        sleep(2);

        c.turnRight();
        sleep(2);
        
        CvCapture* capture = startCamera();
        cvNamedWindow( "mywindow", CV_WINDOW_AUTOSIZE );

        // Do not release the frame!
        //If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
        //remove higher bits using AND operator
        while ( 1 ) {
                // Get one frame
                IplImage* frame = cvQueryFrame( capture );
                if ( !frame ) {
                        fprintf( stderr, "ERROR: frame is null...\n" );
                        getchar();
                        break;
                }
                cvShowImage( "mywindow", frame );
                // Do not release the frame!
                //If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
                //remove higher bits using AND operator
                if ( (cvWaitKey(10) & 255) == 27 ) break;
        }
        // Release the capture device housekeeping
        cvReleaseCapture( &capture );
        cvDestroyWindow( "mywindow" );*/
        
        c.turn(90);
        sleep(4);
        c.turn(-90);
        sleep(4);
        c.turn(60);
        sleep(4);

        c.stop();
}
