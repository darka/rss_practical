#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>

using namespace cv;

/// Global variables

Mat src, src_gray;
Mat dst, detected_edges;

int edgeThresh = 1;
int lowThreshold;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
char* window_name = "Edge Map";

/**
 * @function CannyThreshold
 * @brief Trackbar callback - Canny thresholds input with a ratio 1:3
 */
void CannyThreshold(int, void*)
{
  /// Reduce noise with a kernel 3x3
  blur( src_gray, detected_edges, Size(3,3) );

  /// Canny detector
  Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

  /// Using Canny's output as a mask, we display our result
  dst = Scalar::all(0);

  src.copyTo( dst, detected_edges);
  imshow( window_name, dst );
 }


/** @function main */
int main( int argc, char** argv )
{
  /// Load an image

    CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);
    
    cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_WIDTH, 640 );

    cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_HEIGHT, 480 );
    
    /* Create IplImage to point to each frame */
    IplImage* frame;
    /* Get fps, needed to set the delay */
    int fps = ( int )cvGetCaptureProperty( capture, CV_CAP_PROP_FPS );
    /* Loop until frame ended or ESC is pressed */
    
    namedWindow( window_name, CV_WINDOW_AUTOSIZE );
            
    while(1) {
        /* grab frame image, and retrieve */
        frame = cvQueryFrame(capture);
        cv::Mat matFrame(frame,false);
        src = matFrame;

        if( !src.data )
        { return -1; }

        /// Create a matrix of the same type and size as src (for dst)
        dst.create( src.size(), src.type() );

        /// Convert the image to grayscale
        cvtColor( src, src_gray, CV_BGR2GRAY );

        /// Create a Trackbar for user to enter threshold
        createTrackbar( "Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold );

        /// Show the image
        CannyThreshold(0, 0);

        /// Wait until user exit program by pressing a key
        waitKey(0);
        }

  return 0;
  }