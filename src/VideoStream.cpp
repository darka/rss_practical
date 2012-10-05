#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

int main(int argc, char** argv) {

    /* Create a window */
    cvNamedWindow("VideoStream", CV_WINDOW_AUTOSIZE);
    /* capture frame from video file */
    CvCapture* capture = cvCaptureFromCAM(CV_CAP_ANY);
    cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_WIDTH, 640 );
    cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_HEIGHT, 480 );
    
    /* Create IplImage to point to each frame */
    IplImage* frame;
    /* Get fps, needed to set the delay */
    int fps = ( int )cvGetCaptureProperty( capture, CV_CAP_PROP_FPS );
    /* Loop until frame ended or ESC is pressed */
    while(1) {
        /* grab frame image, and retrieve */
        frame = cvQueryFrame(capture);
        cv::Mat matFrame(frame,false);
        if(!frame) break;
        /* display frame into window */
        cvShowImage("VideoStream", frame);
        /* Set correct stream delay */

    }
    /* destroy pointer to video */
    cvReleaseCapture(&capture);
    /* delete window */
    cvDestroyWindow("VideoStream");
    return 1;
}
