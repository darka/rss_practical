// The "Square Detector" program.
// It loads several images sequentially and tries to find squares in
// each image

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>

#include <iostream>
#include <math.h>
#include <string.h>

using namespace cv;
using namespace std;

void help()
{
        cout <<
        "\nA program using pyramid scaling, Canny, contours, contour simpification and\n"
        "memory storage (it's got it all folks) to find\n"
        "squares in a list of images pic1-6.png\n"
        "Returns sequence of squares detected on the image.\n"
        "the sequence is stored in the specified memory storage\n"
        "Call:\n"
        "./squares\n"
    "Using OpenCV version %s\n" << CV_VERSION << "\n" << endl;
}


int thresh = 50, N = 1; // karlphillip: decreased N to 2, was 11.
const char* wndname = "Square Detection Demo";

// helper function:
// finds a cosine of angle between vectors
// from pt0->pt1 and from pt0->pt2
double angle( Point pt1, Point pt2, Point pt0 )
{
    double dx1 = pt1.x - pt0.x;
    double dy1 = pt1.y - pt0.y;
    double dx2 = pt2.x - pt0.x;
    double dy2 = pt2.y - pt0.y;
    return (dx1*dx2 + dy1*dy2)/sqrt((dx1*dx1 + dy1*dy1)*(dx2*dx2 + dy2*dy2) + 1e-10);
}

int ct1 = 5;
int ct2 = 200;
int app = 50;
// returns sequence of squares detected on the image.
// the sequence is stored in the specified memory storage
void findSquares( IplImage* frame)
{

    IplImage* gray2;
    gray2 = cvCreateImage(cvSize(frame->width, frame->height), IPL_DEPTH_8U, 1);
    std::cout << "gray2\n";    
  //  cvtColor( image, gray0, CV_BGR2GRAY );
    cvCvtColor(frame,gray2,CV_BGR2GRAY);
    
    CvScalar avg;
    CvScalar avgStd;
    cvAvgSdv(gray2, &avg, &avgStd, NULL);
    std::cout << "test\n";    
    cvSmooth(gray2, gray2, CV_GAUSSIAN, 3, 3);
    cvThreshold(gray2, gray2, (int)avg.val[0]-7*(int)(avgStd.val[0]/8), 255, CV_THRESH_BINARY_INV);
    
    cvErode(gray2, gray2, NULL,1); 

    cvDilate(gray2, gray2, NULL, 2);
    std::cout << "gray0\n";
    Mat gray0(gray2, false);
    std::cout << "gray1\n";
    // karlphillip: dilate the image so this technique can detect the white square,
    Mat gray1;
    Canny( gray0, gray1, ct1, ct2, 3 );
    //imshow(wndname, gray1);
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    
    findContours(gray1, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
    Mat drawing = Mat::zeros( gray1.size(), CV_8UC3 );    

    vector<vector<Point> > squares;
        
    vector<Point> approx;
    for (size_t i = 0; i < contours.size(); ++i)
    {
                approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*(1.0/app), true);
                squares.push_back(approx);
    }


        

    int idx = 0;
    for( ; idx >= 0; idx = hierarchy[idx][0] )
    {
        //double area = contourArea(Mat(squares[idx]));
        //if(area > 300)
        //{
        //std::cout << area << std::endl;
        //vector <Point> contour = squares[i];
        drawContours(gray1, squares, idx, Scalar(255, 0, 0), CV_FILLED, 8, hierarchy);
        //drawContours(image, squares, -1, Scalar(255, 0, 0));
        //}

    }

    imshow(wndname, gray1);
}

Mat edgeDetection(IplImage* frame)
{

        Mat src(frame);
        Mat src_gray;
        cvtColor( src, src_gray, CV_BGR2GRAY );
        Mat dst, detected_edges;

        int lowThreshold = 95;
        int ratio = 3;
        int kernel_size = 3;

        /// Reduce noise with a kernel 3x3
        blur( src_gray, detected_edges, Size(3,3) );

        /// Canny detector
        Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

        /// Using Canny's output as a mask, we display our result
        dst = Scalar::all(0);

        src.copyTo( dst, detected_edges);
        imshow( wndname, dst );
        
        return dst;

}

int main(int argc, char** argv) {

        CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
        cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_WIDTH, 352 );
        cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_HEIGHT, 288 );
        
        if ( !capture ) {
                fprintf( stderr, "ERROR: capture is NULL \n" );
                getchar();
                return -1;
        }
        // Create a window in which the captured images will be presented
        cvNamedWindow( "mywindow", CV_WINDOW_AUTOSIZE );
        cvNamedWindow( wndname, CV_WINDOW_AUTOSIZE );
        createTrackbar("T1", wndname, &ct1, 200);
        createTrackbar("T2", wndname, &ct2, 200);
        createTrackbar("APP", wndname, &app, 100);
        
        while ( 1 ) {
                // Get one frame
                IplImage* frame = cvQueryFrame( capture );
                //Mat image(frame, false);
                

                if ( !frame ) {
                        fprintf( stderr, "ERROR: frame is null...\n" );
                        getchar();
                        break;
                }
                
                ///Mat filtered = edgeDetection(frame);
                
                findSquares( frame );
                //cvShowImage( "mywindow", frame );

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
