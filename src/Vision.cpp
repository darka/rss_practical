#include "Vision.hpp"
#include <algorithm>

using namespace cv;

CvCapture* Vision::capture = NULL;
BASE_TYPE Vision::baseType = BASE_QUEEN;
IplImage* Vision::orig = NULL;
IplImage* Vision::orig_small = NULL;
IplImage* Vision::detected_floor = NULL;
bool Vision::origReady = false;
bool Vision::runBaseDetection = false;
bool Vision::releaseBox = false;
int Vision::baseCenterX = 0;
int Vision::baseCenterY = 0;
IplImage* Vision::src_base = cvLoadImage("base2.png");

CvHistogram* Vision::hist_hue_ground = NULL;
CvHistogram* Vision::hist_sat_ground = NULL;
CvHistogram* Vision::hist_val_ground = NULL;

const char* Vision::hist_window_1 = "hist1";
const char* Vision::hist_window_2 = "hist2";
const char* Vision::hist_window_3 = "hist3";
    
const float Vision::queenHist[] = {
 0.5, 0.75, 1.0, 1.0, 1.25, 1.75, 1.5, 1.75, 2.25, 2.25, 2.25, 1.75, 1.75, 1.5, 1.5, 1.0, 1.0, 1.0, 1.25, 1.0, 0.75, 0.75, 0.75, 0.5, 0.0, 0.0, 0.5, 0.75, 0.5, 0.25, 0.5, 0.5, 0.5, 0.5, 0.75, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.75, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.25, 1.25, 1.5, 2.0, 2.25, 2.75, 3.5, 4.0, 4.5, 5.5, 5.75, 6.5, 7.75, 8.5, 10.0, 10.0, 10.75, 11.25, 12.75, 13.5, 13.75, 13.75, 12.75, 13.0, 12.25, 12.25, 11.5, 11.25, 11.25, 11.5, 11.5, 11.5, 12.0, 12.25, 11.5, 12.25, 12.25, 12.75, 12.5, 12.0, 11.0, 11.0, 11.25, 11.0, 12.0, 12.25, 12.75, 14.5, 15.0, 15.75, 17.0, 17.75, 17.25, 18.25, 19.75, 22.0, 24.25, 25.5, 27.25, 28.5, 30.25, 31.5, 33.25, 35.75, 38.25, 39.25, 39.75, 43.25, 49.25, 49.75, 51.25, 52.75, 55.0, 50.25, 50.5, 44.0, 41.0, 36.5, 34.0, 30.75, 28.75, 25.75, 24.25, 21.5, 20.5, 18.25, 17.0, 15.75, 14.25, 14.0, 12.5, 10.75, 8.75, 8.5, 7.25, 6.5, 5.75, 5.5, 5.25, 5.5, 5.5, 4.25, 3.75, 3.0, 2.5, 2.25, 1.75, 1.75, 2.0, 1.75, 2.0, 2.0, 1.75, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 1.75, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.25, 2.0, 2.25, 2.25, 2.0, 1.75, 2.0, 2.0, 2.0, 1.75, 1.75, 2.0, 2.0, 2.5, 1.75, 2.25, 2.25, 2.25, 2.0, 2.25, 2.0, 1.75, 2.0, 2.0, 2.0, 1.75, 1.75, 1.5, 1.5, 1.5, 1.5, 1.5, 1.75, 1.5, 1.5, 1.0, 1.5, 1.25, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.25, 1.0, 1.0, 1.0, 1.0, 0.75, 0.75, 0.75, 0.75, 0.75, 0.5, 0.5, 0.5, 0.5, 0.5, 0.25, 0.5 };

Vision::Vision()
: saved_angle(-1)
, correct_box(false)
{
        
        initSift();
        calcHistGround();

	//printf("Starting threads...\n");
        
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
                //cvNamedWindow( "mywindow4", CV_WINDOW_AUTOSIZE );
                //cvNamedWindow( "mywindow5", CV_WINDOW_AUTOSIZE );
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
                //cvMoveWindow("mywindow2", 400, 20);
                cvMoveWindow("mywindow3", 800, 20);
                //cvMoveWindow("mywindow3", 0, 320);
                //cvMoveWindow("mywindow4", 400, 320);
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

int Vision::calcHist(IplImage* img, const char* window_name)
{
        /* Always check if the program can find a file */
        if( !img )
        return -1;


        IplImage* img_hsv = cvCreateImage( cvGetSize(img), img->depth, img->nChannels );

        IplImage* channel = cvCreateImage( cvGetSize(img), 8, 1 );
        cvCvtColor(img,img_hsv,CV_BGR2HSV);

        IplImage *hist_img_hue = cvCreateImage(cvSize(CAMERA_WIDTH,CAMERA_HEIGHT), 8, 3);
        IplImage *hist_img_sat = cvCreateImage(cvSize(CAMERA_WIDTH,CAMERA_HEIGHT), 8, 3);
        IplImage *hist_img_val = cvCreateImage(cvSize(CAMERA_WIDTH,CAMERA_HEIGHT), 8, 3);
        cvSet( hist_img_hue, cvScalarAll(255), 0 );
        cvSet( hist_img_sat, cvScalarAll(255), 0 );
        cvSet( hist_img_val, cvScalarAll(255), 0 );

        CvHistogram *hist_hue;
        CvHistogram *hist_sat;
        CvHistogram *hist_val;
        
        int hist_size = 256;      
        float range[]={0,256};
        float* ranges[] = { range };

        float max_value = 0.0;
        float max = 0.0;
        float w_scale = 0.0;

        /* Create a 1-D Arrays to hold the histograms */
        hist_hue = cvCreateHist(1, &hist_size, CV_HIST_ARRAY, ranges, 1);
        hist_sat = cvCreateHist(1, &hist_size, CV_HIST_ARRAY, ranges, 1);
        hist_val = cvCreateHist(1, &hist_size, CV_HIST_ARRAY, ranges, 1);

        /* Set image to obtain RED as Channel of Interest (COI) */
        cvSetImageCOI(img,1);
        cvCopy(img,channel);
        cvResetImageROI(img);

        /* Calculate histogram of the Image and store it in the array */
        cvCalcHist( &channel, hist_hue, 0, NULL );
        cvNormalizeHist(hist_hue, 1.0);
        //cvNormalize(ImageVal, ImageValNorm, 0, 255, CV_MINMAX);

        /* Calculate and Plot the histograms Green and Blue channels as well */
        /* Green channel */
        cvSetImageCOI(img,2);
        cvCopy(img,channel);
        cvResetImageROI(img);

        cvCalcHist( &channel, hist_sat, 0, NULL );
        cvNormalizeHist(hist_sat, 1.0);
        
        /* Blue channel */
        cvSetImageCOI(img,3);
        cvCopy(img,channel);
        cvResetImageROI(img);

        cvCalcHist( &channel, hist_val, 0, NULL );
        cvNormalizeHist(hist_val, 1.0);

        /* Find the minimum and maximum values of the histograms */
        cvGetMinMaxHistValue( hist_hue, 0, &max_value, 0, 0 );
        cvGetMinMaxHistValue( hist_sat, 0, &max, 0, 0 );

        max_value = (max > max_value) ? max : max_value;

        cvGetMinMaxHistValue( hist_val, 0, &max, 0, 0 );

        max_value = (max > max_value) ? max : max_value;    
        // The variable max_value has the maximum of the three histograms

        /* Using the maximum value, Scale/Squeeze the histogram (to fit the image) */
        cvScale( hist_hue->bins, hist_hue->bins, ((float)hist_img_hue->height)/max_value, 0 );
        cvScale( hist_sat->bins, hist_sat->bins, ((float)hist_img_sat->height)/max_value, 0 );
        cvScale( hist_val->bins, hist_val->bins, ((float)hist_img_val->height)/max_value, 0 );

        //printf("Scale: %4.2f pixels per 100 units\n", max_value*100/((float)hist_img->height));                         
           //A scale to estimate the number of pixels

        /* Scale/Squeeze the histogram range to image width */
        w_scale = ((float)hist_img_hue->width)/hist_size;
        
        int Rmax = 0;//hist_img->height+1;
        int Gmax = 0;//hist_img->height+1;
        int Bmax = 0;//hist_img->height+1;
        
        int rm = 0;
        int gm = 0;
        int bm = 0;

        /* Plot the Histograms */
        bool queenDetected = false;
        int sum = 0;
        for( int i = 1; i < hist_size; i++ )
        {
                      
                int a = std::max(cvRound(cvGetReal1D(hist_hue->bins,i) - cvGetReal1D(hist_hue_ground->bins,i)), 0);
                
                if (queenHist[i] > 5)
                {
                        float result = (float)a - queenHist[i];
                        sum += result;
                }        
                
                cvRectangle( hist_img_hue, cvPoint((int)i*w_scale , hist_img_hue->height),
                cvPoint((int)(i+1)*w_scale, hist_img_hue->height - a),
                CV_RGB(255,0,0), -1, 8, 0 );
                
                //std::cout << "x: " << ((int)(i+1)*w_scale) << ", y: " << (hist_img->height - cvRound(cvGetReal1D(hist_hue->bins,i))) <<'\n';
                int b = std::max(cvRound(cvGetReal1D(hist_sat->bins,i) - cvGetReal1D(hist_sat_ground->bins,i)), 0);
                cvRectangle( hist_img_sat, cvPoint((int)i*w_scale , hist_img_sat->height),
                cvPoint((int)(i+1)*w_scale, hist_img_sat->height - b),
                CV_RGB(0,255,0), -1, 8, 0 );
                
                int c = std::max(cvRound(cvGetReal1D(hist_val->bins,i) - cvGetReal1D(hist_val_ground->bins,i)), 0);
                cvRectangle( hist_img_val, cvPoint((int)i*w_scale , hist_img_val->height),
                cvPoint((int)(i+1)*w_scale, hist_img_val->height - c),
                CV_RGB(0,0,255), -1, 8, 0 );
        }
        //std::cout << '\n';
        
        std::cout << "queen: " << sum << '\n';
        
        if (queenDetected)
                std::cout << "Queen detected!\n";

        cvShowImage(hist_window_1, hist_img_hue);
        cvShowImage(hist_window_2, hist_img_sat);
        cvShowImage(hist_window_3, hist_img_val);
        
        /*
        cvNamedWindow( "Image", 1 );
        cvShowImage( "Image",img);



        cvWaitKey(0);

        cvDestroyWindow( "Image" );
        cvDestroyWindow( "Histogram" );
        cvReleaseImage( &img );
        */
        //std::cout << "max r: " << Rmax << " g: " << Gmax << " b: " << Bmax << '\n';
        //std::cout << "max r: " << rm << " g: " << gm << " b: " << bm << '\n';

        cvReleaseImage( &img_hsv );        
        cvReleaseImage( &hist_img_val );
        cvReleaseImage( &hist_img_sat );
        cvReleaseImage( &hist_img_hue );
        cvReleaseImage( &channel );
        return 0;
}


int Vision::calcHistGround()
{
        /* Always check if the program can find a file */


        std::cout << "hi! 1\n";
        IplImage* img = cvLoadImage("ground.png");
        if( !img )
        return -1;
        IplImage* img_hsv = cvCreateImage( cvGetSize(img), img->depth, img->nChannels );
        std::cout << "hi! 2\n";
        IplImage* channel = cvCreateImage( cvGetSize(img), 8, 1 );
        cvCvtColor(img,img_hsv,CV_BGR2HSV);
        std::cout << "hi! 3\n";
        IplImage *hist_img = cvCreateImage(cvSize(CAMERA_WIDTH,CAMERA_HEIGHT), 8, 3);
        cvSet( hist_img, cvScalarAll(255), 0 );
        std::cout << "hi! 4\n";
        
        int hist_size = 256;      
        float range[]={0,256};
        float* ranges[] = { range };

        float max_value = 0.0;
        float max = 0.0;
        float w_scale = 0.0;

        std::cout << "hi! 5\n";

        /* Create a 1-D Arrays to hold the histograms */
        hist_hue_ground = cvCreateHist(1, &hist_size, CV_HIST_ARRAY, ranges, 1);
        hist_sat_ground = cvCreateHist(1, &hist_size, CV_HIST_ARRAY, ranges, 1);
        hist_val_ground = cvCreateHist(1, &hist_size, CV_HIST_ARRAY, ranges, 1);
        std::cout << "hi! 6\n";
        /* Set image to obtain RED as Channel of Interest (COI) */
        cvSetImageCOI(img,1);
        cvCopy(img,channel);
        cvResetImageROI(img);
        std::cout << "hi! 7\n";
        /* Calculate histogram of the Image and store it in the array */
        cvCalcHist( &channel, hist_hue_ground, 0, NULL );
        cvNormalizeHist(hist_hue_ground, 1.0);
        //cvNormalize(ImageVal, ImageValNorm, 0, 255, CV_MINMAX);
        std::cout << "hi! 8\n";
        /* Calculate and Plot the histograms Green and Blue channels as well */
        /* Green channel */
        cvSetImageCOI(img,2);
        cvCopy(img,channel);
        cvResetImageROI(img);
        std::cout << "hi! 9\n";
        cvCalcHist( &channel, hist_sat_ground, 0, NULL );
        cvNormalizeHist(hist_sat_ground, 1.0);
        
        /* Blue channel */
        cvSetImageCOI(img,3);
        cvCopy(img,channel);
        cvResetImageROI(img);
        std::cout << "hi! 10\n";
        cvCalcHist( &channel, hist_val_ground, 0, NULL );
        cvNormalizeHist(hist_val_ground, 1.0);

        /* Plot the Histograms */
        /*for( int i = 1; i < hist_size; i++ )
        {
                if (Rmax < cvRound(cvGetReal1D(hist_hue->bins,i)) ){
                        rm = i;
                        Rmax = cvRound(cvGetReal1D(hist_hue->bins,i));
                        //std::cout<< "rmax " << cvRound(cvGetReal1D(hist_hue->bins,i)) << " i: " << i <<std::endl;
                }
                
                if (Gmax < cvRound(cvGetReal1D(hist_sat->bins,i)) ){
                        gm = i;
                        Gmax = cvRound(cvGetReal1D(hist_sat->bins,i));
                        //std::cout<< "gmax " << cvRound(cvGetReal1D(hist_sat->bins,i)) << " i: " << i <<std::endl;
                }
                
                if (Bmax < cvRound(cvGetReal1D(hist_val->bins,i)) ){
                        bm = i;
                        Bmax = cvRound(cvGetReal1D(hist_val->bins,i));
                        //std::cout<< "bmax " << cvRound(cvGetReal1D(hist_val->bins,i)) << " i: " << i <<std::endl;
                }
        
                cvRectangle( hist_img, cvPoint((int)i*w_scale , hist_img->height),
                cvPoint((int)(i+1)*w_scale, hist_img->height - cvRound(cvGetReal1D(hist_hue->bins,i))),
                CV_RGB(255,0,0), -1, 8, 0 );
                
                //std::cout << "x: " << ((int)(i+1)*w_scale) << ", y: " << (hist_img->height - cvRound(cvGetReal1D(hist_hue->bins,i))) <<'\n';
                
                cvRectangle( hist_img, cvPoint((int)i*w_scale , hist_img->height),
                cvPoint((int)(i+1)*w_scale, hist_img->height - cvRound(cvGetReal1D(hist_sat->bins,i))),
                CV_RGB(0,255,0), -1, 8, 0 );
                
                cvRectangle( hist_img, cvPoint((int)i*w_scale , hist_img->height),
                cvPoint((int)(i+1)*w_scale, hist_img->height - cvRound(cvGetReal1D(hist_val->bins,i))),
                CV_RGB(0,0,255), -1, 8, 0 );
        }*/

        //cvShowImage(window_name, hist_img);
        
        /*
        cvNamedWindow( "Image", 1 );
        cvShowImage( "Image",img);



        cvWaitKey(0);

        cvDestroyWindow( "Image" );
        cvDestroyWindow( "Histogram" );
        cvReleaseImage( &img );
        */
        //std::cout << "max r: " << Rmax << " g: " << Gmax << " b: " << Bmax << '\n';
        //std::cout << "max r: " << rm << " g: " << gm << " b: " << bm << '\n';

        cvReleaseImage( &img_hsv );        
        cvReleaseImage( &img );    
        cvReleaseImage( &hist_img );
        cvReleaseImage( &channel );
        return 0;
}

/*
void Vision::matchBase(cv::Mat const& src_test)
{
        Mat hsv_base;
        Mat hsv_test;
        Mat hsv_test_down;

        //cv::Rect myROI(120, 40, 400, 400);

        //cv::Mat cropped_src_test = src_test(myROI);
        /// Convert to HSV
        cvtColor( src_base, hsv_base, CV_BGR2HSV );
        cvtColor( src_test, hsv_test, CV_BGR2HSV );
        //hsv_test_down = hsv_test( Range( hsv_test.rows/2, hsv_test.rows - 1 ), Range( 0, hsv_test.cols - 1 ) );

        /// Using 30 bins for hue and 32 for saturation
        int h_bins = 50; int s_bins = 60;
        int histSize[] = { h_bins };

        // hue varies from 0 to 256, saturation from 0 to 180
        float h_ranges[] = { 0, 256 };
        float s_ranges[] = { 0, 180 };

        const float* ranges[] = { h_ranges };

        // Use the o-th and 1-st channels
        int channels[] = { 0 };

        /// Histograms
        MatND hist_base;
        MatND hist_test;

        /// Calculate the histograms for the HSV images
        std::cout << "debug 1\n";
        calcHist( &hsv_base, 1, channels, Mat(), hist_base, 1, histSize, ranges, true, false );
        normalize( hist_base, hist_base, 0, 1, NORM_MINMAX, -1, Mat() );
        std::cout << "debug 2\n";
        //calcHist( &hsv_test, 1, channels, Mat(), hist_test, 2, histSize, ranges, true, false );
        //normalize( hist_test, hist_test, 0, 1, NORM_MINMAX, -1, Mat() );
        std::cout << "debug 3\n";
        /// Apply the histogram comparison methods
        for( int i = 0; i < 4; i++ )
        { 
                int compare_method = i;
                double base_base = compareHist( hist_base, hist_base, compare_method );
                //double base_test = compareHist( hist_test, hist_base, compare_method );

                printf( " Method [%d]  : %f \n", i, base_base);
        }
        std::cout << "debug 4\n";
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
*/
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

        //if (windowsEnabled) cvShowImage( "mywindow3", dst );
}


BoxDetectionResult Vision::detectBoxes()
{
        IplImage* frame = orig_small;
        IplImage* frameHD = orig;
        const int BoxROIError = 65;

        //if (windowsEnabled) cvShowImage("mywindow6", frame);

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
                        //std::cout << "**** box at " << box_center_x << ", " << box_center_y << "; image center at " << image_center_x << ", " << image_center_y << "\n";

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
        //keypoint_match_count.push_back(0);
        keypoint_match_count.push_back(20);
//        image_names.push_back("ferrari.png");
//        keypoint_match_count.push_back(7);
//        image_names.push_back("celebes.png");
//        keypoint_match_count.push_back(11);
//        image_names.push_back("fry.png");
//        keypoint_match_count.push_back(5);
//        image_names.push_back("mario.png");
//        keypoint_match_count.push_back(11);
//        image_names.push_back("terminator.png");
//        keypoint_match_count.push_back(11);
//        image_names.push_back("iron.png");
//        keypoint_match_count.push_back(11);
//        image_names.push_back("starry.png");
//        keypoint_match_count.push_back(11);
//        image_names.push_back("thor.png");
//        keypoint_match_count.push_back(11);


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
                std::cout << "Hi there.\n";
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
                        calcHist(src_base, "Histogram2");
                        calcHist(orig, "Histogram");
                        origReady = true;
                }
                if (windowsEnabled) cvShowImage("mywindow8", orig);
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
        while (orig_small == NULL) { usleep(100); std::cout << "WOOSH THREAD\n"; }
        while (true)
        {
        		//std::cout <<runBaseDetection << "\n";
                if (runBaseDetection)
                {
                /// Canny detector
                        std::cout << "whoosh\n";
                        Mat detected_edges;
                        Mat orig_small_copy(orig_small, true);
                        cvtColor( orig_small_copy, detected_edges, CV_BGR2GRAY );
                        cv::Mat sharpened;
                        cv::GaussianBlur(detected_edges, sharpened, cv::Size(0, 0), W1);
                        cv::addWeighted(detected_edges, W2/10.0, sharpened, -W3/10.0, W4, sharpened);
                        //if (windowsEnabled) imshow( "mywindow3", detected_edges );

                        std::cout << "whoosh2\n";

                        Canny( sharpened, sharpened, lowThreshold, highThreshold, 3, true );
                        dilate(sharpened, sharpened, getStructuringElement( MORPH_RECT,
                                                        Size( 3, 3 ),
                                                        Point( 2, 2) ));
                        //if (windowsEnabled) imshow( "mywindow7", sharpened );
                        Mat dst;
                        /// Using Canny's output as a mask, we display our result
                        dst = Scalar::all(0);

                        std::cout << "whoosh3\n";
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
                        std::cout << "whoosh4\n";
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
                        std::cout << "whoosh5\n";
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
                                        releaseBox = true;
                                        baseType = BASE_QUEEN;
                                }
                                /*else if (108 <= c3 && c3 <= 152 &&
                                         108 <= c2 && c2 <= 152 &&
                                         108 <= c1 && c1 <= 152)
                                {
                                        canReleaseBox = true;
                                        baseType = BASE_GREY;
                                } */
                        }
                        //if (windowsEnabled) imshow( "mywindow8", contourDrawing2 );
                        //if (windowsEnabled) imshow( "mywindow10", orig_small_copy );
               }
               else
               {
                        releaseBox = false;
                        usleep(10);
               }

        }

    pthread_exit(NULL);
}

