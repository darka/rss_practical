#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <algorithm>

int calcHist(IplImage* img)
{
        /* Always check if the program can find a file */
        if( !img )
        return -1;

        IplImage* channel = cvCreateImage( cvGetSize(img), 8, 1 );
        IplImage *hist_img = cvCreateImage(cvSize(300,240), 8, 3);
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
                        std::cout<< "rmax " << cvRound(cvGetReal1D(hist_red->bins,i)) << " i: " << i <<std::endl;
                }
                
                if (Gmax < cvRound(cvGetReal1D(hist_green->bins,i)) ){
                        gm = i;
                        Gmax = cvRound(cvGetReal1D(hist_green->bins,i));
                        std::cout<< "gmax " << cvRound(cvGetReal1D(hist_green->bins,i)) << " i: " << i <<std::endl;
                }
                
                if (Bmax < cvRound(cvGetReal1D(hist_blue->bins,i)) ){
                        bm = i;
                        Bmax = cvRound(cvGetReal1D(hist_blue->bins,i));
                        std::cout<< "bmax " << cvRound(cvGetReal1D(hist_blue->bins,i)) << " i: " << i <<std::endl;
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

        cvNamedWindow( "Image", 1 );
        cvShowImage( "Image",img);

        /* create a window to show the histogram of the image */
        cvNamedWindow("Histogram", 1);
        cvShowImage( "Histogram", hist_img);

        cvWaitKey(0);

        cvDestroyWindow( "Image" );
        cvDestroyWindow( "Histogram" );
        cvReleaseImage( &img );
        std::cout << "max r: " << Rmax << " g: " << Gmax << " b: " << Bmax << '\n';
        std::cout << "max r: " << rm << " g: " << gm << " b: " << bm << '\n';
        return 0;
}

int main(int argc, char** argv) {

        bool temp = true;

        CvCapture* capture = cvCaptureFromCAM( CV_CAP_ANY );
        cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_WIDTH, 320 );
        cvSetCaptureProperty( capture, CV_CAP_PROP_FRAME_HEIGHT, 240 );
        
        if ( !capture ) {
                fprintf( stderr, "ERROR: capture is NULL \n" );
                getchar();
                return -1;
        }
        // Create a window in which the captured images will be presented
        cvNamedWindow( "mywindow", CV_WINDOW_AUTOSIZE );
        // Show the image captured from the camera in the window and repeat
        while ( temp ) {
                // Get one frame
                IplImage* frame = cvQueryFrame( capture );
                if ( !frame ) {
                        fprintf( stderr, "ERROR: frame is null...\n" );
                        getchar();
                        break;
                }
                //cvShowImage( "mywindow", frame );
                calcHist(frame);
                temp =false;
                
                // Do not release the frame!
                //If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
                //remove higher bits using AND operator
                if ( (cvWaitKey(10) & 255) == 27 ) break;
        }
        // Release the capture device housekeeping
        cvReleaseCapture( &capture );
        cvDestroyWindow( "mywindow" );
        return 0;
}
