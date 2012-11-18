#include "Vision.hpp"




inline static bool Vision::inRangeHsv(unsigned char hue,
                                      unsigned char sat,
                                      unsigned char val,
                                      int           range)
{
    // CALCULATES IF HSV VALUE OF PIXEL IS IN SPECIFIED RANGE
    //
    // INPUT:   HSV-VALUE OF PIXEL AND RANGE
    // OUTPUT:  TRUE OR FALSE

    const char diffRange = 14;
    const char blackness = 70;

    if ( maxHue >= hue &&
         hue >= minHue &&
         maxSat >= sat &&
         sat >= minSat &&
         maxVal >= val &&
         val >= minVal )
    {
            return true;
    }

    return false;



}

inline static bool Vision::inRange(unsigned char red,
                                   unsigned char green,
                                   unsigned char blue,
                                   int           range)
{
    // CALCULATES IF RGB VALUE OF PIXEL IS IN SPECIFIED RANGE
    //
    // INPUT:   RGB-VALUE OF PIXEL AND RANGE
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


inline void static Vision::interpolatedCoordinates( int &min_x,
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


inline static bool Vision::boxIsCentered( int image_center_x,
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


inline static bool Vision::checkForMatch( size_t                        point_i,
                                          std::vector<cv::KeyPoint>&    other_points,
                                          cv::Mat&                      descriptors_image,
                                          cv::Mat&                      descriptors_camera )
{


}




