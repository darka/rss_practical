#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "Controller.hpp"
#include "Vision.hpp"

class Robot
{
public:
        Robot(Controller& ctrl, Vision& vision);
        
        static std::pair<size_t, size_t> longestLine(int* vec, size_t size);
        inline void stopAndRotate();
        inline void grabBox();
        inline void dropBox(double angle);
        void run(IplImage* detected_floor, IplImage* normalCapture, IplImage* hdCapture);
        void moveBackConsideringFreeSpace(IplImage* detected_floor);

private:
        const static int freeSpaceThreshold = 20; 
        const static int irThreshold = 300;
        
        const static bool movementEnabled = false;
        bool lastMoveWasTowardsBox;
        bool hasBox;
        Controller* ctrl;
        Vision* vision;

        int moveable[Vision::CAMERA_WIDTH];

        // TODO: Try to get rid of these        
        int sawBoxCounter;
        bool running;
};

#endif // ROBOT_HPP
