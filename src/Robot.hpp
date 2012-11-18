#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "Controller.hpp"
#include "Vision.hpp"

class Robot
{
public:
        Robot(Controller& ctrl, Vision& vision);
        ~Robot();
        
        void start();
        static std::pair<size_t, size_t> Robot::longestLine(int* vec, size_t size);
        inline void stopAndRotate();
        inline void grabBox();
        inline void dropBox();
        void run();
private:
        const static int freeSpaceThreshold = 20; 
        const static int irThreshold = 300;
        
        const static bool movementEnabled = true;        
        bool canReleaseBox; 
        bool boxDetected;
        bool hasBox;
        Controller* ctrl;
        Vision* vision;

        // Try to remove these        
        int stoppedForPicturesCounter = 0;
        int sawBoxCounter = 3;
        bool running;
};

#endif // ROBOT_HPP
