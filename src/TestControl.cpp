#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <algorithm>
#include <cstring>
#include <vector>
#include <cmath>
#include "Controller.hpp"

#define PI 3.14159265


int main(int argc, char** argv) {
        Controller ctrl;
        usleep(2000000);
        ctrl.turnAt(45);
        usleep(1000000);
        ctrl.turnAt(-45);
        usleep(1000000);
       // ctrl.turnAt(22);
        usleep(1000000);
       // ctrl.turnAt(-22);
        //usleep(1000000);
        /*
        ctrl.openServo();
        //ctrl.rotateOnSpotLeft();
        
        usleep(1200000);
        ctrl.closeServo();
        usleep(1200000);
        //ctrl.rotateOnSpotLeft();
        
        //ctrl.moveBackwardLeft();
        
        //usleep(1500);*/
        //ctrl.stop();
}
