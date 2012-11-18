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
        ctrl.turn(0); 
        usleep(1000000);
        ctrl.turn(45); 
        usleep(1000000);
        ctrl.turn(-45);
        usleep(1000000); 
        ctrl.stop();
}
