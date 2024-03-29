#include "Controller.hpp"
#include "Vision.hpp"
#include "Robot.hpp"

#include <iostream>

int main(int argc, char** argv)
{
    Controller ctrl;
    Vision vision;
    Robot robot(ctrl, vision);
    std::cout << "Quitting...\n";
}
