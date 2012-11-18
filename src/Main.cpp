#include "Controller.hpp"
#include "Vision.hpp"
#include "Robot.hpp"

int main()
{
        Controller ctrl;
        Vision vision;
        Robot robot(ctrl, vision);
        robot.start();
}
