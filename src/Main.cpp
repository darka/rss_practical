#include "Controller.hpp"
#include "Vision.hpp"
#include "Robot.hpp"

#include <iostream>

int main(int argc, char** argv)
{
        const char* base;
        const char* box;
        if (argc == 0 || argc > 2 || strlen(argv[2]) > 1)
        {
                Vision::base = argv[2][0];
                std::cout << "Base: " << Vision::base << '\n';
                box = argv[1];
        }
        else
        {
                std::cout << "Please specify base and box\n";
                return 1;
        }
        Controller ctrl;
        Vision vision(box);
        Robot robot(ctrl, vision);
        std::cout << "Quitting...\n";
}
