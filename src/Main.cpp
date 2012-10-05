#include "Controller.hpp"
#include <iostream>

int main()
{

  Controller c;
  c.moveForward();
  sleep(2);
  c.turnLeft();
  sleep(2);
  c.stop();
}
