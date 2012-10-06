#ifndef CONTROLLER_HPP
#define CONTROLLER_HPP

#include <phidget21.h>

class Controller
{
public:
  explicit Controller();
  ~Controller();
  
  void moveForward();
  void moveBackward();
  void turnLeft();
  void turnRight();
  void turn(double angle);
  void stop();
  
  double speed;
  double accel;
  CPhidgetMotorControlHandle motoControl;
};


int AttachHandler(CPhidgetHandle MC, void *userptr);
int DetachHandler(CPhidgetHandle MC, void *userptr);
int ErrorHandler(CPhidgetHandle MC, void *userptr, int ErrorCode, const char *Description);
int InputChangeHandler(CPhidgetMotorControlHandle MC, void *usrptr, int Index, int State);
int VelocityChangeHandler(CPhidgetMotorControlHandle MC, void *usrptr, int Index, double Value);
int CurrentChangeHandler(CPhidgetMotorControlHandle MC, void *usrptr, int Index, double Value);


#endif // CONTROLLER_HPP
