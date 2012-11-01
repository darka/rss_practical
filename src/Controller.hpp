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
  void moveBackwardLeft();
  void moveBackwardRight();
  void turnLeft();
  void turnRight();
  void turn(double angle);
  void stop();
  void rotateOnSpot();
  
  int getIRLeftValue();
  int getIRRightValue();
  int getWhiskerLeftValue();
  int getWhiskerRightValue();
  
  double speed;
  double speedLeftFactor;
  double speedRightFactor;
  double accel;
  double accelLeftFactor;
  double accelRightFactor;
  double backwardTurnSlowFactor;
  double backwardTurnFastFactor;
  CPhidgetMotorControlHandle motoControl;
  CPhidgetInterfaceKitHandle ifKit;
  
  static bool whiskersTouched;
  
private:
  int getSensorValue(int sensorId);
};



int AttachHandler(CPhidgetHandle MC, void *userptr);
int DetachHandler(CPhidgetHandle MC, void *userptr);
int ErrorHandler(CPhidgetHandle MC, void *userptr, int ErrorCode, const char *Description);
int InputChangeHandler(CPhidgetMotorControlHandle MC, void *usrptr, int Index, int State);
int VelocityChangeHandler(CPhidgetMotorControlHandle MC, void *usrptr, int Index, double Value);
int CurrentChangeHandler(CPhidgetMotorControlHandle MC, void *usrptr, int Index, double Value);
int InputChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int State);

#endif // CONTROLLER_HPP
