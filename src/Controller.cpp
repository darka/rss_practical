#include "Controller.hpp"
#include <iostream>

int AttachHandler(CPhidgetHandle MC, void *userptr)
{
	int serialNo;
	const char *name;

	CPhidget_getDeviceName (MC, &name);
	CPhidget_getSerialNumber(MC, &serialNo);

	return 0;
}

int DetachHandler(CPhidgetHandle MC, void *userptr)
{
	int serialNo;
	const char *name;

	CPhidget_getDeviceName (MC, &name);
	CPhidget_getSerialNumber(MC, &serialNo);

	return 0;
}

int ErrorHandler(CPhidgetHandle MC, void *userptr, int ErrorCode, const char *Description)
{
	return 0;
}


int InputChangeHandler(CPhidgetMotorControlHandle MC, void *usrptr, int Index, int State)
{
	return 0;
}

int VelocityChangeHandler(CPhidgetMotorControlHandle MC, void *usrptr, int Index, double Value)
{
	return 0;
}

int CurrentChangeHandler(CPhidgetMotorControlHandle MC, void *usrptr, int Index, double Value)
{
	return 0;
}

Controller::Controller()
: motoControl(0)
, speed(-100)
, accel(-100)
{
	//create the motor control object
	CPhidgetMotorControl_create(&motoControl);

	//Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
	CPhidget_set_OnAttach_Handler((CPhidgetHandle)motoControl, AttachHandler, NULL);
	CPhidget_set_OnDetach_Handler((CPhidgetHandle)motoControl, DetachHandler, NULL);
	CPhidget_set_OnError_Handler((CPhidgetHandle)motoControl, ErrorHandler, NULL);

	//Registers a callback that will run if an input changes.
	//Requires the handle for the Phidget, the function that will be called, and a arbitrary pointer that will be supplied to the callback function (may be NULL).
	CPhidgetMotorControl_set_OnInputChange_Handler (motoControl, InputChangeHandler, NULL);

	//Registers a callback that will run if a motor changes.
	//Requires the handle for the Phidget, the function that will be called, and a arbitrary pointer that will be supplied to the callback function (may be NULL).
	CPhidgetMotorControl_set_OnVelocityChange_Handler (motoControl, VelocityChangeHandler, NULL);

	//Registers a callback that will run if the current draw changes.
	//Requires the handle for the Phidget, the function that will be called, and a arbitrary pointer that will be supplied to the callback function (may be NULL).
	CPhidgetMotorControl_set_OnCurrentChange_Handler (motoControl, CurrentChangeHandler, NULL);

	//open the motor control for device connections
	CPhidget_open((CPhidgetHandle)motoControl, -1);

	//get the program to wait for a motor control device to be attached
	CPhidget_waitForAttachment((CPhidgetHandle)motoControl, 10000);

}

void Controller::moveForward()
{
        CPhidgetMotorControl_setAcceleration (motoControl, 0, -accel);
	CPhidgetMotorControl_setVelocity (motoControl, 0, -speed);

	CPhidgetMotorControl_setAcceleration (motoControl, 1, accel);
	CPhidgetMotorControl_setVelocity (motoControl, 1, speed);
}

void Controller::moveBackward()
{
        CPhidgetMotorControl_setAcceleration (motoControl, 0, accel);
	CPhidgetMotorControl_setVelocity (motoControl, 0, speed);

	CPhidgetMotorControl_setAcceleration (motoControl, 1, -accel);
	CPhidgetMotorControl_setVelocity (motoControl, 1, -speed);
}

void Controller::turnLeft()
{
        CPhidgetMotorControl_setAcceleration (motoControl, 0, 0);
	CPhidgetMotorControl_setVelocity (motoControl, 0, 0);

	CPhidgetMotorControl_setAcceleration (motoControl, 1, speed);
	CPhidgetMotorControl_setVelocity (motoControl, 1, accel);
}

void Controller::turnRight()
{
        CPhidgetMotorControl_setAcceleration (motoControl, 0, -speed);
	CPhidgetMotorControl_setVelocity (motoControl, 0, -accel);

	CPhidgetMotorControl_setAcceleration (motoControl, 1, 0);
	CPhidgetMotorControl_setVelocity (motoControl, 1, 0);
}

void Controller::stop()
{
        CPhidgetMotorControl_setAcceleration (motoControl, 0, 0);
	CPhidgetMotorControl_setVelocity (motoControl, 0, 0);

	CPhidgetMotorControl_setAcceleration (motoControl, 1, 0);
	CPhidgetMotorControl_setVelocity (motoControl, 1, 0);
}

Controller::~Controller()
{
	CPhidget_close((CPhidgetHandle)motoControl);
	CPhidget_delete((CPhidgetHandle)motoControl);
}