#include "Controller.hpp"
#include <iostream>
#include <cassert>

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

int InputChangeHandler(CPhidgetInterfaceKitHandle IFK, void *usrptr, int Index, int State)
{
        std::cout << "Whiskers touched.\n";
	Controller::whiskersTouched = true;
	return 0;
}

bool Controller::whiskersTouched = false;

Controller::Controller()
: motoControl(0)
, speed(45)
, speedLeftFactor(1.0)
, speedRightFactor(1.2)
, accelLeftFactor(1.0)
, accelRightFactor(1.2)
, rotationOnSpotSpeed(50)
, accel(45)
, backwardTurnFastFactor(1.5)
, backwardTurnSlowFactor(-1.5)
, servo(0)
, servoOpen(35.00)
, servoClosed(120.00)
/*, speed(100)
, speedLeftFactor(1.0)
, speedRightFactor(1.0)
, accelLeftFactor(1.0)
, accelRightFactor(1.0)
, rotationOnSpotSpeed(40)
, accel(100)
, backwardTurnFastFactor(1)
, backwardTurnSlowFactor(-1)*/
{
	//create the motor control object
	CPhidgetMotorControl_create(&motoControl);

	//Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
	CPhidget_set_OnAttach_Handler((CPhidgetHandle)motoControl, AttachHandler, NULL);
	CPhidget_set_OnDetach_Handler((CPhidgetHandle)motoControl, DetachHandler, NULL);
	CPhidget_set_OnError_Handler((CPhidgetHandle)motoControl, ErrorHandler, NULL);



	ifKit = 0;

	//create the InterfaceKit object
	CPhidgetInterfaceKit_create(&ifKit);

	//Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
	CPhidget_set_OnAttach_Handler((CPhidgetHandle)ifKit, AttachHandler, NULL);
	CPhidget_set_OnDetach_Handler((CPhidgetHandle)ifKit, DetachHandler, NULL);
	CPhidget_set_OnError_Handler((CPhidgetHandle)ifKit, ErrorHandler, NULL);
	
        CPhidgetInterfaceKit_set_OnInputChange_Handler (ifKit, InputChangeHandler, NULL);
        
	//open the interfacekit for device connections
	CPhidget_open((CPhidgetHandle)ifKit, -1);
	
	CPhidgetAdvancedServo_create(&servo);
	CPhidget_open((CPhidgetHandle)servo, -1);
	
	int result;
	const char *err;
	if((result = CPhidget_waitForAttachment((CPhidgetHandle)ifKit, 10000)))
	{
		CPhidget_getErrorDescription(result, &err);
	}

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

	CPhidgetAdvancedServo_setEngaged(servo, 0, 1);
	CPhidgetAdvancedServo_setPosition (servo, 0, 40.00);
}

int Controller::getSensorValue(int sensorId)
{
        int value;
        CPhidgetInterfaceKit_getSensorValue(ifKit, sensorId, &value);
        return value;
}

int Controller::getWhiskerLeftValue()
{
        return getSensorValue(6);
}

int Controller::getWhiskerRightValue()
{
        return getSensorValue(7);
}

int Controller::getIRLeftValue()
{
        return getSensorValue(0);
}

int Controller::getIRRightValue()
{
        return getSensorValue(1);
}

void Controller::openServo()
{
        CPhidgetAdvancedServo_setPosition (servo, 0, servoOpen);
}

void Controller::closeServo()
{
        CPhidgetAdvancedServo_setPosition (servo, 0, servoClosed);
}

void Controller::moveForward()
{
        CPhidgetMotorControl_setAcceleration (motoControl, 0, -accel * accelLeftFactor);
	CPhidgetMotorControl_setVelocity (motoControl, 0, -speed * speedLeftFactor);

	CPhidgetMotorControl_setAcceleration (motoControl, 1, accel * accelRightFactor);
	CPhidgetMotorControl_setVelocity (motoControl, 1, speed * speedRightFactor);
}

void Controller::moveBackward()
{
        CPhidgetMotorControl_setAcceleration (motoControl, 0, accel * accelLeftFactor);
	CPhidgetMotorControl_setVelocity (motoControl, 0, speed * speedLeftFactor);

	CPhidgetMotorControl_setAcceleration (motoControl, 1, -accel * accelRightFactor);
	CPhidgetMotorControl_setVelocity (motoControl, 1, -speed * speedRightFactor);
}

void Controller::moveBackwardLeft()
{
        CPhidgetMotorControl_setAcceleration (motoControl, 0, accel * backwardTurnSlowFactor);
	CPhidgetMotorControl_setVelocity (motoControl, 0, speed * backwardTurnSlowFactor);

	CPhidgetMotorControl_setAcceleration (motoControl, 1, -accel*backwardTurnFastFactor);
	CPhidgetMotorControl_setVelocity (motoControl, 1, -speed*backwardTurnFastFactor);
}

void Controller::moveBackwardRight()
{
        CPhidgetMotorControl_setAcceleration (motoControl, 0, accel*backwardTurnFastFactor);
	CPhidgetMotorControl_setVelocity (motoControl, 0, speed*backwardTurnFastFactor);

	CPhidgetMotorControl_setAcceleration (motoControl, 1, -accel * backwardTurnSlowFactor);
	CPhidgetMotorControl_setVelocity (motoControl, 1, -speed * backwardTurnSlowFactor);
}

void Controller::turn(double angle)
{
        //assert(-90 <= angle && angle <= 90);
        
        if (-90 <= angle && angle < -10)
        {
                turnLeft();
        }
        else if (10 < angle && angle <= 90)
        {
                turnRight();
        }
        else
        {
                moveForward();
                /*double normalAngle = angle / 90;
                       
                if(normalAngle < 0){
                        CPhidgetMotorControl_setAcceleration (motoControl, 0, -speed* (1+normalAngle) );
                        CPhidgetMotorControl_setVelocity (motoControl, 0, -accel* (1+normalAngle) );

                        CPhidgetMotorControl_setAcceleration (motoControl, 1, speed* -normalAngle);
                        CPhidgetMotorControl_setVelocity (motoControl, 1, accel* -normalAngle);
                }
                else {
                        CPhidgetMotorControl_setAcceleration (motoControl, 0, -speed* normalAngle);
        	        CPhidgetMotorControl_setVelocity (motoControl, 0, -accel* normalAngle);

        	        CPhidgetMotorControl_setAcceleration (motoControl, 1, speed* (1-normalAngle));
        	        CPhidgetMotorControl_setVelocity (motoControl, 1, accel* (1-normalAngle));
                }*/

        }
}

void Controller::turnLeft()
{
        CPhidgetMotorControl_setAcceleration (motoControl, 0, 0);
	CPhidgetMotorControl_setVelocity (motoControl, 0, 0);

	CPhidgetMotorControl_setAcceleration (motoControl, 1, accel * accelRightFactor);
	CPhidgetMotorControl_setVelocity (motoControl, 1, speed * speedRightFactor);
}

void Controller::turnRight()
{
        CPhidgetMotorControl_setAcceleration (motoControl, 0, -accel * accelLeftFactor);
	CPhidgetMotorControl_setVelocity (motoControl, 0, -speed * speedLeftFactor);

	CPhidgetMotorControl_setAcceleration (motoControl, 1, 0);
	CPhidgetMotorControl_setVelocity (motoControl, 1, 0);
}

void Controller::rotateOnSpot()
{
        int rotationSpeed = 100;
        CPhidgetMotorControl_setAcceleration (motoControl, 0, -rotationSpeed);
	CPhidgetMotorControl_setVelocity (motoControl, 0, -rotationSpeed);

	CPhidgetMotorControl_setAcceleration (motoControl, 1, -rotationSpeed);
	CPhidgetMotorControl_setVelocity (motoControl, 1, -rotationSpeed);
}

void Controller::rotateOnSpotLeft()
{
        CPhidgetMotorControl_setAcceleration (motoControl, 0, -rotationOnSpotSpeed);
	CPhidgetMotorControl_setVelocity (motoControl, 0, -rotationOnSpotSpeed);

	CPhidgetMotorControl_setAcceleration (motoControl, 1, -rotationOnSpotSpeed);
	CPhidgetMotorControl_setVelocity (motoControl, 1, -rotationOnSpotSpeed);
}

void Controller::rotateOnSpotRight()
{
        CPhidgetMotorControl_setAcceleration (motoControl, 0, rotationOnSpotSpeed);
	CPhidgetMotorControl_setVelocity (motoControl, 0, rotationOnSpotSpeed);

	CPhidgetMotorControl_setAcceleration (motoControl, 1, rotationOnSpotSpeed);
	CPhidgetMotorControl_setVelocity (motoControl, 1, rotationOnSpotSpeed);
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
        CPhidgetAdvancedServo_setEngaged(servo, 0, 0);
	CPhidget_close((CPhidgetHandle)servo);
	CPhidget_delete((CPhidgetHandle)servo);

}
