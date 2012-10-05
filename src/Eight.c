// - MotorControl simple -
// This simple example creates a MotorControl handle, hooks the event handlers, then opens and waits for a MotorControl Phidget to be attached.
// Once on has been attaced it will display device information and display any event data read.  The program will then do a short simulation
// of the functionality of the motor by increasing and decreasing the speed of an attached motor.  PLEASE NOTE:  This assignment
// was desgined assuming only one motor attached at motor index 0 (for motorcontrol boards with multiple motors).
//
// Copyright 2008 Phidgets Inc.  All rights reserved.
// This work is licensed under the Creative Commons Attribution 2.5 Canada License. 
// view a copy of this license, visit http://creativecommons.org/licenses/by/2.5/ca/

#include <time.h>
#include <unistd.h>
#include <stdio.h>
#include <phidget21.h>

int AttachHandler(CPhidgetHandle MC, void *userptr)
{
	int serialNo;
	const char *name;

	CPhidget_getDeviceName (MC, &name);
	CPhidget_getSerialNumber(MC, &serialNo);
	printf("%s %10d attached!\n", name, serialNo);

	return 0;
}

int DetachHandler(CPhidgetHandle MC, void *userptr)
{
	int serialNo;
	const char *name;

	CPhidget_getDeviceName (MC, &name);
	CPhidget_getSerialNumber(MC, &serialNo);
	printf("%s %10d detached!\n", name, serialNo);

	return 0;
}

int ErrorHandler(CPhidgetHandle MC, void *userptr, int ErrorCode, const char *Description)
{
	printf("Error handled. %d - %s\n", ErrorCode, Description);
	return 0;
}


int InputChangeHandler(CPhidgetMotorControlHandle MC, void *usrptr, int Index, int State)
{
	printf("Input %d > State: %d\n", Index, State);
	return 0;
}

int VelocityChangeHandler(CPhidgetMotorControlHandle MC, void *usrptr, int Index, double Value)
{
	printf("Motor %d > Current Speed: %f\n", Index, Value);
	return 0;
}

int CurrentChangeHandler(CPhidgetMotorControlHandle MC, void *usrptr, int Index, double Value)
{
	printf("Motor: %d > Current Draw: %f\n", Index, Value);
	return 0;
}

int display_properties(CPhidgetMotorControlHandle phid)
{
	int serialNo, version, numInputs, numMotors;
	const char* ptr;

	CPhidget_getDeviceType((CPhidgetHandle)phid, &ptr);
	CPhidget_getSerialNumber((CPhidgetHandle)phid, &serialNo);
	CPhidget_getDeviceVersion((CPhidgetHandle)phid, &version);
	
	CPhidgetMotorControl_getInputCount(phid, &numInputs);
	CPhidgetMotorControl_getMotorCount(phid, &numMotors);

	printf("%s\n", ptr);
	printf("Serial Number: %10d\nVersion: %8d\n", serialNo, version);
	printf("# Inputs: %d\n# Motors: %d\n", numInputs, numMotors);

	return 0;
}

int motorcontrol_simple()
{
	int result;
	const char *err;

	//Declare a motor control handle
	CPhidgetMotorControlHandle motoControl = 0;

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
	printf("Waiting for MotorControl to be attached....");
	if((result = CPhidget_waitForAttachment((CPhidgetHandle)motoControl, 10000)))
	{
		CPhidget_getErrorDescription(result, &err);
		printf("Problem waiting for attachment: %s\n", err);
		return 0;
	}

	//Display the properties of the attached motor control device
	display_properties(motoControl);

	//read motor control event data
	printf("Reading.....\n");

	//keep displaying motor control event data until user input is read
	printf("Press any key to continue\n");
	getchar();

	//Control the motor a bit.
	//Step 1: increase acceleration to 50, set target sped at 100
	//CPhidgetMotorControl_setAcceleration (motoControl, 1, 50.00);
	//CPhidgetMotorControl_setVelocity (motoControl, 1, 100.0);
        //
        
        const double fastVelocity = -100.0;
        const double slowVelocity = -20.0;
        const double accel = -90.0;
        const int ld = 4;

        //for (int i = 0; i < 2; ++i)
        //{
          //struct timeval now;
	  CPhidgetMotorControl_setAcceleration (motoControl, 0, -accel);
	  CPhidgetMotorControl_setVelocity (motoControl, 0, -fastVelocity);


          CPhidgetMotorControl_setAcceleration (motoControl, 1, accel);
	  CPhidgetMotorControl_setVelocity (motoControl, 1, slowVelocity);

          //gettimeofday(&now, NULL);
          //suseconds_t start = now.tv_usec;

          //clock_t last_check = clock() + CLOCKS_PER_SEC;
          //while (clock() + CLOCKS_PER_SEC - last_check < 5) {}
          sleep(ld);
   	  printf("Part I passed.\n");
/*
////////////////

	  CPhidgetMotorControl_setAcceleration (motoControl, 0, 0);
	  CPhidgetMotorControl_setVelocity (motoControl, 0, 0);

	  CPhidgetMotorControl_setAcceleration (motoControl, 1, 0);
	  CPhidgetMotorControl_setVelocity (motoControl, 1, 0);
	  
          sleep(5);
   	  printf("Waiting part passed.\n");

//////////////////
*/

	  CPhidgetMotorControl_setAcceleration (motoControl, 0, -accel);
	  CPhidgetMotorControl_setVelocity (motoControl, 0, -fastVelocity);

	  CPhidgetMotorControl_setAcceleration (motoControl, 1, accel);
	  CPhidgetMotorControl_setVelocity (motoControl, 1, fastVelocity);
          
          //start = now.tv_usec;
          //while (gettimeofday(&now, NULL) == 0 && now.tv_usec - start < 3000) {}
          sleep(3);
   	  printf("Part II passed.\n");

	  CPhidgetMotorControl_setAcceleration (motoControl, 0, -accel);
	  CPhidgetMotorControl_setVelocity (motoControl, 0, -slowVelocity);

	  CPhidgetMotorControl_setAcceleration (motoControl, 1, accel);
	  CPhidgetMotorControl_setVelocity (motoControl, 1, fastVelocity);

          sleep(ld);
          //start = now.tv_usec;
          //while (gettimeofday(&now, NULL) == 0 && now.tv_usec - start < 5000) {}
   	  printf("Part III passed.\n");

	  CPhidgetMotorControl_setAcceleration (motoControl, 0, -accel);
	  CPhidgetMotorControl_setVelocity (motoControl, 0, -fastVelocity);

	  CPhidgetMotorControl_setAcceleration (motoControl, 1, accel);
	  CPhidgetMotorControl_setVelocity (motoControl, 1, fastVelocity);
          sleep(3);
          
          //start = now.tv_usec;
          //while (gettimeofday(&now, NULL) == 0 && now.tv_usec - start < 3000) {}
   	  printf("Part IV passed.\n");
        //}

	//printf("Press any key to continue\n");
	//getchar();

	CPhidgetMotorControl_setAcceleration (motoControl, 0, 0.00);
	CPhidgetMotorControl_setVelocity (motoControl, 0, 0.0);

	CPhidgetMotorControl_setAcceleration (motoControl, 1, 0.00);
	CPhidgetMotorControl_setVelocity (motoControl, 1, 0.0);
	
	//since user input has been read, this is a signal to terminate the program so we will close the phidget and delete the object we created
	printf("Closing...\n");
	CPhidget_close((CPhidgetHandle)motoControl);
	CPhidget_delete((CPhidgetHandle)motoControl);

	//all done, exit
	return 0;
}

int main(int argc, char* argv[])
{
	motorcontrol_simple();
	return 0;
}

  
