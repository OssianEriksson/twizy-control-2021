// This program sends out the speed and angle messages to the CAN-bus.
// TODO: Add filters with warnings for values lower or greater than what the car can handle


#include <iostream>
#include <stdio.h>
#include <string>
#include <canlib.h>
#include "CANComunication.h"
#include <math.h>

int CANComunication::StartCan()
{
	std::cout << "startcan" << std::endl;
	canInitializeLibrary();

	int CHANNEL_NUMBER = 0;
	hnd = canOpenChannel(CHANNEL_NUMBER, canOPEN_ACCEPT_VIRTUAL);

	if (hnd < 0)
	{
		printf("canOpenCHannel failed, status=%d\n", hnd);
		exit(1);

	}
	stat = canSetBusParams(hnd, canBITRATE_500K, 0, 0, 0, 0, 0);

	canSetBusOutputControl(hnd, canDRIVER_NORMAL);

	stat = canBusOn(hnd);
	if (stat < 0) {
		printf("canBusOn failed, status=%d\n", stat);
		exit(1);
	}
	std::cout << "exit startcan" << std::endl;
}



int CANComunication::SendMessage()
{
	
	// calculate messsage 
		// steering 
	angle_1 = round( 6.375 * abs(steer));
	angle_2;
	if (steer < 0)
		angle_2 = 0;
	else
		angle_2 = 255;

	char msg_steer[8] = { angle_1,angle_2,0,0,0,0,0,0 };
	// gas
	throttle = round((255 / 5) * abs(speed));

	char dir = 0;

	if(speed < 0)
		dir = -1;
	else
		dir = 0;

	char msg_speed[8] = { throttle,dir,0,0,0,0,0,0 };

	// print steer and speed
	std::cout << "Speed is ->  " << speed << "  Stering anlge is  " << steer << std::endl;

	// write to canbus
	stat = canWrite(hnd, 150, msg_steer, 8, 0);
	canWriteSync(hnd, 100);
	if (stat < 0) {
		printf("Failed, steering status == %d\n", stat);
	}
	stat = canWrite(hnd, 154, msg_speed, 8, 0);
	canWriteSync(hnd, 100);
	if (stat < 0) {
		printf("Failed, throtle status == %d\n", stat);
}

return 0;
}

int CANComunication::StopCan()
{
	

	canBusOff(hnd);
	canClose(hnd);

	return 0;

}


