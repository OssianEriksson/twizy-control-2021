// This program subscribes to the "controls" msg and sends it to CANComunications.cpp

#include "ros/ros.h"
#include "CANComunication.h"
#include "twizy/car_control.h"
 
CANComunication can;

// steer is an angle between -40 and 40 where negative numbers turn the wheel to the left
// Speed is in km/h with a span of -5 to 5 km/h. This uses the custom message car_control
// which is made of two int32 called speed and angle. 
void callback_send(const twizy::car_control::ConstPtr& msg)
{	
	can.speed = msg->speed;
	can.steer = msg->angle;
	can.SendMessage();
}

// Initializes the node and subscribes to the "controls" topic
int main(int argc, char **argv){
    std::cout << "main" << std::endl;
	can.StartCan();
	ros::init(argc,argv,"control_listener");
	ros::NodeHandle n;
	// subscribes to the topic "controls" and calls the function callback_send when it receives a message.
	ros::Subscriber sub = n.subscribe("controls", 5, callback_send);

	ros::spin();
}


