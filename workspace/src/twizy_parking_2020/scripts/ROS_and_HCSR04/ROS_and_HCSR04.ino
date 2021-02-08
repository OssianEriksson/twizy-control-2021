#include <ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>

ros::NodeHandle nh;
std_msgs::Float64MultiArray Distance;

ros::Publisher ultrasonic("ultrasonic",&Distance);

// Observe the location of the file. When running, the file in rosserial is used
// Also, new files need to be uploaded to the actual arduino board

// defines pins numbers
const int trigPin1 = 8;
const int echoPin1 = 9;

const int trigPin2 = 10;
const int echoPin2 = 11;

const int trigPin3 = 12;
const int echoPin3 = 13;

// defines variables
float duration;
float distanceFrontWheel, distanceRearWheel, distanceReverse;

void setup() {
  nh.initNode();
  nh.advertise(ultrasonic);
  
  
  pinMode(trigPin1, OUTPUT); // Sets the trigPin as an Output
  pinMode(echoPin1, INPUT); // Sets the echoPin as an Input
  pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
  pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  
  Serial.begin(57600); // Starts the serial communication

}
//void calculateDistance(String whichSensor, const int trigPin, const int echoPin, float distance)

float calculateDistance(const int trigPin, const int echoPin, float distance){
    // Clears the trigPin
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPin, HIGH);
  
  // Calculating the distance
  distance = (duration/2)/29.1;

  return distance;
  
}


void loop() {
  // Clears the trigPin
  float distance1 = calculateDistance(trigPin1, echoPin1, distanceFrontWheel);
  if (float distance1 > 450) {float distance1 = 450;}
  delay(50);
  float distance2 =  calculateDistance(trigPin2, echoPin2, distanceRearWheel);
  if (float distance2 > 450) {float distance2 = 450;}
  delay(50);
  float distance3 = calculateDistance(trigPin3, echoPin3, distanceReverse);
  if (float distance3 > 450) {float distance3 = 450;}

  float distanceArray [3] = {distance1, distance2, distance3}; 
  
  Distance.data = distanceArray;
  Distance.data_length = 3;
  ultrasonic.publish(&Distance);
  nh.spinOnce();
  // Prints the distance on the Serial Monitor
  //Serial.print("Distance: ");
  //Serial.println(distance);
  delay(50);
}
