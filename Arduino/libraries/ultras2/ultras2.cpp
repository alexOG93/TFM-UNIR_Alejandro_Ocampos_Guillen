//  This will include the Header File so that the Source File has access
//  to the function definitions in the myFirstLibrary library.
#include "ultras2.h" 

//  This is where the constructor Source Code appears. The '::' indicates that
//  it is part of the myFirstLibrary class and should be used for all constructors
//  and functions that are part of a class.
Ultras2::Ultras2(int echoPin1, int trigPin1, int echoPin2,  int trigPin2, int echoPin3,  int trigPin3, int echoPin4,  int trigPin4){

  //  This is where the pinModes are defined for circuit operation.
   pinMode(trigPin1, OUTPUT);
  pinMode(echoPin1, INPUT);
   pinMode(trigPin2, OUTPUT);
  pinMode(echoPin2, INPUT);
   pinMode(trigPin3, OUTPUT);
  pinMode(echoPin3, INPUT);
  pinMode(trigPin4, OUTPUT);
  pinMode(echoPin4, INPUT);


  //  The arguments of the constructor are then saved into the private variables.
  _echoPin1 = echoPin1;
  _trigPin1 = trigPin1; 
  _echoPin2 = echoPin2;
  _trigPin2 = trigPin2;
  _echoPin3 = echoPin3;
  _trigPin3 = trigPin3;
  _echoPin4 = echoPin4;
  _trigPin4 = trigPin4;


}

//  For the 'on', 'off' and 'flash' functions, their function return type (void) is
//  specified before the class-function link. They also use the private variables
//  saved in the constructor code.

void Ultras2::initMsg(sensor_msgs::Range &uSonarRangeMsg){
	uSonarRangeMsg.radiation_type = sensor_msgs::Range::ULTRASOUND;
	uSonarRangeMsg.header.frame_id = "robot_base_link";
	uSonarRangeMsg.field_of_view = 0.523599f; //30 degrees in radians
	uSonarRangeMsg.min_range = 5;    // Centimeters
	uSonarRangeMsg.max_range = 20000;       // Centimeters
}

void Ultras2::on(long &cm1, long &cm2, long &cm3, long &cm4){
	digitalWrite(_trigPin1, LOW);
	delayMicroseconds(2);
	digitalWrite(_trigPin1, HIGH);
	delayMicroseconds(10);
	digitalWrite(_trigPin1, LOW);
	_duration1 = pulseIn(_echoPin1, HIGH);
	cm1 = (_duration1/2) / 29.1; 

	digitalWrite(_trigPin2, LOW);
	delayMicroseconds(2);
	digitalWrite(_trigPin2, HIGH);
	delayMicroseconds(10);
	digitalWrite(_trigPin2, LOW);
	_duration2 = pulseIn(_echoPin2, HIGH);
	cm2 = (_duration2/2) / 29.1; 

	digitalWrite(_trigPin3, LOW);
	delayMicroseconds(2);
	digitalWrite(_trigPin3, HIGH);
	delayMicroseconds(10);
	digitalWrite(_trigPin3, LOW);
	_duration3 = pulseIn(_echoPin3, HIGH);
	cm3 = (_duration3/2) / 29.1; 

	digitalWrite(_trigPin4, LOW);
	delayMicroseconds(2);
	digitalWrite(_trigPin4, HIGH);
	delayMicroseconds(10);
	digitalWrite(_trigPin4, LOW); 
	_duration4 = pulseIn(_echoPin4, HIGH);
	cm4 = (_duration4/2) / 29.1;  

}

void Ultras2::off(){
 // Serial.print("sensors off");
 // Mensaje
 
}
