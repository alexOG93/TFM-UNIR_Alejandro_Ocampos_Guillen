#ifndef Ultras2_h
#define Ultras2_h
#include "Arduino.h"
#include <ros.h>
#include <sensor_msgs/Range.h>

class Ultras2{

  //  'public:' and 'private:' refer to the security of the functions
  //  and variables listed in that set. Contents that are public can be 
  //  accessed from a sketch for use, however private contents can only be
  //  accessed from within the class itself.
  public:
  
    //  The first item in the class is known as the constructor. It shares the
    //  same name as the class and is used to create an instance of the class.
    //  It has no return type and is only used once per instance. 
    Ultras2(int echoPin1, int trigPin1, int echoPin2,  int trigPin2, int echoPin3,  int trigPin3, int echoPin4,  int trigPin4);
    /*int echoPin1 = 37;    // Echo blue 37
    int trigPin1 = 35;    // Trigger white 35
    int echoPin2 = 33;    // Echo blue 
    int trigPin2 = 31;    // Trigger white
    int echoPin3 = 29;    // Echo blue 
    int trigPin3 = 27;    // Trigger white
    int echoPin4 = 25;    // Echo blue 25
    int trigPin4 = 23;    // Trigger white 23
   */
    
    //  Below are the functions of the class. They are the functions available
    //  in the library for a user to call.    
    // methods
	void initMsg(sensor_msgs::Range &msgSensor);
    void on(long &cm1, long &cm2, long &cm3, long &cm4);              
    void off();
   

  private:                 
    //  When dealing with private variables, it is common convention to place
    //  an underscore before the variable name to let a user know the variable
    //  is private.   
    int _echoPin1, _trigPin1, _echoPin2,  _trigPin2, _echoPin3,  _trigPin3, _echoPin4,  _trigPin4;
    long _duration1, _duration2, _duration3, _duration4;

};

//  The end wrapping of the #ifndef Include Guard
#endif
