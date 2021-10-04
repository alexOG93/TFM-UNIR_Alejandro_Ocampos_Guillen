#include "Arduino.h"

//  This will include the Header File so that the Source File has access
//  to the function definitions in the myFirstLibrary library.
#include "Controlpololu2.h" 

//  This is where the constructor Source Code appears. The '::' indicates that
//  it is part of the myFirstLibrary class and should be used for all constructors
//  and functions that are part of a class.
Controlpololu2::Controlpololu2(int pwm1, int pwm2, int pwm3,  int pwm4, int ina1,  int inb1, int ina2,  int inb2, int ina3,  int inb3, int ina4,  int inb4){

  //  This is where the pinModes are defined for circuit operation.
pinMode(pwm1, OUTPUT);
pinMode(ina1, OUTPUT);
pinMode(inb1, OUTPUT);
pinMode(pwm2, OUTPUT);
pinMode(ina2, OUTPUT);
pinMode(inb2, OUTPUT);
pinMode(pwm3, OUTPUT);
pinMode(ina3, OUTPUT);
pinMode(inb3, OUTPUT);
pinMode(pwm4, OUTPUT);
pinMode(ina4, OUTPUT);
pinMode(inb4, OUTPUT);


  //  The arguments of the constructor are then saved into the private variables.
  _pwm1 = pwm1;
  _pwm2 = pwm2; 
  _pwm3 = pwm3;
  _pwm4 = pwm4;
  _ina1 = ina1;
  _inb1 = inb1;
  _ina2 = ina2;
  _inb2 = inb2;
  _ina3 = ina3;
  _inb3 = inb3;
  _ina4 = ina4;
  _inb4 = inb4;
}

//  For the 'on', 'off' and 'flash' functions, their function return type (void) is
//  specified before the class-function link. They also use the private variables
//  saved in the constructor code.

void Controlpololu2::modo1(int vel){
analogWrite(_pwm1,vel);
digitalWrite(_ina1, LOW);
digitalWrite(_inb1, HIGH);
analogWrite(_pwm2,vel);
digitalWrite(_ina2, HIGH);
digitalWrite(_inb2, LOW);
analogWrite(_pwm3,vel);
digitalWrite(_ina3, LOW);
digitalWrite(_inb3, HIGH);
analogWrite(_pwm4,vel);
digitalWrite(_ina4, HIGH);
digitalWrite(_inb4, LOW);
  
}
void Controlpololu2::modo2(int vel){
analogWrite(_pwm1,vel);
digitalWrite(_ina1, LOW);
digitalWrite(_inb1, HIGH);
analogWrite(_pwm2,vel);
digitalWrite(_ina2, LOW);
digitalWrite(_inb2, HIGH);
analogWrite(_pwm3,vel);
digitalWrite(_ina3, LOW);
digitalWrite(_inb3, HIGH);
analogWrite(_pwm4,vel);
digitalWrite(_ina4, LOW);
digitalWrite(_inb4, HIGH);
  
}
void Controlpololu2::modo3(int vel){
analogWrite(_pwm1,vel);
digitalWrite(_ina1, HIGH);
digitalWrite(_inb1, LOW);
analogWrite(_pwm2,vel);
digitalWrite(_ina2, HIGH);
digitalWrite(_inb2, LOW);
analogWrite(_pwm3,vel);
digitalWrite(_ina3, HIGH);
digitalWrite(_inb3, LOW);
analogWrite(_pwm4,vel);
digitalWrite(_ina4, HIGH);
digitalWrite(_inb4, LOW);
  
}
void Controlpololu2::modo4(int vel){
analogWrite(_pwm1,vel);
digitalWrite(_ina1, LOW);
digitalWrite(_inb1, HIGH);
analogWrite(_pwm2,vel);
digitalWrite(_ina2, HIGH);
digitalWrite(_inb2, LOW);
analogWrite(_pwm3,vel);
digitalWrite(_ina3, HIGH);
digitalWrite(_inb3, LOW);
analogWrite(_pwm4,vel);
digitalWrite(_ina4, LOW);
digitalWrite(_inb4, HIGH);
  
}
void Controlpololu2::modo5(int vel){
analogWrite(_pwm1,vel);
digitalWrite(_ina1, LOW);
digitalWrite(_inb1, HIGH);
analogWrite(_pwm2,0);
digitalWrite(_ina2, LOW);
digitalWrite(_inb2, LOW);
analogWrite(_pwm3,vel);
digitalWrite(_ina3, LOW);
digitalWrite(_inb3, HIGH);
analogWrite(_pwm4,0);
digitalWrite(_ina4, LOW);
digitalWrite(_inb4, LOW);
  
}
void Controlpololu2::modo6(int vel){
analogWrite(_pwm1,vel);
digitalWrite(_ina1, HIGH);
digitalWrite(_inb1, LOW);
analogWrite(_pwm2,0);
digitalWrite(_ina2, LOW);
digitalWrite(_inb2, LOW);
analogWrite(_pwm3,vel);
digitalWrite(_ina3, HIGH);
digitalWrite(_inb3, LOW);
analogWrite(_pwm4,0);
digitalWrite(_ina4, LOW);
digitalWrite(_inb4, LOW);
  
}
void Controlpololu2::modo7(int vel){
analogWrite(_pwm1,vel);
digitalWrite(_ina1, LOW);
digitalWrite(_inb1, HIGH);
analogWrite(_pwm2,vel);
digitalWrite(_ina2, HIGH);
digitalWrite(_inb2, LOW);
analogWrite(_pwm3,0);
digitalWrite(_ina3, LOW);
digitalWrite(_inb3, LOW);
analogWrite(_pwm4,0);
digitalWrite(_ina4, LOW);
digitalWrite(_inb4, LOW);
  
}
void Controlpololu2::modo8(int vel){
analogWrite(_pwm1,0);
digitalWrite(_ina1, LOW);
digitalWrite(_inb1, LOW);
analogWrite(_pwm2,0);
digitalWrite(_ina2, LOW);
digitalWrite(_inb2, LOW);
analogWrite(_pwm3,vel);
digitalWrite(_ina3, LOW);
digitalWrite(_inb3, HIGH);
analogWrite(_pwm4,vel);
digitalWrite(_ina4, HIGH);
digitalWrite(_inb4, LOW);
  
  
}

void Controlpololu2::modo9(int vel){
analogWrite(_pwm1,vel);
digitalWrite(_ina1, LOW);
digitalWrite(_inb1, HIGH);
analogWrite(_pwm2,0);
digitalWrite(_ina2, LOW);
digitalWrite(_inb2, LOW);
analogWrite(_pwm3,0);
digitalWrite(_ina3, LOW);
digitalWrite(_inb3, LOW);
analogWrite(_pwm4,vel);
digitalWrite(_ina4, LOW);
digitalWrite(_inb4, HIGH);
  
}

void Controlpololu2::modo10(int vel){
analogWrite(_pwm1,vel);
digitalWrite(_ina1, HIGH);
digitalWrite(_inb1, LOW);
analogWrite(_pwm2,0);
digitalWrite(_ina2, LOW);
digitalWrite(_inb2, LOW);
analogWrite(_pwm3,0);
digitalWrite(_ina3, LOW);
digitalWrite(_inb3, LOW);
analogWrite(_pwm4,vel);
digitalWrite(_ina4, HIGH);
digitalWrite(_inb4, LOW);
  
}

void Controlpololu2::modo11(int vel){
analogWrite(_pwm1,0);
digitalWrite(_ina1, LOW);
digitalWrite(_inb1, LOW);
analogWrite(_pwm2,vel);
digitalWrite(_ina2, HIGH);
digitalWrite(_inb2, LOW);
analogWrite(_pwm3,0);
digitalWrite(_ina3, LOW);
digitalWrite(_inb3, LOW);
analogWrite(_pwm4,vel);
digitalWrite(_ina4, HIGH);
digitalWrite(_inb4, LOW);
  
}

void Controlpololu2::modo12(int vel){
analogWrite(_pwm1,0);
digitalWrite(_ina1, LOW);
digitalWrite(_inb1, LOW);
analogWrite(_pwm2,vel);
digitalWrite(_ina2, LOW);
digitalWrite(_inb2, HIGH);
analogWrite(_pwm3,0);
digitalWrite(_ina3, LOW);
digitalWrite(_inb3, LOW);
analogWrite(_pwm4,vel);
digitalWrite(_ina4, LOW);
digitalWrite(_inb4, HIGH);
  
}

void Controlpololu2::modo13(int vel){
analogWrite(_pwm1,vel);
digitalWrite(_ina1, HIGH);
digitalWrite(_inb1, LOW);
analogWrite(_pwm2,vel);
digitalWrite(_ina2, LOW);
digitalWrite(_inb2, HIGH);
analogWrite(_pwm3,vel);
digitalWrite(_ina3, HIGH);
digitalWrite(_inb3, LOW);
analogWrite(_pwm4,vel);
digitalWrite(_ina4, LOW);
digitalWrite(_inb4, HIGH);
  
}

void Controlpololu2::modo14(int vel){
analogWrite(_pwm1,vel);
digitalWrite(_ina1, HIGH);
digitalWrite(_inb1, LOW);
analogWrite(_pwm2,vel);
digitalWrite(_ina2, LOW);
digitalWrite(_inb2, HIGH);
analogWrite(_pwm3,vel);
digitalWrite(_ina3, LOW);
digitalWrite(_inb3, HIGH);
analogWrite(_pwm4,vel);
digitalWrite(_ina4, HIGH);
digitalWrite(_inb4, LOW);
  
}

void Controlpololu2::modo15(int vel){
analogWrite(_pwm1,vel);
digitalWrite(_ina1, HIGH);
digitalWrite(_inb1, LOW);
analogWrite(_pwm2,vel);
digitalWrite(_ina2, LOW);
digitalWrite(_inb2, HIGH);
analogWrite(_pwm3,0);
digitalWrite(_ina3, LOW);
digitalWrite(_inb3, LOW);
analogWrite(_pwm4,0);
digitalWrite(_ina4, LOW);
digitalWrite(_inb4, LOW);
  
}

void Controlpololu2::modo16(int vel){
analogWrite(_pwm1,0);
digitalWrite(_ina1, LOW);
digitalWrite(_inb1, LOW);
analogWrite(_pwm2,0);
digitalWrite(_ina2, LOW);
digitalWrite(_inb2, LOW);
analogWrite(_pwm3,vel);
digitalWrite(_ina3, HIGH);
digitalWrite(_inb3, LOW);
analogWrite(_pwm4,vel);
digitalWrite(_ina4, LOW);
digitalWrite(_inb4, HIGH);
  
}

void Controlpololu2::off(){
analogWrite(_pwm1,0);
digitalWrite(_ina1, LOW);
digitalWrite(_inb1, HIGH);
analogWrite(_pwm2,0);
digitalWrite(_ina2, HIGH);
digitalWrite(_inb2, LOW);
analogWrite(_pwm3,0);
digitalWrite(_ina3, LOW);
digitalWrite(_inb3, HIGH);
analogWrite(_pwm4,0);
digitalWrite(_ina4, HIGH);
digitalWrite(_inb4, LOW);
  
}
