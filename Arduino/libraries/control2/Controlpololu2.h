#ifndef Controlpololu2_h
#define Controlpololu2_h
#include "Arduino.h"


class Controlpololu2{

  //  'public:' and 'private:' refer to the security of the functions
  //  and variables listed in that set. Contents that are public can be 
  //  accessed from a sketch for use, however private contents can only be
  //  accessed from within the class itself.
  public:
  
    //  The first item in the class is known as the constructor. It shares the
    //  same name as the class and is used to create an instance of the class.
    //  It has no return type and is only used once per instance. 
    Controlpololu2(int pwm1, int pwm2, int pwm3,  int pwm4, int ina1,  int inb1, int ina2,  int inb2, int ina3,  int inb3, int ina4,  int inb4);
    /*
     int pwm1 = 8; 
int pwm2 = 9; 
int pwm3 = 10; 
int pwm4 = 11; 
int ina1 = 53; 
int inb1 = 51;
int ina2 = 49;
int inb2 = 47;
int ina3 = 45;
int inb3 = 43;
int ina4 = 41;
int inb4 = 39;
   */
    
    //  Below are the functions of the class. They are the functions available
    //  in the library for a user to call.    
    // methods
    void modo1(int vel);     
    void modo2(int vel);         
    void modo3(int vel);  
    void modo4(int vel); 
    void modo5(int vel); 
    void modo6(int vel); 
    void modo7(int vel); 
    void modo8(int vel); 
    void modo9(int vel); 
    void modo10(int vel); 
    void modo11(int vel);
    void modo12(int vel);
    void modo13(int vel);
    void modo14(int vel);
    void modo15(int vel);
    void modo16(int vel);
    void off();
   

  private:                  
    
    //  When dealing with private variables, it is common convention to place
    //  an underscore before the variable name to let a user know the variable
    //  is private.   
    int _pwm1, _pwm2, _pwm3, _pwm4, _ina1, _inb1, _ina2, _inb2, _ina3, _inb3, _ina4, _inb4, _vel;
   
};

//  The end wrapping of the #ifndef Include Guard
#endif
