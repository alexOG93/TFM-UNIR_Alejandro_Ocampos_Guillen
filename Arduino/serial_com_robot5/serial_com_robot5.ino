#include <Controlpololu2.h>
#include <ultras2.h>
//#include <rosControl.h>
#include <encoderrs2.h>

Ultras2 ultra2(36, 34, 32,  30, 28,  26, 24,  22);
Encoderrs2 Encoders2(38, 40, 42, 44, 46, 48, 50, 52);
Controlpololu2 control(8, 9, 10,  11, 53,  51, 49,  47, 45, 43, 41, 39);

long cm1, cm2, cm3, cm4; 
int key = 0;
int velocidad = 25;
unsigned long key_time = 0;
unsigned long timeout, loop_time;

void setup() {
  Serial.begin(115200);
  timeout = 500; // 500 ms of timeout for receiving a key
}

void loop() {
  delay(100);
  ultra2.on(cm1, cm2, cm3, cm4);
  Serial.print(cm1);
  Serial.print(",");
  Serial.print(cm2);
  Serial.print(",");
  Serial.print(cm3);
  Serial.print(",");
  Serial.println(cm4);

  // Lectura de letra con la indicación del modo de movimiento requerido
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    key = int(data[0]);
    key_time = millis();
  }
  else{
    loop_time = millis();
    if ((loop_time - key_time) > timeout){
      key = 0;
    }
  }
  switch (key) {
    case 'w':
    {       
    //llamar al subproceso 1
     control.modo1(velocidad); 
    }
      break;
    case 'd':
    {
       //llamar al subproceso 2
      control.modo2(velocidad); 
    }
      break;
    case 'a':
    {
       //llamar al subproceso 3
      control.modo3(velocidad); 
    }
      break;
    case 'h':
    {
       //llamar al subproceso 4
    control.modo4(velocidad); 
    }
      break;
    case 'e':
    {
       //llamar al subproceso 5
      control.modo5(velocidad); 
    }
      break;
    case 'z':
    {
       //llamar al subproceso 6
      control.modo6(velocidad); 
    }
      break;
    case 'y':
    {
       //llamar al subproceso 7
      control.modo7(velocidad); 
    }
      break;
    case 't':
    {
       //llamar al subproceso 8
      control.modo8(velocidad); 
    }
      break;
    case 'q':
    {
       //llamar al subproceso 11
      control.modo11(velocidad); 
    }
      break;
    case 'c':
    {
       //llamar al subproceso 12
      control.modo12(velocidad); 
    }
      break;
    case 'x':
    {
       //llamar al subproceso 13
      control.modo13(velocidad); 
    }
      break;
    case 'g':
    {
       //llamar al subproceso 14
      control.modo14(velocidad); 
    }
      break;
    case 'n':
    {
       //llamar al subproceso 15
      control.modo15(velocidad); 
    }
      break;
    case 'b':
    {
       //llamar al subproceso 16
      control.modo16(velocidad); 
    }
      break;
    default:
    {
      control.off();
         
    }
      break;      
  }
}
