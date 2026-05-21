/**
 * Example code for a robot using a NoU3, meant to test every motor and servo.
 * The NoU3 documentation and tutorials can be found at https://alfredo-nou3.readthedocs.io/
 */

#include <Alfredo_NoU3.h>

NoU_Motor motor1(1);
NoU_Motor motor2(2);
NoU_Motor motor3(3);
NoU_Motor motor4(4);
NoU_Motor motor5(5);
NoU_Motor motor6(6);
NoU_Motor motor7(7);
NoU_Motor motor8(8);

NoU_Servo servo1(1);
NoU_Servo servo2(2);
NoU_Servo servo3(3);
NoU_Servo servo4(4);

//FVT mode is used for "Functional Verification testing"
//This is used internally by Alfredo to test all NoU3s before they pass QA testing

//#define FVT_MODE

#ifdef FVT_MODE
bool startTest = false;
float motorPeriod = 0.05;
#else
bool startTest = true;
float motorPeriod = 0.2;
#endif

void setup() {
    NoU3.begin();
    Serial.begin(115200);

    pinMode(0, INPUT_PULLUP);
    delay(1000);
}

void loop() {
  if(digitalRead(0) != 1){
    startTest = true;
  }

  if(!startTest){
    NoU3.stopMotors();
  } else {
    NoU3.setServiceLight(LIGHT_ENABLED);
    
    for (float i = -1; i < 1; i += motorPeriod) {
        motor1.set(i);
        motor2.set(i);
        motor3.set(i);
        motor4.set(i);
        motor5.set(i);
        motor6.set(i);
		    motor7.set(i);
        motor8.set(i);
		
        servo1.write(map(i*100.0, -100, 100, 0, 180));
        servo2.write(map(i*100.0, -100, 100, 0, 180));
        servo3.write(map(i*100.0, -100, 100, 0, 180));
        servo4.write(map(i*100.0, -100, 100, 0, 180));

        delay(5);
        Serial.println(i);
    }
    for (float i = 1; i > -1; i -= motorPeriod) {
        motor1.set(i);
        motor2.set(i);
        motor3.set(i);
        motor4.set(i);
        motor5.set(i);
        motor6.set(i);
		    motor7.set(i);
        motor8.set(i);
		
        servo1.write(map(i*100.0, -100, 100, 0, 180));
        servo2.write(map(i*100.0, -100, 100, 0, 180));
        servo3.write(map(i*100.0, -100, 100, 0, 180));
        servo4.write(map(i*100.0, -100, 100, 0, 180));
		
        delay(5);
        Serial.println(i);
    }
  }
}