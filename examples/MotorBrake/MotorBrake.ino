/**
   Example code for a robot using a NoU3, demonstrating the two brake modes of a motor, using motor 1.
   The NoU3 documentation and tutorials can be found at https://alfredo-nou3.readthedocs.io/
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

const int numStates = 4;
int step = 0;
unsigned long lastStateChangedMillis = 0;

void setup()
{
  NoU3.begin();
  Serial.begin(115200);

  // you can use motor1.setState(RELEASE); or motor.setState(BRAKE); in setup if you want the brake to always be on or off.
}

void loop()
{
  NoU3.updateServiceLight();

  unsigned long currentMillis = millis();
  if (currentMillis - lastStateChangedMillis > 3000) {
    lastStateChangedMillis = currentMillis;
    // runs every 3 seconds
    switch (step) {
      case 0:
        motor1.set(.75);
        NoU3.setServiceLight(LIGHT_ENABLED);
        break;
      case 1:
        motor1.setState(RELEASE); // stop slowly, and set brake mode to RELEASE
        // // equivalent to
        // motor1.setBrake(RELEASE); // sets the brake mode, but has no effect until motor.set(0)
        // motor1.set(0);
        NoU3.setServiceLight(LIGHT_OFF);
        break;
      case 2:
        motor1.set(-1);
        NoU3.setServiceLight(LIGHT_ENABLED);
        break;
      case 3:
        motor1.setState(BRAKE); // stop quickly, and set brake mode to BRAKE
        // // equivalent to
        // motor1.setBrake(BRAKE); // sets the brake mode, but has no effect until motor.set(0)
        // motor1.set(0);
        NoU3.setServiceLight(LIGHT_ON);
        break;
    }

    step++;
    if (step >= numStates) {
      step = 0;
    }

  }
}