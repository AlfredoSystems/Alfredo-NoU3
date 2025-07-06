/**
 * Example code for a robot using a NoU3 controlled with PestoLink: https://pestol.ink
 * The NoU3 documentation and tutorials can be found at https://alfredo-nou3.readthedocs.io/
 */

#include <PestoLink-Receive.h> // >=1.0.8
#include <Alfredo_NoU3.h>

// If your robot has more than a drivetrain, add those actuators here 
NoU_Motor frontLeftMotor(1);
NoU_Motor frontRightMotor(2);
NoU_Motor rearLeftMotor(3);
NoU_Motor rearRightMotor(4);

// This creates the drivetrain object, you shouldn't have to mess with this
NoU_Drivetrain drivetrain(&frontLeftMotor, &frontRightMotor, &rearLeftMotor, &rearRightMotor);

//The gyroscope sensor is by default precise, but not accurate. This is fixable by adjusting the angular scale factor.
//Tuning procedure: 
//Rotate the robot in place 5 times. Use the Serial printout to read the current gyro angle in Radians, we will call this "measured_angle".
//measured_angle should be nearly 31.416 which is 5*2*pi. Update measured_angle below to complete the tuning process. 
float measured_angle = 31.416;
float angular_scale = (5.0*2.0*PI) / measured_angle;

void setup() {
    //EVERYONE SHOULD CHANGE "NoU3_Bluetooth" TO THE NAME OF THEIR ROBOT HERE BEFORE PAIRING THEIR ROBOT TO ANY LAPTOP
    PestoLink.begin("NoU3_Bluetooth");
    Serial.begin(115200);

    NoU3.begin();
    
    NoU3.calibrateIMUs(); // this takes exactly one second. Do not move the robot during calibration.

    frontLeftMotor.setInverted(true);
    rearLeftMotor.setInverted(true);
}

unsigned long lastPrintTime = 0;

void loop() {
    NoU3.updateIMUs();
    if (lastPrintTime + 100 < millis()){
        Serial.printf("gyro yaw (radians): %.3f\r\n",  NoU3.yaw * angular_scale );
        lastPrintTime = millis();
    }

    // This measures your batteries voltage and sends it to PestoLink
    float batteryVoltage = NoU3.getBatteryVoltage();
    PestoLink.printBatteryVoltage(batteryVoltage);

    if (PestoLink.isConnected()) {
        float fieldPowerX = -PestoLink.getAxis(1);
        float fieldPowerY = PestoLink.getAxis(0);
        float rotationPower = -PestoLink.getAxis(2);

        // Get robot heading (in radians) from the gyro
        float heading = NoU3.yaw * angular_scale;

        // Rotate joystick vector to be robot-centric
        float cosA = cos(heading);
        float sinA = sin(heading);

        float robotPowerX = fieldPowerX * cosA + fieldPowerY * sinA;
        float robotPowerY = -fieldPowerX * sinA + fieldPowerY * cosA;

        //set motor power
        drivetrain.holonomicDrive(robotPowerX, robotPowerY, rotationPower);

        NoU3.setServiceLight(LIGHT_ENABLED);
    } else {
        NoU3.setServiceLight(LIGHT_DISABLED);
        drivetrain.holonomicDrive(0, 0, 0); // stop motors if connection is lost
    }
}
