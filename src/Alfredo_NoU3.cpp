#include "esp32-hal-ledc.h"
#include "Arduino.h"

#include "Alfredo_NoU3_LSM6.h"
#include "Alfredo_NoU3_MMC5.h"
#include "Alfredo_NoU3_PCA9.h"

#include "Alfredo_NoU3.h"

PCA9685 pca9685;

LSM6DSOXClass LSM6;
SFE_MMC5983MA MMC5;

NoU_Agent NoU3;

float fmap(float val, float in_min, float in_max, float out_min, float out_max)
{
    return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

volatile bool newDataAvailableLSM6 = true;
volatile bool newDataAvailableMMC5 = true;
void interruptRoutineLSM6() {
  newDataAvailableLSM6 = true;
}
void interruptRoutineMMC5() {
  newDataAvailableMMC5 = true;
}

void NoU_Agent::begin()
{
    Wire.begin(PIN_I2C_SDA_QWIIC, PIN_I2C_SCL_QWIIC, 400000);

    Wire1.begin(PIN_I2C_SDA_IMU, PIN_I2C_SCL_IMU, 400000);
    beginMotors();
    beginIMUs();
    beginServiceLight();
}

void NoU_Agent::beginMotors()
{
    pca9685.setupSingleDevice(Wire1, 0x40);
    pca9685.setupOutputEnablePin(12);
    pca9685.enableOutputs(12);
    pca9685.setToFrequency(1500);
}

void NoU_Agent::beginIMUs()
{
    // Initialize LSM6
    if (LSM6.begin(Wire1) == false)
    {
        Serial.println("LSM6 did not respond.");
    } else {
        //Serial.println("LSM6 is good.");
    }
    pinMode(PIN_INTERRUPT_LSM6, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_INTERRUPT_LSM6), interruptRoutineLSM6, RISING);
    LSM6.enableInterrupt(); // LSM6 collects readings at 104 hz

    // Initialize MMC5
    if (MMC5.begin(Wire1) == false)
    {
        Serial.println("MMC5 did not respond.");
    } else {
        //Serial.println("MMC5 is good.");
    }
    MMC5.softReset();
    MMC5.setFilterBandwidth(800);
    MMC5.setContinuousModeFrequency(100); // Allowed values are 1000, 200, 100, 50, 20, 10, 1 and 0
    MMC5.enableAutomaticSetReset();
    MMC5.enableContinuousMode();
    pinMode(PIN_INTERRUPT_MMC5, INPUT);
    attachInterrupt(digitalPinToInterrupt(PIN_INTERRUPT_MMC5), interruptRoutineMMC5, RISING);
    MMC5.enableInterrupt();
}

bool NoU_Agent::updateIMUs()
{
  bool isDataNew = false;
  // Check LSM6 for new data
  if (newDataAvailableLSM6) {
    isDataNew = true;
    newDataAvailableLSM6 = false;

    if (LSM6.accelerationAvailable()) {
      LSM6.readAcceleration(&acceleration_x, &acceleration_y, &acceleration_z);// result in Gs
    }

    if (LSM6.gyroscopeAvailable()) {
      LSM6.readGyroscope(&gyroscope_x, &gyroscope_y, &gyroscope_z);  // Results in degrees per second

    }
  }

  // Check MMC5983MA for new data
  if (newDataAvailableMMC5) {
    isDataNew = true;
    newDataAvailableMMC5 = false;
    MMC5.clearMeasDoneInterrupt();

    MMC5.readAccelerometer(&magnetometer_x, &magnetometer_y, &magnetometer_z);  // Results in µT (microteslas)
  }

  return isDataNew;
}   

void NoU_Agent::beginServiceLight()
{
    ledcAttachChannel(RSL_PIN, RSL_PWM_FREQ, RSL_PWM_RES, RSL_CHANNEL);
    setServiceLight(LIGHT_DISABLED);
    updateServiceLight();
}

void NoU_Agent::setServiceLight(serviceLightState state)
{
    stateServiceLight = state;
}

void NoU_Agent::updateServiceLight()
{
    switch (stateServiceLight)
    {
    case LIGHT_OFF:
        ledcWrite(RSL_PIN, 1);
        break;
    case LIGHT_ON:
        ledcWrite(RSL_PIN, (1 << RSL_PWM_RES) - 1);
        break;
    case LIGHT_ENABLED:
        ledcWrite(RSL_PIN, millis() % 1000 < 500 ? (millis() % 500) * 2 : (500 - (millis() % 500)) * 2);
        break;
    case LIGHT_DISABLED:
        ledcWrite(RSL_PIN, (1 << RSL_PWM_RES) - 1);
        break;
    }
}

NoU_Motor::NoU_Motor(uint8_t motorPort)
{
    this->motorPort = motorPort;
    set(0);
}

void NoU_Motor::set(float output)
{
    uint8_t portMap[8][2] = {{4, 5}, {6, 7}, {8, 9}, {10, 11}, {14, 15}, {12, 13}, {2, 3}, {0, 1}};

    float motorPower = applyCurve(output);
    if (inverted)
        motorPower = motorPower * -1;

    if(motorPower==0)
    {
        if(brakeOn){
            pca9685.setChannelDutyCycle(portMap[this->motorPort - 1][0], 100);
            pca9685.setChannelDutyCycle(portMap[this->motorPort - 1][1], 100);
        }else{
            pca9685.setChannelDutyCycle(portMap[this->motorPort - 1][0], 0);
            pca9685.setChannelDutyCycle(portMap[this->motorPort - 1][1], 0);
        }
    } else if (motorPower > 0)
    {
        pca9685.setChannelDutyCycle(portMap[this->motorPort - 1][0], abs(motorPower * 100));
        pca9685.setChannelDutyCycle(portMap[this->motorPort - 1][1], 0);
    } else
    {
        pca9685.setChannelDutyCycle(portMap[this->motorPort - 1][0], 0);
        pca9685.setChannelDutyCycle(portMap[this->motorPort - 1][1], abs(motorPower * 100));
    }
}

float NoU_Motor::applyCurve(float input)
{
    return fmap((fabs(input) < deadband ? 0 : 1)                                                        // apply deadband
                    * pow(max(fmap(constrain(fabs(input), -1, 1), deadband, 1, 0, 1), 0.0f), exponent), // account for deadband, apply exponent
                0, 1, minimumOutput, maximumOutput)                                                     // apply minimum and maximum output limits
           * (input == 0 ? 0 : input > 0 ? 1
                                         : -1) // apply original sign
           * (inverted ? -1 : 1);              // account for inversion
}

void NoU_Motor::setMinimumOutput(float minimumOutput)
{
    minimumOutput = constrain(minimumOutput, 0, maximumOutput);
    this->minimumOutput = minimumOutput;
}

void NoU_Motor::setMaximumOutput(float maximumOutput)
{
    maximumOutput = constrain(maximumOutput, minimumOutput, 1);
    this->maximumOutput = maximumOutput;
}

void NoU_Motor::setDeadband(float deadband)
{
    deadband = constrain(deadband, 0, 1);
    this->deadband = deadband;
}

void NoU_Motor::setExponent(float exponent)
{
    exponent = max(0.0f, exponent);
    this->exponent = exponent;
}

void NoU_Motor::setInverted(boolean inverted)
{
    this->inverted = inverted;
}

void NoU_Motor::setBrake(uint8_t brake)
{
    switch(brake){
        case BRAKE:
            brakeOn = true;
            break;
        case RELEASE:
            brakeOn = false;
            break;
    }
}

void NoU_Motor::setState(uint8_t state)
{
    switch(state){
        case BRAKE:
            setBrake(BRAKE);
            set(0);
            break;
        case RELEASE:
            setBrake(RELEASE);
            set(0);
            break;
    }
}


NoU_Servo::NoU_Servo(uint8_t servoPort, uint16_t minPulse, uint16_t maxPulse)
{
    switch (servoPort)
    {
    case 1:
        pin = PIN_SERVO_1;
        channel = SERVO_1_CHANNEL;
        break;
    case 2:
        pin = PIN_SERVO_2;
        channel = SERVO_2_CHANNEL;
        break;
    case 3:
        pin = PIN_SERVO_3;
        channel = SERVO_3_CHANNEL;
        break;
    case 4:
        pin = PIN_SERVO_4;
        channel = SERVO_4_CHANNEL;
        break;
    case 5:
        pin = PIN_SERVO_5;
        channel = SERVO_5_CHANNEL;
        break;
    case 6:
        pin = PIN_SERVO_6;
        channel = SERVO_6_CHANNEL;
        break;
    }
    this->minPulse = minPulse;
    this->maxPulse = maxPulse;
    ledcAttachChannel(pin, SERVO_PWM_FREQ, SERVO_PWM_RES, channel);
}

void NoU_Servo::write(float degrees)
{
    writeMicroseconds(fmap(degrees, 0, 180, minPulse, maxPulse));
}

void NoU_Servo::writeMicroseconds(uint16_t pulseLength)
{
    this->pulse = pulseLength;
    ledcWrite(pin, fmap(pulseLength, 0, 20000, 0, (1 << SERVO_PWM_RES) - 1));
}

void NoU_Servo::setMinimumPulse(uint16_t minPulse)
{
    this->minPulse = minPulse;
}

void NoU_Servo::setMaximumPulse(uint16_t maxPulse)
{
    this->maxPulse = maxPulse;
}

uint16_t NoU_Servo::getMicroseconds()
{
    return pulse;
}

float NoU_Servo::getDegrees()
{
    return fmap(pulse, minPulse, maxPulse, 0, 180);
}

NoU_Drivetrain::NoU_Drivetrain(NoU_Motor *leftMotor, NoU_Motor *rightMotor)
    : frontLeftMotor(leftMotor), frontRightMotor(rightMotor),
      rearLeftMotor(), rearRightMotor(), drivetrainType(TWO_MOTORS)
{
}

NoU_Drivetrain::NoU_Drivetrain(NoU_Motor *frontLeftMotor, NoU_Motor *frontRightMotor,
                               NoU_Motor *rearLeftMotor, NoU_Motor *rearRightMotor)
    : frontLeftMotor(frontLeftMotor), frontRightMotor(frontRightMotor),
      rearLeftMotor(rearLeftMotor), rearRightMotor(rearRightMotor), drivetrainType(FOUR_MOTORS)
{
}

float NoU_Drivetrain::applyInputCurve(float input)
{
    return (fabs(input) < inputDeadband ? 0 : 1)                                                        // apply deadband
           * pow(max(fmap(constrain(fabs(input), -1, 1), inputDeadband, 1, 0, 1), 0.0f), inputExponent) // account for deadband, apply exponent
           * (input > 0 ? 1 : -1);                                                                      // apply original sign
}

void NoU_Drivetrain::setMotors(float frontLeftPower, float frontRightPower, float rearLeftPower, float rearRightPower)
{
    if (drivetrainType == FOUR_MOTORS)
    {
        rearLeftMotor->set(rearLeftPower);
        rearRightMotor->set(rearRightPower);
    }
    frontLeftMotor->set(frontLeftPower);
    frontRightMotor->set(frontRightPower);
}

void NoU_Drivetrain::tankDrive(float leftPower, float rightPower)
{
    leftPower = applyInputCurve(leftPower);
    rightPower = applyInputCurve(rightPower);
    setMotors(leftPower, rightPower, leftPower, rightPower);
}

void NoU_Drivetrain::arcadeDrive(float throttle, float rotation, boolean invertedReverse)
{
    throttle = applyInputCurve(throttle);
    rotation = applyInputCurve(rotation);
    float leftPower = 0;
    float rightPower = 0;
    float maxInput = (throttle > 0 ? 1 : -1) * max(fabs(throttle), fabs(rotation));
    if (throttle > 0)
    {
        if (rotation > 0)
        {
            leftPower = maxInput;
            rightPower = throttle - rotation;
        }
        else
        {
            leftPower = throttle + rotation;
            rightPower = maxInput;
        }
    }
    else
    {
        if (rotation > 0)
        {
            leftPower = invertedReverse ? maxInput : throttle + rotation;
            rightPower = invertedReverse ? throttle + rotation : maxInput;
        }
        else
        {
            leftPower = invertedReverse ? throttle - rotation : maxInput;
            rightPower = invertedReverse ? maxInput : throttle - rotation;
        }
    }
    setMotors(leftPower, rightPower, leftPower, rightPower);
}

void NoU_Drivetrain::curvatureDrive(float throttle, float rotation, boolean isQuickTurn)
{
    throttle = applyInputCurve(throttle);
    rotation = applyInputCurve(rotation);

    float angularPower;
    boolean overPower;

    if (isQuickTurn)
    {
        if (fabs(throttle) < quickStopThreshold)
        {
            quickStopAccumulator = (1 - quickStopAlpha) * quickStopAccumulator + quickStopAlpha * rotation * 2;
        }
        overPower = true;
        angularPower = rotation;
    }
    else
    {
        overPower = false;
        angularPower = fabs(throttle) * rotation - quickStopAccumulator;

        if (quickStopAccumulator > 1)
            quickStopAccumulator--;
        else if (quickStopAccumulator < -1)
            quickStopAccumulator++;
        else
            quickStopAccumulator = 0;
    }

    float leftPower;
    float rightPower;

    leftPower = throttle + angularPower;
    rightPower = throttle - angularPower;

    if (overPower)
    {
        if (leftPower > 1)
        {
            rightPower -= leftPower - 1;
            leftPower = 1;
        }
        else if (rightPower > 1)
        {
            leftPower -= rightPower - 1;
            rightPower = 1;
        }
        else if (leftPower < -1)
        {
            rightPower -= leftPower + 1;
            leftPower = -1;
        }
        else if (rightPower < -1)
        {
            leftPower -= rightPower + 1;
            rightPower = -1;
        }
    }
    float maxMagnitude = max(fabs(leftPower), fabs(rightPower));
    if (maxMagnitude > 1)
    {
        leftPower /= maxMagnitude;
        rightPower /= maxMagnitude;
    }
    setMotors(leftPower, rightPower, leftPower, rightPower);
}

void NoU_Drivetrain::holonomicDrive(float xVelocity, float yVelocity, float rotation)
{
    if (drivetrainType == TWO_MOTORS)
        return;
    xVelocity = applyInputCurve(xVelocity);
    yVelocity = applyInputCurve(yVelocity);
    rotation = applyInputCurve(rotation);
    float frontLeftPower = xVelocity + yVelocity + rotation;
    float frontRightPower = -xVelocity + yVelocity - rotation;
    float rearLeftPower = -xVelocity + yVelocity + rotation;
    float rearRightPower = xVelocity + yVelocity - rotation;
    float maxMagnitude = max(fabs(frontLeftPower), max(fabs(frontRightPower), max(fabs(rearLeftPower), fabs(rearRightPower))));
    if (maxMagnitude > 1)
    {
        frontLeftPower /= maxMagnitude;
        frontRightPower /= maxMagnitude;
        rearLeftPower /= maxMagnitude;
        rearRightPower /= maxMagnitude;
    }
    setMotors(frontLeftPower, frontRightPower, rearLeftPower, rearRightPower);
}

void NoU_Drivetrain::setMinimumOutput(float minimumOutput)
{
    if (drivetrainType == FOUR_MOTORS)
    {
        rearLeftMotor->setMinimumOutput(minimumOutput);
        rearRightMotor->setMinimumOutput(minimumOutput);
    }
    frontLeftMotor->setMinimumOutput(minimumOutput);
    frontRightMotor->setMinimumOutput(minimumOutput);
}

void NoU_Drivetrain::setMaximumOutput(float maximumOutput)
{
    if (drivetrainType == FOUR_MOTORS)
    {
        rearLeftMotor->setMaximumOutput(maximumOutput);
        rearRightMotor->setMaximumOutput(maximumOutput);
    }
    frontLeftMotor->setMaximumOutput(maximumOutput);
    frontRightMotor->setMaximumOutput(maximumOutput);
}

void NoU_Drivetrain::setInputExponent(float inputExponent)
{
    inputExponent = max(0.0f, inputExponent);
    this->inputExponent = inputExponent;
}

void NoU_Drivetrain::setInputDeadband(float inputDeadband)
{
    inputDeadband = constrain(inputDeadband, 0, 1);
    this->inputDeadband = inputDeadband;
}
