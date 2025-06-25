#include "esp32-hal-ledc.h"
#include "Arduino.h"

#include "Alfredo_NoU3_LSM6.h"
#include "Alfredo_NoU3_MMC5.h"
#include "Alfredo_NoU3_PCA9.h"
#include "Alfredo_NoU3_encoder.h"

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
void taskUpdateIMUs(void* pvParameters){
    while (true) {
        NoU3.updateIMUs();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void taskUpdateServiceLight(void* pvParameters){
    while (true) {
        NoU3.updateServiceLight();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
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
    pca9685.setToFrequency(1526);
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

    xTaskCreatePinnedToCore(taskUpdateIMUs, "taskUpdateIMUs", 4096, NULL, 2, NULL, 1);
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
      acceleration_x -= acceleration_x_offset;
      acceleration_y -= acceleration_y_offset;
      acceleration_z -= acceleration_z_offset;
    }

    if (LSM6.gyroscopeAvailable()) {
      LSM6.readGyroscope(&gyroscope_x, &gyroscope_y, &gyroscope_z);  // Results in rad per second
      gyroscope_x -= gyroscope_x_offset;
      gyroscope_y -= gyroscope_y_offset;
      gyroscope_z -= gyroscope_z_offset;
    }

    updateAngles();
  }

  // Check MMC5983MA for new data
  if (newDataAvailableMMC5) {
    isDataNew = true;
    newDataAvailableMMC5 = false;
    MMC5.clearMeasDoneInterrupt();

    MMC5.readMagnetometer(&magnetometer_x, &magnetometer_y, &magnetometer_z);  // Results in µT (microteslas)
  }

  return isDataNew;
}  

void NoU_Agent::updateAngles()
{
    static unsigned long lastTime = 0;
    
    unsigned long currentTime = millis();

    if (lastTime == 0) {
        lastTime = currentTime;
        return;
    }
        
    float timestep = (currentTime - lastTime) / 1000.0;  // convert ms to seconds
    lastTime = currentTime;

    float deltaPitch = gyroscope_x * timestep;
    float deltaRoll  = gyroscope_y * timestep;
    float deltaYaw   = gyroscope_z * timestep;

    pitch += deltaPitch;
    roll  += deltaRoll;
    yaw   += deltaYaw;
}

void NoU_Agent::calibrateIMUs(float gravity_x, float gravity_y, float gravity_z)
{
    int num_vals = 0;

    float acceleration_x_accumulator = 0;
    float acceleration_y_accumulator = 0;
    float acceleration_z_accumulator = 0;
    float gyroscope_x_accumulator = 0;
    float gyroscope_y_accumulator = 0;
    float gyroscope_z_accumulator = 0;
    
    unsigned long startTime = millis();
    unsigned long calibrationTimeMs = 1000.0;

    while(millis() < startTime + calibrationTimeMs) {
        // Check if data is available and measurements are not complete
        if (NoU3.updateIMUs()) {

            // Store measurements in arrays
            acceleration_x_accumulator += NoU3.acceleration_x;
            acceleration_y_accumulator += NoU3.acceleration_y;
            acceleration_z_accumulator += NoU3.acceleration_z;
            gyroscope_x_accumulator += NoU3.gyroscope_x;
            gyroscope_y_accumulator += NoU3.gyroscope_y;
            gyroscope_z_accumulator += NoU3.gyroscope_z;

            num_vals++;
        }
    }

    // Calculate averages
    acceleration_x_offset = (acceleration_x_accumulator / num_vals) - gravity_x;
    acceleration_y_offset = (acceleration_y_accumulator / num_vals) - gravity_y;
    acceleration_z_offset = (acceleration_z_accumulator / num_vals) - gravity_z;
    gyroscope_x_offset = gyroscope_x_accumulator / num_vals;
    gyroscope_y_offset = gyroscope_y_accumulator / num_vals;
    gyroscope_z_offset = gyroscope_z_accumulator / num_vals;

    // Print averages
    //Serial.print("Average values after "); Serial.print(num_vals); Serial.println(" measurements:");
    //Serial.print("Acceleration X (Gs): "); Serial.println(acceleration_x_offset, 3);
    //Serial.print("Acceleration Y (Gs): "); Serial.println(acceleration_y_offset, 3);
    //Serial.print("Acceleration Z (Gs): "); Serial.println(acceleration_z_offset, 3);
    //Serial.print("Gyroscope X (deg/s): "); Serial.println(gyroscope_x_offset, 3);
    //Serial.print("Gyroscope Y (deg/s): "); Serial.println(gyroscope_y_offset, 3);
    //Serial.print("Gyroscope Z (deg/s): "); Serial.println(gyroscope_z_offset, 3);
}

void NoU_Agent::beginServiceLight()
{
    ledcAttachChannel(RSL_PIN, RSL_PWM_FREQ, RSL_PWM_RES, RSL_CHANNEL);
    setServiceLight(LIGHT_DISABLED);
    updateServiceLight();

    xTaskCreatePinnedToCore(taskUpdateServiceLight, "taskUpdateServiceLight", 4096, NULL, 2, NULL, 1);
}

void NoU_Agent::setServiceLight(serviceLightState state)
{
    stateServiceLight = state;
}

void NoU_Agent::updateServiceLight()
{
    int dutyLED = 0;

    switch (stateServiceLight)
    {
    case LIGHT_OFF:
        dutyLED = 1;
        break;
    case LIGHT_ON:
        dutyLED = (1 << RSL_PWM_RES) - 1;
        break;
    case LIGHT_ENABLED:
        dutyLED = millis() % 1000 < 500 ? (millis() % 500) * 2 : (500 - (millis() % 500)) * 2;
        break;
    case LIGHT_DISABLED:
        dutyLED = (1 << RSL_PWM_RES) - 1;
        break;
    }
    //Serial.println(dutyLED);
    ledcWrite(RSL_PIN, dutyLED);
}

NoU_Motor::NoU_Motor(uint8_t motorPort)
{
    this->motorPort = motorPort;
    set(0);
}

void NoU_Motor::beginEncoder(int8_t pinA, int8_t pinB)
{
    //the 6 pin pairs are for M2-M7. M1 and M8 do not have encoders
    const uint8_t encoderPinMap[6][2] = {
        {17, 18},  // M2, E2
        {15, 16},  // M3, E3
        {10, 11},  // M4, E5
        {41, 42},  // M5, E4
        {40, 39},  // M6, E6
        {38, 37}   // M7, E1
    };

    if (pinA == -1 || pinB == -1){
        if (this->motorPort < 2 || this->motorPort > 7){
            return;
        }

        pinA = encoderPinMap[this->motorPort - 2][0];
        pinB = encoderPinMap[this->motorPort - 2][1];
    }

    this->_encoder.begin(pinA, pinB);
}

int32_t NoU_Motor::getPosition()
{
return this->_encoder.getPosition();
}

void NoU_Motor::set(float output)
{
    uint8_t portMap[8][2] = {{4, 5}, {6, 7}, {8, 9}, {10, 11}, {14, 15}, {12, 13}, {2, 3}, {0, 1}};

    float motorPower = applyCurve(output);
    if (inverted)
        motorPower = motorPower * -1;
    
    double pinZeroDuty = 0;
    double pinOneDuty = 0;

    if (brakeMode){
        if (motorPower <= 0) {
            pinZeroDuty = ((abs(motorPower)*-1)+1) * 100;
            pinOneDuty = 100;
        } else {
            pinZeroDuty = 100;
            pinOneDuty = ((abs(motorPower)*-1)+1) * 100;
        }
    } else {
        if (motorPower >= 0) {
            pinZeroDuty = abs(motorPower * 100);
            pinOneDuty = 0;
        } else {
            pinZeroDuty = 0;
            pinOneDuty = abs(motorPower * 100);
        }
    }

    pca9685.setChannelDutyCycle(portMap[this->motorPort - 1][0], pinZeroDuty);
    pca9685.setChannelDutyCycle(portMap[this->motorPort - 1][1], pinOneDuty);
}

float NoU_Motor::applyCurve(float input)
{
    float sign = (input == 0) ? 0 : (input > 0 ? 1 : -1);
    float x = fabs(input);

    if (x < deadband)
        return 0;

    float curved = pow(max(fmap(constrain(x, 0, 1), deadband, 1, 0, 1), 0.0f), exponent);
    float output = fmap(curved, 0, 1, minimumOutput, maximumOutput);

    return output * sign;
}


void NoU_Motor::setMotorCurve(float minimumOutput, float maximumOutput, float deadband, float exponent)
{
    setMinimumOutput(minimumOutput);
    setMaximumOutput(maximumOutput);
    setDeadband(deadband);
    setExponent(exponent);
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

void NoU_Motor::setInverted(boolean isInverted)
{
    this->inverted = isInverted;
}

void NoU_Motor::setBrakeMode(boolean isBrakeMode)
{
    this->brakeMode = isBrakeMode;
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
      rearLeftMotor(), rearRightMotor(), drivetrainType(DRIVE_TWO_MOTORS)
{
}

NoU_Drivetrain::NoU_Drivetrain(NoU_Motor *frontLeftMotor, NoU_Motor *frontRightMotor,
                               NoU_Motor *rearLeftMotor, NoU_Motor *rearRightMotor)
    : frontLeftMotor(frontLeftMotor), frontRightMotor(frontRightMotor),
      rearLeftMotor(rearLeftMotor), rearRightMotor(rearRightMotor), drivetrainType(DRIVE_FOUR_MOTORS)
{
}


void NoU_Drivetrain::setMotors(float frontLeftPower, float frontRightPower, float rearLeftPower, float rearRightPower)
{
    if (drivetrainType == DRIVE_FOUR_MOTORS)
    {
        rearLeftMotor->set(rearLeftPower);
        rearRightMotor->set(rearRightPower);
    }
    frontLeftMotor->set(frontLeftPower);
    frontRightMotor->set(frontRightPower);
}

void NoU_Drivetrain::tankDrive(float leftPower, float rightPower)
{
    setMotors(leftPower, rightPower, leftPower, rightPower);
}

void NoU_Drivetrain::arcadeDrive(float throttle, float rotation, boolean invertedReverse)
{
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

void NoU_Drivetrain::holonomicDrive(float xVelocity, float yVelocity, float rotation, bool plusConfig)
{
    if (drivetrainType == DRIVE_TWO_MOTORS)
        return;
    
    float frontLeftPower = 0;
    float frontRightPower = 0;
    float rearLeftPower = 0;
    float rearRightPower = 0;

    if(plusConfig){
        frontLeftPower = yVelocity - rotation;
        frontRightPower = xVelocity - rotation;
        rearLeftPower = -xVelocity - rotation;
        rearRightPower = -yVelocity - rotation;
    } else { //X config
        frontLeftPower = xVelocity + yVelocity - rotation;
        frontRightPower = xVelocity - yVelocity - rotation;
        rearLeftPower = -xVelocity + yVelocity - rotation;
        rearRightPower = -xVelocity - yVelocity - rotation;
    }

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


void NoU_Drivetrain::setMotorCurves(float minimumOutput, float maximumOutput, float deadband, float exponent)
{
    setMinimumOutput(minimumOutput);
    setMaximumOutput(maximumOutput);
    setDeadband(deadband);
    setExponent(exponent);
}

void NoU_Drivetrain::setMinimumOutput(float minimumOutput)
{
    if (drivetrainType == DRIVE_FOUR_MOTORS)
    {
        rearLeftMotor->setMinimumOutput(minimumOutput);
        rearRightMotor->setMinimumOutput(minimumOutput);
    }
    frontLeftMotor->setMinimumOutput(minimumOutput);
    frontRightMotor->setMinimumOutput(minimumOutput);
}

void NoU_Drivetrain::setMaximumOutput(float maximumOutput)
{
    if (drivetrainType == DRIVE_FOUR_MOTORS)
    {
        rearLeftMotor->setMaximumOutput(maximumOutput);
        rearRightMotor->setMaximumOutput(maximumOutput);
    }
    frontLeftMotor->setMaximumOutput(maximumOutput);
    frontRightMotor->setMaximumOutput(maximumOutput);
}

void NoU_Drivetrain::setExponent(float exponent)
{
    if (drivetrainType == DRIVE_FOUR_MOTORS)
    {
        rearLeftMotor->setExponent(exponent);
        rearRightMotor->setExponent(exponent);
    }
    frontLeftMotor->setExponent(exponent);
    frontRightMotor->setExponent(exponent);
}

void NoU_Drivetrain::setDeadband(float deadband)
{
    if (drivetrainType == DRIVE_FOUR_MOTORS)
    {
        rearLeftMotor->setDeadband(deadband);
        rearRightMotor->setDeadband(deadband);
    }
    frontLeftMotor->setDeadband(deadband);
    frontRightMotor->setDeadband(deadband);
}
