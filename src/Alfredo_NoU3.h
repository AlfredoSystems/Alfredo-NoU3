#ifndef ALFREDO_NOU3_H
#define ALFREDO_NOU3_H

#include <inttypes.h>

#include "Alfredo_NoU3_LSM6.h"
#include "Alfredo_NoU3_MMC5.h"

const int PIN_SNS_VERSION = 1;
const int PIN_SNS_VIN = 2;

const int PIN_SERVO_1 = 4;
const int PIN_SERVO_2 = 5;
const int PIN_SERVO_3 = 6;
const int PIN_SERVO_4 = 7;
const int PIN_SERVO_5 = 8;
const int PIN_SERVO_6 = 9;

const int PIN_I2C_SDA_QWIIC = 33;
const int PIN_I2C_SCL_QWIIC = 34;
const int PIN_I2C_SDA_IMU = 35;
const int PIN_I2C_SCL_IMU = 36;

const int RSL_PIN = 45;

const int PIN_INTERRUPT_LSM6 = 48;
const int PIN_INTERRUPT_MMC5 = 47;

// PWM Configuration
const int SERVO_PWM_RES = 12; // bits
const int SERVO_PWM_FREQ = 50; // Hz
const int RSL_PWM_RES = 10; // bits
const int RSL_PWM_FREQ = 1000; // Hz

const int RSL_CHANNEL = 1;
const int SERVO_1_CHANNEL = 2;
const int SERVO_2_CHANNEL = 3;
const int SERVO_3_CHANNEL = 4;
const int SERVO_4_CHANNEL = 5;
const int SERVO_5_CHANNEL = 6;
const int SERVO_6_CHANNEL = 7;

#define BRAKE 3
#define RELEASE 4

typedef enum {
    LIGHT_OFF,
    LIGHT_ON,
    LIGHT_DISABLED,
    LIGHT_ENABLED
} serviceLightState;

// Drivetrain configurations (TODO: This should be an enum?)
#define TWO_MOTORS 0
#define FOUR_MOTORS 1

class NoU_Agent {
    public:
		void begin();

		void beginMotors();
		void beginIMUs();
        bool updateIMUs();

		float getBatteryVoltage(){ return analogReadMilliVolts(PIN_SNS_VIN) * 0.001 * 7.818; };
		float getVersionVoltage(){ return analogReadMilliVolts(PIN_SNS_VERSION) * 0.001 ; };

        void beginServiceLight();
        void setServiceLight(serviceLightState state);
        void updateServiceLight();

        float acceleration_x, acceleration_y, acceleration_z;
        float gyroscope_x, gyroscope_y, gyroscope_z;
        float magnetometer_x, magnetometer_y, magnetometer_z;
        
        serviceLightState stateServiceLight;
};

class NoU_Motor {
    public:
        NoU_Motor(uint8_t motorPort);
        void set(float output);
        void setInverted(boolean isInverted);
        void setMinimumOutput(float minimumOutput);
        void setMaximumOutput(float maximumOutput);
        void setExponent(float exponent);
        void setDeadband(float deadband);
        void setBrake(uint8_t brake);
        void setState(uint8_t state);
    private:
        float applyCurve(float output);
        uint8_t motorPort;
        bool inverted = false;
        float minimumOutput = 0;
        float maximumOutput = 1;
        float exponent = 1;
        float deadband = 0;
        bool brakeOn = false;
};

class NoU_Servo {
    public:
        NoU_Servo(uint8_t servoPort, uint16_t minPulse = 540, uint16_t maxPulse = 2300);
        void write(float degrees);
        void writeMicroseconds(uint16_t pulseLength);
        void setMinimumPulse(uint16_t minPulse);
        void setMaximumPulse(uint16_t maxPulse);
        uint16_t getMicroseconds();
        float getDegrees();
    private:
        uint8_t pin;
        uint8_t channel;
        uint16_t minPulse;
        uint16_t maxPulse;
        uint16_t pulse;
};

class NoU_Drivetrain {
    public:
        NoU_Drivetrain(NoU_Motor* leftMotor, NoU_Motor* rightMotor);
        NoU_Drivetrain(NoU_Motor* frontLeftMotor, NoU_Motor* frontRightMotor,
                        NoU_Motor* rearLeftMotor, NoU_Motor* rearRightMotor);
        void tankDrive(float leftPower, float rightPower);
        void arcadeDrive(float throttle, float rotation, boolean invertedReverse = false);
        void curvatureDrive(float throttle, float rotation, boolean isQuickTurn = true);
        void holonomicDrive(float xVelocity, float yVelocity, float rotation);
        void setMinimumOutput(float minimumOutput);
        void setMaximumOutput(float maximumOutput);
        void setInputExponent(float inputExponent);
        void setInputDeadband(float inputDeadband);
    private:
        void setMotors(float frontLeftPower, float frontRightPower, float rearLeftPower, float rearRightPower);
        float applyInputCurve(float input);
        float applyOutputCurve(float output);
        NoU_Motor *frontLeftMotor;
        NoU_Motor *frontRightMotor;
        NoU_Motor *rearLeftMotor;
        NoU_Motor *rearRightMotor;
        uint8_t drivetrainType;
        float quickStopThreshold = 0.2;
        float quickStopAlpha = 0.1;
        float quickStopAccumulator;
        float minimumOutput = 0;
        float maximumOutput = 1;
        float inputExponent = 1;
        float inputDeadband = 0;
};

extern NoU_Agent NoU3;
extern LSM6DSOXClass LSM6;
extern SFE_MMC5983MA MMC5;

#endif
