#include <Alfredo_NoU3.h>

#include <Adafruit_Sensor_Calibration.h>
#include <Adafruit_AHRS.h>

// pick your filter! slower == better quality output
//Adafruit_NXPSensorFusion filter; // slowest
Adafruit_Madgwick filter;  // faster than NXP
//Adafruit_Mahony filter;  // fastest/smalleset
Adafruit_Sensor_Calibration_EEPROM cal;
#define FILTER_UPDATE_RATE_HZ 80
#define PRINT_EVERY_N_UPDATES 10

//#define AHRS_DEBUG_OUTPUT

uint32_t timestamp;

float acl_x = 0, acl_y = 0, acl_z = 0;
float gyr_x = 0, gyr_y = 0, gyr_z = 0;
float mag_x = 0, mag_y = 0, mag_z = 0;

float degToRad(float degPerSec) {
    const float DEG_TO_RAD_CONVERSION = 3.14159265358979323846 / 180.0;
    return degPerSec * DEG_TO_RAD_CONVERSION;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;  // Wait here until the Serial Monitor is opened
  }

  NoU3.beginMotors();
  NoU3.beginIMUs();

  if ((millis() - timestamp) < (1000 / FILTER_UPDATE_RATE_HZ)) return;
  timestamp = millis();

  if (!cal.begin()) {
    Serial.println("Failed to initialize calibration helper");
  } else if (!cal.loadCalibration()) {
    Serial.println("No calibration loaded/found");
  }

  Serial.println("starting filter");
  filter.begin(FILTER_UPDATE_RATE_HZ);
  timestamp = millis();

  Serial.println("starting loop");
}

void loop() {

  float roll, pitch, heading;
  static uint8_t counter = 0;

  NoU3.updateIMUs();

  if ((millis() - timestamp) > (1000 / FILTER_UPDATE_RATE_HZ)){
    timestamp = millis();

    acl_x = NoU3.acceleration_x;
    acl_y = NoU3.acceleration_y;
    acl_z = NoU3.acceleration_z;
    gyr_x = NoU3.gyroscope_x;
    gyr_y = NoU3.gyroscope_y;
    gyr_z = NoU3.gyroscope_z;
    mag_x = NoU3.magnetometer_x;
    mag_y = NoU3.magnetometer_y;
    mag_z = NoU3.magnetometer_z;

    //hard iron calibration
    float mx = mag_x - cal.mag_hardiron[0];
    float my = mag_y - cal.mag_hardiron[1];
    float mz = mag_z - cal.mag_hardiron[2];

    // soft iron calibration
    mag_x = mx * cal.mag_softiron[0] + my * cal.mag_softiron[1] + mz * cal.mag_softiron[2];
    mag_y = mx * cal.mag_softiron[3] + my * cal.mag_softiron[4] + mz * cal.mag_softiron[5];
    mag_z = mx * cal.mag_softiron[6] + my * cal.mag_softiron[7] + mz * cal.mag_softiron[8];

    // gyro calibration
    gyr_x -= cal.gyro_zerorate[0];
    gyr_y -= cal.gyro_zerorate[1];
    gyr_z -= cal.gyro_zerorate[2];

    // accel calibration
    //mag_x -= cal.accel_zerog[0];
    //mag_y -= cal.accel_zerog[1];
    //mag_z -= cal.accel_zerog[2];

    // Update the SensorFusion filter
    filter.update(gyr_x * 0.0174533, gyr_y * 0.0174533, gyr_z * 0.0174533, acl_x, acl_y, acl_z, mag_x, mag_y, mag_z);

    // only print the calculated output once in a while
    if (counter++ >= PRINT_EVERY_N_UPDATES) {

      // reset the counter
      counter = 0;

      // print the heading, pitch and roll
      roll = filter.getRoll();
      pitch = filter.getPitch();
      heading = filter.getYaw();
      Serial.print("Orientation: ");
      Serial.print(heading);
      Serial.print(", ");
      Serial.print(pitch);
      Serial.print(", ");
      Serial.println(roll);

      /*
      float qw, qx, qy, qz;
      filter.getQuaternion(&qw, &qx, &qy, &qz);
      Serial.print("Quaternion: ");
      Serial.print(qw, 4);
      Serial.print(", ");
      Serial.print(qx, 4);
      Serial.print(", ");
      Serial.print(qy, 4);
      Serial.print(", ");
      Serial.println(qz, 4);
      */
    }
  }
}
