#include <Alfredo_NoU3.h>

#include <Adafruit_Sensor_Calibration.h>
Adafruit_Sensor_Calibration_EEPROM cal;

float acl_x = 0, acl_y = 0, acl_z = 0;
float gyr_x = 0, gyr_y = 0, gyr_z = 0;
float mag_x = 0, mag_y = 0, mag_z = 0;

void setup() {
  Serial.begin(115200);
  NoU3.beginMotors();
  NoU3.beginIMUs();
}

void loop() {
  NoU3.updateIMUs();

  acl_x = NoU3.acceleration_x;
  acl_y = NoU3.acceleration_y;
  acl_z = NoU3.acceleration_z;
  gyr_x = NoU3.gyroscope_x;
  gyr_y = NoU3.gyroscope_y;
  gyr_z = NoU3.gyroscope_z;
  mag_x = NoU3.magnetometer_x;
  mag_y = NoU3.magnetometer_y;
  mag_z = NoU3.magnetometer_z;

  //gyr_x -= cal.gyro_zerorate[0];
  //gyr_y -= cal.gyro_zerorate[1];
  //gyr_z -= cal.gyro_zerorate[2];

  if(NoU3.checkDataIMU() == true){
    printRaw();
  }
}

void printRaw(){
  // 'Raw' values to match expectation of MotionCal
  Serial.print("Raw:");
  Serial.print(int(acl_x*8192.0)); Serial.print(",");
  Serial.print(int(acl_y*8192.0)); Serial.print(",");
  Serial.print(int(acl_z*8192.0)); Serial.print(",");
  Serial.print(int(gyr_x*16.0)); Serial.print(",");
  Serial.print(int(gyr_y*16.0)); Serial.print(",");
  Serial.print(int(gyr_z*16.0)); Serial.print(",");
  Serial.print(int(mag_x*10.0)); Serial.print(",");
  Serial.print(int(mag_y*10.0)); Serial.print(",");
  Serial.print(int(mag_z*10.0)); Serial.println("");
}