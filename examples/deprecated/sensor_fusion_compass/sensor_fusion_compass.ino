#include <Alfredo_NoU3.h>

float yaw_gyro_deg = 0;
float yaw_mag_deg = 0;
float wrapped_yaw_mag_deg = 0;
float yaw = 0;

const float alpha = 0.98; // Complementary filter weighting
unsigned long lastTime = 0;

float gyro_z_offset_degrees = 0.444;

void setup() {
  Serial.begin(115200);
  NoU3.begin();

}

void loop() {

  if (NoU3.updateIMUs()) {

    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;

    // Gyroscope yaw update
    yaw_gyro_deg += (NoU3.gyroscope_z - gyro_z_offset_degrees) * dt;

    // Magnetometer yaw
    yaw_mag_deg = -1 * degrees(atan2(NoU3.magnetometer_y, NoU3.magnetometer_x));

    // Wrap Magnetometer yaw
    wrapped_yaw_mag_deg = wrapYaw(yaw_mag_deg);

    // Apply complementary filter with drift compensation
    yaw = alpha * (yaw_gyro_deg) + (1 - alpha) * wrapped_yaw_mag_deg;

    // Print results
    Serial.print("yaw_gyro_deg: ");
    Serial.print(yaw_gyro_deg);
    Serial.print(" wrapped_yaw_mag_deg: ");
    Serial.print(wrapped_yaw_mag_deg);
    Serial.print(" Yaw: ");
    Serial.print(yaw);
    
    Serial.println(" ");

  }
}

// Function to process yaw and keep track of rotations
float wrapYaw(float currentYaw) {
  static bool isInitialized = false;
  static int rotations = 0;
  static float previousYaw = 0;

  if(isInitialized == false){
    previousYaw = currentYaw;
    isInitialized = true;
  }

    // Check for wrapping
    if (currentYaw - previousYaw > 180.0) {
        rotations--; // Wrapped from -180 to 180
    } else if (currentYaw - previousYaw < -180.0) {
        rotations++; // Wrapped from 180 to -180
    }

    // Wrap the yaw angle between -180 and 180
    float wrappedYaw = currentYaw + rotations * 360.0;

    previousYaw = currentYaw;

    return wrappedYaw;
}