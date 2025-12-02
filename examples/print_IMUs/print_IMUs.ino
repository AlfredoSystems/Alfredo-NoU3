#include <Alfredo_NoU3.h>

void setup() {
  Serial.begin(115200);
  delay(2000);
  NoU3.begin();

  Serial.println("Accel_X\tAccel_Y\tAccel_Z\tGyro_X\tGyro_Y\tGyro_Z\tMag_X\tMag_Y\tMag_Z\tRoll\tPitch\tYaw");
}

long lastPrintMs = 0;  // Stores the last time the function was called

void loop() {
  if (millis() - lastPrintMs >= 200) {
    lastPrintMs = millis();  // Update the last time the function was called
    formatPrint(NoU3.acceleration_x); // Gs
    formatPrint(NoU3.acceleration_y);
    formatPrint(NoU3.acceleration_z);
    formatPrint(NoU3.gyroscope_x); // rad/s
    formatPrint(NoU3.gyroscope_y);
    formatPrint(NoU3.gyroscope_z);
    formatPrint(NoU3.magnetometer_x); // uT
    formatPrint(NoU3.magnetometer_y);
    formatPrint(NoU3.magnetometer_z);
    formatPrint(NoU3.roll);
    formatPrint(NoU3.pitch);
    formatPrint(NoU3.yaw);
    Serial.println('\t');
  }
}

void formatPrint(float num) {
  char buffer[11];
  snprintf(buffer, sizeof(buffer), "%7.3f   ", num);
  Serial.print(buffer);
}
