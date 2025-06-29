#include <Alfredo_NoU3.h>

void setup() {
  Serial.begin(115200);
  NoU3.begin();
}

long lastPrintMs = 0;  // Stores the last time the function was called

void loop() {

  NoU3.updateIMUs();

  if (millis() - lastPrintMs >= 20) {
    lastPrintMs = millis();  // Update the last time the function was called
    formatPrint(NoU3.acceleration_x);
    Serial.print(", ");
    formatPrint(NoU3.acceleration_y);
    Serial.print(", ");
    formatPrint(NoU3.acceleration_z);
    Serial.print(", ");
    formatPrint(NoU3.gyroscope_x);
    Serial.print(", ");
    formatPrint(NoU3.gyroscope_y);
    Serial.print(", ");
    formatPrint(NoU3.gyroscope_z);
    Serial.print(", ");
    formatPrint(NoU3.magnetometer_x);
    Serial.print(", ");
    formatPrint(NoU3.magnetometer_y);
    Serial.print(", ");
    formatPrint(NoU3.magnetometer_z);
    Serial.println("");
  }
}

void formatPrint(float num) {
  char buffer[11];
  snprintf(buffer, sizeof(buffer), "%.3f ", num);
  Serial.print(buffer);
}
