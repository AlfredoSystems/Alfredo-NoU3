#include <Alfredo_NoU3.h>

const int numMeasurements = 2000;

float acceleration_x[numMeasurements];
float acceleration_y[numMeasurements];
float acceleration_z[numMeasurements];
float gyroscope_x[numMeasurements];
float gyroscope_y[numMeasurements];
float gyroscope_z[numMeasurements];

int currentIndex = 0;
bool measurementsComplete = false;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait here until the Serial Monitor is opened
  }

  NoU3.begin();

  Serial.println("Serial connection established. Starting data collection...");
}

void loop() {
  
  // Check if data is available and measurements are not complete
  if (!measurementsComplete && NoU3.updateIMUs()) {

    // Store measurements in arrays
    acceleration_x[currentIndex] = NoU3.acceleration_x;
    acceleration_y[currentIndex] = NoU3.acceleration_y;
    acceleration_z[currentIndex] = NoU3.acceleration_z;
    gyroscope_x[currentIndex] = NoU3.gyroscope_x;
    gyroscope_y[currentIndex] = NoU3.gyroscope_y;
    gyroscope_z[currentIndex] = NoU3.gyroscope_z;

    currentIndex++;

    // Check if we've reached the target number of measurements
    if (currentIndex >= numMeasurements) {
      Serial.print(currentIndex);
      Serial.println(" measurements taken");
      measurementsComplete = true;
      calculateAndPrintAverages();
    }
  }
}

void calculateAndPrintAverages() {
  float sumAx = 0, sumAy = 0, sumAz = 0;
  float sumGx = 0, sumGy = 0, sumGz = 0;

  // Sum all values for each variable
  for (int i = 0; i < numMeasurements; i++) {
    sumAx += acceleration_x[i];
    sumAy += acceleration_y[i];
    sumAz += acceleration_z[i];
    sumGx += gyroscope_x[i];
    sumGy += gyroscope_y[i];
    sumGz += gyroscope_z[i];
  }

  // Calculate averages
  float avgAx = sumAx / numMeasurements;
  float avgAy = sumAy / numMeasurements;
  float avgAz = sumAz / numMeasurements;
  float avgGx = sumGx / numMeasurements;
  float avgGy = sumGy / numMeasurements;
  float avgGz = sumGz / numMeasurements;

  // Print averages
  Serial.print("Average values after "); Serial.print(currentIndex); Serial.println(" measurements:");
  Serial.print("Acceleration X (Gs): "); Serial.println(avgAx, 3);
  Serial.print("Acceleration Y (Gs): "); Serial.println(avgAy, 3);
  Serial.print("Acceleration Z (Gs): "); Serial.println(avgAz, 3);
  Serial.print("Gyroscope X (deg/s): "); Serial.println(avgGx, 3);
  Serial.print("Gyroscope Y (deg/s): "); Serial.println(avgGy, 3);
  Serial.print("Gyroscope Z (deg/s): "); Serial.println(avgGz, 3);
}