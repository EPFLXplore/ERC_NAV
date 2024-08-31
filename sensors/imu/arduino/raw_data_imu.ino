#include <Arduino_BMI270_BMM150.h>


void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.println("IMU initialized!");
}

void loop() {
  float x_acc, y_acc, z_acc;
  float x_gyro, y_gyro, z_gyro;
  float x_mag, y_mag, z_mag;

  // Check if new data is available
  if (IMU.accelerationAvailable()) {
    IMU.readAcceleration(x_acc, y_acc, z_acc);
  }

  if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x_gyro, y_gyro, z_gyro);
  }

  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(x_mag, y_mag, z_mag);
  }

  // Print the data to the Serial Monitor
  Serial.print("Acceleration (Gs): ");
  Serial.print(x_acc);
  Serial.print(", ");
  Serial.print(y_acc);
  Serial.print(", ");
  Serial.print(z_acc);
  Serial.print("\t");

// Need to convert gyroscope to rad/s
  Serial.print("Gyroscope (degrees/s): ");
  Serial.print(x_gyro);
  Serial.print(", ");
  Serial.print(y_gyro);
  Serial.print(", ");
  Serial.print(z_gyro);
  Serial.print("\t");

  Serial.print("Magnetometer (uT): ");
  Serial.print(x_mag);
  Serial.print(", ");
  Serial.print(y_mag);
  Serial.print(", ");
  Serial.print(z_mag);
  Serial.println();

  delay(100); // Adjust the delay as needed
}



