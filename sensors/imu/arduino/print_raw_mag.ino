#include "Arduino_BMI270_BMM150.h"



void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
}

void loop()
{
    float x_mag, y_mag, z_mag;

    if (IMU.magneticFieldAvailable()) {
        IMU.readMagneticField(x_mag, y_mag, z_mag); // in uT
    }

    Serial.print(x_mag); Serial.print(" "); Serial.print(y_mag); Serial.print(" "); Serial.println(z_mag);
    delay(2000);
}