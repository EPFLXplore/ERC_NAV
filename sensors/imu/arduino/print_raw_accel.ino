#include "Arduino_BMI270_BMM150.h"

const float factor = 9.80665;

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
    float x_acc, y_acc, z_acc;
    float Xa_print, Ya_print, Za_print;

    // Check if new data is available
    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(x_acc, y_acc, z_acc); // in Gs
    }

    // print in m/s^2
    Xa_print = x_acc* factor; 
    Ya_print = y_acc* factor;
    Za_print = z_acc* factor;

    Serial.print(Xa_print); Serial.print(" "); Serial.print(Ya_print); Serial.print(" "); Serial.println(Za_print);
    delay(2000);
}