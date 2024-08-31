#include "Arduino_BMI270_BMM150.h"

const float factor = 980665;
// Calibration parameters
float A_inv_acc[3][3] = {
  {0.002332, -0.000044, -0.000004},
  {-0.000044, 0.001662, 0.000325},
  {-0.000004, 0.000325, 0.001649}
};

float b_acc[3] = {104881.247741, 297051.278986, 365117.069758};

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
    float Xa_off, Ya_off, Za_off;
    float Xa_cal, Ya_cal, Za_cal;

    // Check if new data is available
    if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(x_acc, y_acc, z_acc); // read in Gs
    }

    Xa_off = x_acc*factor - b_acc[0];
    Ya_off = y_acc*factor - b_acc[1];
    Za_off = z_acc*factor - b_acc[2];
    Xa_cal = A_inv_acc[0][0]*Xa_off + A_inv_acc[0][1]*Ya_off + A_inv_acc[0][2]*Za_off;
    Ya_cal = A_inv_acc[1][0]*Xa_off + A_inv_acc[1][1]*Ya_off + A_inv_acc[1][2]*Za_off;
    Za_cal = A_inv_acc[2][0]*Xa_off + A_inv_acc[2][1]*Ya_off + A_inv_acc[2][2]*Za_off;

    // Print in mGal

    Serial.print(Xa_cal); Serial.print(" "); Serial.print(Ya_cal); Serial.print(" "); Serial.println(Za_cal);
    delay(125);
}