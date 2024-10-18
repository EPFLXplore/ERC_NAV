#include "Arduino_BMI270_BMM150.h"

// Calibration parameters given by Magneto 1.2 for raw magnetometer output in nT
float A_inv_acc[3][3] = {
  {0.923189, -0.010470, 0.007076},
  {-0.010470, 0.900017, 0.004535},
  {0.007076, 0.004535, 0.927810}
};

float b_acc[3] = {5.528364, 0.666516, 0.187235};

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize magnetometer!");
    while (1);
  }
}

void loop()
{
    float x_mag, y_mag, z_mag;
    float Xm_off, Ym_off, Zm_off;
    float Xm_cal, Ym_cal, Zm_cal;

    
    if (IMU.magneticFieldAvailable()) { // Query if new magnetic field data from the IMU is available
        IMU.readMagneticField(x_mag, y_mag, z_mag); // return the magnetic field in uT
    }

    Xm_off = x_mag - b_acc[0];
    Ym_off = y_mag - b_acc[1];
    Zm_off = z_mag - b_acc[2];
    Xm_cal = A_inv_acc[0][0]*Xm_off + A_inv_acc[0][1]*Ym_off + A_inv_acc[0][2]*Zm_off;
    Ym_cal = A_inv_acc[1][0]*Xm_off + A_inv_acc[1][1]*Ym_off + A_inv_acc[1][2]*Zm_off;
    Zm_cal = A_inv_acc[2][0]*Xm_off + A_inv_acc[2][1]*Ym_off + A_inv_acc[2][2]*Zm_off;


    Serial.print(Xm_cal); Serial.print(" "); Serial.print(Ym_cal); Serial.print(" "); Serial.println(Zm_cal);
    delay(12);
}
