#include "Arduino_BMI270_BMM150.h"

// Calibration parameters given by Magneto 1.2 for raw magnetometer output in nT
float A_inv_acc[3][3] = {
  {0.923189, -0.010470, 0.007076},
  {-0.010470, 0.900017, 0.004535},
  {0.007076, 0.004535, 0.927810}
};

float b_acc[3] = {5.528364, 0.666516, 0.187235};

const float g_to_ms2 = 9.80665;

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Started");

  if (!IMU.begin()) {
    Serial.println("Failed to initialize imu!");
    while (1);
  }
}

void loop()
{
    float x_mag, y_mag, z_mag;
    float Xm_off, Ym_off, Zm_off;
    float Xm_cal, Ym_cal, Zm_cal;

    float acc_x, acc_y, acc_z;
    float gyr_x, gyr_y, gyr_z;
    
    if (IMU.magneticFieldAvailable()) {
        IMU.readMagneticField(x_mag, y_mag, z_mag); // return the magnetic field in uT (10⁻6 Tesla)
    }

    if (IMU.accelerationAvailable()){
      IMU.readAcceleration(acc_x,acc_y,acc_z); //reads accel in Gs;
    }

    if(IMU.gyroscopeAvailable()){
      IMU.readGyroscope(gyr_x,gyr_y,gyr_z); //reads gyroscope accels in dps (degrees per second)
    }

    float gyr_off_x = 0.1191326105;
    float gyr_off_y = 0.0049374478;
    float gyr_off_z = -0.3535112593;

    gyr_x = gyr_x - gyr_off_x;
    gyr_y = gyr_y - gyr_off_y;
    gyr_z = gyr_z - gyr_off_z;

    Xm_off = x_mag - b_acc[0];
    Ym_off = y_mag - b_acc[1];
    Zm_off = z_mag - b_acc[2];
    Xm_cal = A_inv_acc[0][0]*Xm_off + A_inv_acc[0][1]*Ym_off + A_inv_acc[0][2]*Zm_off;
    Ym_cal = A_inv_acc[1][0]*Xm_off + A_inv_acc[1][1]*Ym_off + A_inv_acc[1][2]*Zm_off;
    Zm_cal = A_inv_acc[2][0]*Xm_off + A_inv_acc[2][1]*Ym_off + A_inv_acc[2][2]*Zm_off;


    Serial.println(String(acc_x*g_to_ms2) + " " + String(acc_y*g_to_ms2) + " " + String(acc_z*g_to_ms2) + " " + String(gyr_x) + " " + String(gyr_y) + " " + String(gyr_z) + " " + String(Xm_cal) + " " + String(Ym_cal) + " " + String(Zm_cal) + "\n");
    
}
