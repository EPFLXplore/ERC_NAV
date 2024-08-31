// #include <Simpletimer.h>
// Calibration and Manual fusion

#include <Arduino_BMI270_BMM150.h>
#include <SensorFusion.h> 




const float factor_acc = 9.80665; // G to m/s^2
const float deg_to_rad = 0.01745329251; // pi/180


// Calibration parameters
float A_inv_acc[3][3] = {
  {1.002568, 0.000330, -0.001953},
  {0.000330, 1.004072, 0.003146},
  {-0.001953, 0.003146, 0.996397}
};

float b_acc[3] = {-0.031602/factor_acc, -0.022283/factor_acc, 0.027434/factor_acc};


float A_inv_mag[3][3] = {
  {1.246440, -0.006369, 0.002019},
  {-0.006369, 1.243367, 0.008961},
  {0.002019, 00.008961, 1.265930}
};
float b_mag[3] = {21.439862, -6.180739, -28.198154};


SF fusion;
float deltat;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  Serial.println("IMU initialized!");
}



void loop()
{

  float x_acc, y_acc, z_acc;
  float x_gyro, y_gyro, z_gyro;
  float x_mag, y_mag, z_mag;
  float roll, pitch, yaw;
  float Xa_off, Ya_off, Za_off, Xa_cal, Ya_cal, Za_cal;
  float Xm_off, Ym_off, Zm_off, Xm_cal, Ym_cal, Zm_cal;
  float x_gyro_print, y_gyro_print, z_gyro_print;
  float x_acc_print, y_acc_print, z_acc_print;
  float* quat;
 
 
// Check if new data is available
  // if (IMU.accelerationAvailable()) {
  IMU.readAcceleration(x_acc, y_acc, z_acc); // in Gs   
  // }

  // if (IMU.gyroscopeAvailable()) {
    IMU.readGyroscope(x_gyro, y_gyro, z_gyro); //degrees/second"
  // }

  // if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(x_mag, y_mag, z_mag); // in uT
  // }


  // Accelerometer calibration => A_inv * (data -b ) => result in Gs
  Xa_off = x_acc - b_acc[0];
  Ya_off = y_acc - b_acc[1];
  Za_off = z_acc - b_acc[2];
  Xa_cal = A_inv_acc[0][0]*Xa_off + A_inv_acc[0][1]*Ya_off + A_inv_acc[0][2]*Za_off;
  Ya_cal = A_inv_acc[1][0]*Xa_off + A_inv_acc[1][1]*Ya_off + A_inv_acc[1][2]*Za_off;
  Za_cal = A_inv_acc[2][0]*Xa_off + A_inv_acc[2][1]*Ya_off + A_inv_acc[2][2]*Za_off;

  // Magnetometer calibration => A_inv * (data -b ) => result in uT
  Xm_off = x_mag - b_mag[0];
  Ym_off = y_mag - b_mag[1];
  Zm_off = z_mag - b_mag[2];
  Xm_cal = A_inv_mag[0][0]*Xm_off + A_inv_mag[0][1]*Ym_off + A_inv_mag[0][2]*Zm_off;
  Ym_cal = A_inv_mag[1][0]*Xm_off + A_inv_mag[1][1]*Ym_off + A_inv_mag[1][2]*Zm_off;
  Zm_cal = A_inv_mag[2][0]*Xm_off + A_inv_mag[2][1]*Ym_off + A_inv_mag[2][2]*Zm_off;

  // Convert Gyroscope data to rad/sec
  x_gyro_print= x_gyro * deg_to_rad;
  y_gyro_print= y_gyro * deg_to_rad;
  z_gyro_print= z_gyro * deg_to_rad;


  //float swp = Ym_cal;
  //Ym_cal = -Zm_cal;
  //Zm_cal = swp;
  deltat = fusion.deltatUpdate(); //this have to be done before calling the fusion update
  
  // float swp = Xa_cal;
  // Xa_cal = Ya_cal;
  // Ya_cal = swp;
  // Za_cal = -Za_cal;

  float swp = Xa_cal;
  Xa_cal = -Ya_cal;
  Ya_cal = -swp;

  swp = x_gyro_print;
  x_gyro_print = -y_gyro_print;
  y_gyro_print = -swp;

  // swp = Zm_cal;
  // Zm_cal = Ym_cal;
  // Ym_cal = swp;
  fusion.MadgwickUpdate(x_gyro_print,y_gyro_print,z_gyro_print, Xa_cal, Ya_cal, Za_cal, Xm_cal, Ym_cal, Zm_cal, deltat); 

  // quat = fusion.getQuat();
  roll = fusion.getRoll();
  pitch = fusion.getPitch();
  yaw = fusion.getYaw();

  // Convert Accelerometer data from Gs to m/s^2
  x_acc_print= Xa_cal * factor_acc;
  y_acc_print= Ya_cal * factor_acc;
  z_acc_print= Za_cal * factor_acc;


  Serial.print(x_acc_print);
  Serial.print(", ");
  Serial.print(y_acc_print);
  Serial.print(", ");
  Serial.print(z_acc_print);
  Serial.print(", ");


  Serial.print(x_gyro_print);
  Serial.print(", ");
  Serial.print(y_gyro_print);
  Serial.print(", ");
  Serial.print(z_gyro_print);
  Serial.print(", ");


  // Serial.print(quat[0]);
  // Serial.print(", ");
  // Serial.print(quat[1]);
  // Serial.print(", ");
  // Serial.print(quat[2]);
  // Serial.print(", ");
  // Serial.print(quat[3]);
  // Serial.println();

  Serial.print(roll);
  Serial.print(", ");
  Serial.print(pitch);
  Serial.print(", ");
  Serial.print(yaw);
  Serial.println();

}