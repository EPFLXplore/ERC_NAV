import sys
import math
import string


#read a file with values from the IMU
def main():
    acc_x = []
    acc_y = []
    acc_z = []
    gyro_x = []
    gyro_y = []
    gyro_z = []

    with open("values_from_imu.txt", "r") as file:
        lines = file.readlines()
        for line in lines:
            
            line = line.replace('\n', '')
            #print(line)
            if not line:
                continue
            #ax, ay, az, gx, gy, gz, mx, my, mz
            values = line.split(' ')

            acc_x.append(float(values[0]))
            acc_y.append(float(values[1]))
            acc_z.append(float(values[2]))
            gyro_x.append(float(values[3]))
            gyro_y.append(float(values[4]))
            gyro_z.append(float(values[5]))
        mean_acc_x = sum(acc_x) / len(acc_x)
        mean_acc_y = sum(acc_y) / len(acc_y)
        mean_acc_z = sum(acc_z) / len(acc_z)
        mean_gyro_x = sum(gyro_x) / len(gyro_x)
        mean_gyro_y = sum(gyro_y) / len(gyro_y)
        mean_gyro_z = sum(gyro_z) / len(gyro_z)

        print(f"biases: {mean_acc_x} {mean_acc_y} {mean_acc_z} {mean_gyro_x} {mean_gyro_y} {mean_gyro_z}")

        std_acc_x = calc_std(acc_x, mean_acc_x)
        std_acc_y = calc_std(acc_y, mean_acc_y)
        std_acc_z = calc_std(acc_z, mean_acc_z)
        std_gyro_x = calc_std(gyro_x, mean_gyro_x)
        std_gyro_y = calc_std(gyro_y, mean_gyro_y)
        std_gyro_z = calc_std(gyro_z, mean_gyro_z)

        mean_std_acc = (std_acc_x + std_acc_y + std_acc_z) / 3
        mean_std_gyro = (std_gyro_x + std_gyro_y + std_gyro_z) / 3

        print("Mean acceleration noise: ", mean_std_acc)
        print("Mean gyro noise: ", mean_std_gyro)

def calc_std(val, mean):
    return sum([(x - mean)**2 for x in val]) / len(val)

if __name__ == "__main__":
    main()