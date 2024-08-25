# import serial

# def parse_data(data):
#     parts = data.strip().split('\t')
#     if len(parts) != 3:
#         return None

#     try:
#         acc_data = parts[0].split(': ')[1].split(', ')
#         gyro_data = parts[1].split(': ')[1].split(', ')
#         mag_data = parts[2].split(': ')[1].split(', ')

#         acc = list(map(float, acc_data))
#         gyro = list(map(float, gyro_data))
#         mag = list(map(float, mag_data))

#         return {'acc': acc, 'gyro': gyro, 'mag': mag}
#     except (IndexError, ValueError):
#         return None

# def main():
#     ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
#     ser.flush()

#     while True:
#         if ser.in_waiting > 0:
#             line = ser.readline().decode('utf-8')
#             data = parse_data(line)
#             if data:
#                 print("\n")
#                 print(f"Acceleration: {data['acc']}")
#                 print(f"Gyroscope: {data['gyro']}")
#                 print(f"Magnetometer: {data['mag']}")

# if __name__ == "__main__":
#     main()


import serial
import time

ser = serial.Serial('/dev/ttyUSB0', baudrate=9600, timeout=1)

last_time = None
intervals = []

try:
    while True:
        data = ser.readline() 
        if data:
            current_time = time.time()
            if last_time is not None:
                interval = current_time - last_time
                intervals.append(interval)
                frequency = 1 / interval
                print(f"Frequency: {frequency:.2f} Hz")
            last_time = current_time

except KeyboardInterrupt:
    ser.close()
    print("Serial port closed.")
    # Optionally, print statistics
    if intervals:
        avg_interval = sum(intervals) / len(intervals)
        avg_frequency = 1 / avg_interval
        print(f"Average Frequency: {avg_frequency:.2f} Hz")
