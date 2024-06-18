from mpu6050 import mpu6050
from time import sleep, time


sensor = mpu6050(0x68)

def calibrate_gyroscope_y(samples = 10000):
    print("Calibrating gyroscope...")
    sum_y_data = 0

    for _ in range(samples):
        gyro_data = sensor.get_gyro_data()
        sum_y_data += gyro_data['y']
        sleep(0.001)

    bias_y = sum_y_data / samples
    print("Calibration complete.")
    return bias_y

gyro_bias = calibrate_gyroscope_y()
print(gyro_bias)