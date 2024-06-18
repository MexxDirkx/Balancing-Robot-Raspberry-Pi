from mpu6050 import mpu6050
from time import sleep, time
import smbus

sensor = mpu6050(0x68)


power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c
bus = smbus.SMBus(1) 
DeviceAddress = 0x68
CONFIG       = 0x1A
bus.write_byte_data(DeviceAddress, power_mgmt_1, 0)
#Write to Configuration register
#Setting DLPF (last three bit of 0X1A to 6 i.e '110' It removes the noise due to vibration.) https://ulrichbuschbaum.wordpress.com/2015/01/18/using-the-mpu6050s-dlpf/
#bus.write_byte_data(DeviceAddress, CONFIG, int('0000110',2))	
bus.write_byte_data(DeviceAddress, CONFIG, int('0000110',2))



def calibrate_gyroscope_y(samples=200):
    print("Calibrating gyroscope...")
    sum_y_data = 0
    for _ in range(samples):
        gyro_data = sensor.get_gyro_data()
        sum_y_data += gyro_data['y']
        sleep(0.001)
    bias_y = sum_y_data / samples
    print("Calibration complete.")
    return bias_y


# Calibrate the sensor
gyro_bias = calibrate_gyroscope_y()

cumulative_angle_y = 0

last_time = time()

try:
    while True:
        current_time = time()
        dt = current_time - last_time  # Time delta in seconds

        gyro_data = sensor.get_gyro_data()

        # Subtract bias and integrate angular velocity to get the angle change
        angle_change_y = (gyro_data['y'] - gyro_bias * (1 + gyro_data['y'] - gyro_bias)) * dt

        # Update cumulative angles
        cumulative_angle_y += angle_change_y

        print(f"Gyrosensor Rotation Y: {cumulative_angle_y:.2f}")

        last_time = current_time
        sleep(0.001)

except KeyboardInterrupt:
    print("Interrupted by user")
