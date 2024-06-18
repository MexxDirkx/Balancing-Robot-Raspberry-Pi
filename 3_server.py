import threading
import RPi.GPIO as GPIO
from time import sleep, time, strftime, gmtime

from mpu6050 import mpu6050
import smbus

# Servers
from flask import Flask, render_template, request, jsonify
import pymysql
import atexit


from simple_pid import PID



# Server configuration
 # Flask
app = Flask(__name__, static_url_path='', static_folder='www', template_folder='www')
 # Connect pymysql
conn = pymysql.connect(host="localhost", unix_socket="/var/run/mysqld/mysqld.sock", user="Mexx", passwd="Dirkx1", db="Balancing_robot")
cur = conn.cursor()

# PID
Kp = -40
Ki = -200
Kd = 0
tunings = (Kp, Ki, Kd)

pid = PID(setpoint=0)
pid.tunings = tunings
pid.output_limits = (0, 100)

pid_output = 0.00001

# Sensor mpu6050 setup
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

# Initialize global variable for angle accumulation
cumulative_angle_y = 0
angle_change_y = 0


# Motor pin definitions
GPIO.setmode(GPIO.BCM)

PIN_EN_STEP_1 = 22
PIN_MS1_STEP_1 = 16
PIN_MS2_STEP_1 = 26
PIN_DIR_STEP_1 = 5
PIN_STEP_1 = 6

PIN_EN_STEP_2 = 27
PIN_MS1_STEP_2 = 20
PIN_MS2_STEP_2 = 21
PIN_DIR_STEP_2 = 13
PIN_STEP_2 = 19

target_value = 0
motor_dir = 1
running = True

# Motors setup
def setup_motor(pins, reverse_dir = False):
    GPIO.setup(pins['STEP'], GPIO.OUT)
    GPIO.setup(pins['DIR'], GPIO.OUT)
    GPIO.setup(pins['EN'], GPIO.OUT)
    GPIO.setup(pins['MS1'], GPIO.OUT)
    GPIO.setup(pins['MS2'], GPIO.OUT)
    GPIO.output(pins['EN'], 0) 
    GPIO.output(pins['DIR'], reverse_dir)
    GPIO.output(pins['MS1'], 1)
    GPIO.output(pins['MS2'], 1)
    pwm = GPIO.PWM(pins['STEP'], 1)  # starting with 1000 Hz
    pwm.start(50)
    return pwm
motors = {
    1: {'PWM': setup_motor({'STEP': PIN_STEP_1, 'DIR': PIN_DIR_STEP_1, 'EN': PIN_EN_STEP_1, 'MS1': PIN_MS1_STEP_1, 'MS2': PIN_MS2_STEP_1}),
        'DIR': PIN_DIR_STEP_1},
    2: {'PWM': setup_motor({'STEP': PIN_STEP_2, 'DIR': PIN_DIR_STEP_2, 'EN': PIN_EN_STEP_2, 'MS1': PIN_MS1_STEP_2, 'MS2': PIN_MS2_STEP_2}, reverse_dir=True),
        'DIR': PIN_DIR_STEP_2}
}


MOTOR_DIR_PINS = [PIN_DIR_STEP_1, PIN_DIR_STEP_2]
MOTOR_PWMS = [motors[1]['PWM'], motors[2]['PWM']]

# Gyroscope functions
def calibrate_gyroscope_y(samples = 1000):
    print("Calibrating gyroscope...")
    sum_y_data = 0

    for _ in range(samples):
        gyro_data = sensor.get_gyro_data()
        sum_y_data += gyro_data['y']
        sleep(0.001)

    bias_y = sum_y_data / samples
    print("Calibration complete.")
    return bias_y

def read_gyroscope():
    global cumulative_angle_y, angle_change_y
    last_time = time()
    while True:
        current_time = time()
        dt = current_time - last_time

        gyro_data = sensor.get_gyro_data()

        angle_change_y = (gyro_data['y'] - gyro_bias * (1 + gyro_data['y'] - gyro_bias)) * dt
        #angle_change_y = (gyro_data['y'] - gyro_bias ) * dt
        #cumulative_angle_y += angle_change_y

        #print(f"Gyrosensor Rotation Y: {cumulative_angle_y:.2f}°")
        #print(f"PID output: {pid_output}")

        last_time = current_time
        sleep(0.001)



# Control motor functions
def set_motors_speed(motor_pins, speed):
    duty_cycle = speed * (75000/100)
    duty_cycle = 1 if duty_cycle == 0 else duty_cycle
    for motor_pin in motor_pins:
        motor_pin.ChangeFrequency(duty_cycle)

def set_motor_direction(motor_dir_pins, direction):
    for motor_dir_pin in motor_dir_pins:
        if motor_dir_pin == motors[2]['DIR']:
            direction = not direction
        GPIO.output(motor_dir_pin, direction)

def stop_motors():
    global running
    running = False
    print("STOP MOTORS")
    GPIO.cleanup()


# Server functions
def run_server_thread():
    atexit.register(stop_motors)
    app.run(debug=True, host='0.0.0.0', port=5050, use_reloader=False)

    #clear_database()
    #
    #while True:
    #    update_database()
    #    sleep(0.5)

@app.route('/')
def index():    
    return render_template('index.html')
@app.route('/updateW', methods=['POST'])
def update_w():
    global target_value
    # Get the value sent from the slider
    target_value = float(request.form['Wnew'])
    print(f"Received new W value: {target_value}")

    return jsonify(success=True, message=f"W value updated to {target_value}")
@app.route('/updateP', methods=['POST'])
def update_p():
    # Get the value sent from the slider
    p_new_value = request.form['Pnew']
    print(f"Received new P value: {p_new_value}")

    pid.Kp = -float(p_new_value)
    print(pid.tunings)

    return jsonify(success=True, message=f"P value updated to {p_new_value}")
@app.route('/updateI', methods=['POST'])
def update_i():
    # Get the value sent from the slider
    i_new_value = request.form['Inew']
    print(f"Received new I value: {i_new_value}")

    pid.Ki = -float(i_new_value)
    print(pid.tunings)

    return jsonify(success=True, message=f"I value updated to {i_new_value}")
@app.route('/updateD', methods=['POST'])
def update_d():
    # Get the value sent from the slider
    d_new_value = request.form['Dnew']
    print(f"Received new D value: {d_new_value}")

    pid.Kd = -float(d_new_value)
    print(pid.tunings)

    return jsonify(success=True, message=f"D value updated to {d_new_value}")


def get_time_string():
    return strftime("%Y-%m-%d %H:%M:%S", gmtime())
def clear_database():
    cur.execute("TRUNCATE TABLE data")

def update_database():
    global cumulative_angle_y, pid_output

    time = get_time_string()
    cur.execute("INSERT INTO data(time, angle, pid_output) VALUES(%s,%s,%s)", (time, cumulative_angle_y, pid_output))
    conn.commit()


#gyro_bias = calibrate_gyroscope_y()

gyro_bias = 1.2107251908396925
gyro_thread = threading.Thread(target=read_gyroscope)
gyro_thread.start()

server_thread = threading.Thread(target = run_server_thread)
server_thread.start()

def fix_direction():
    global motor_dir, cumulative_angle_y
    if angle_change_y < 0:
        if motor_dir == 0:
            set_motor_direction(MOTOR_DIR_PINS, 0)
            motor_dir = 1
            pid.reset()
    elif motor_dir == 1:
        set_motor_direction(MOTOR_DIR_PINS, 1)
        motor_dir = 0
        pid.reset()
    

try:
    while running:
        print("Angle:", angle_change_y)
        fix_direction()


        error = abs(angle_change_y)
        pid_output = pid(error)

        set_motors_speed(MOTOR_PWMS, pid_output)
        #print("MOTOR SPEED:", pid_output)
        
        sleep(0.02)



except KeyboardInterrupt:
    stop_motors()
    print(gyro_bias)
    print("Motors stopped")