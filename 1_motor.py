import RPi.GPIO as GPIO



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



# Motor_1 config
GPIO.setup(PIN_STEP_1, GPIO.OUT)
GPIO.setup(PIN_DIR_STEP_1, GPIO.OUT)

GPIO.setup(PIN_EN_STEP_1, GPIO.OUT)
GPIO.setup(PIN_MS1_STEP_1, GPIO.OUT)
GPIO.setup(PIN_MS2_STEP_1, GPIO.OUT)


GPIO.output(PIN_EN_STEP_1, 0) 
GPIO.output(PIN_DIR_STEP_1, 1)
GPIO.output(PIN_MS1_STEP_1, 1)
GPIO.output(PIN_MS2_STEP_1, 1)


# Motor_2 config
GPIO.setup(PIN_STEP_2, GPIO.OUT)
GPIO.setup(PIN_DIR_STEP_2, GPIO.OUT)

GPIO.setup(PIN_EN_STEP_2, GPIO.OUT)
GPIO.setup(PIN_MS1_STEP_2, GPIO.OUT)
GPIO.setup(PIN_MS2_STEP_2, GPIO.OUT)


GPIO.output(PIN_EN_STEP_2, 0) 
GPIO.output(PIN_DIR_STEP_2, 0)
GPIO.output(PIN_MS1_STEP_2, 1)
GPIO.output(PIN_MS2_STEP_2, 1)


PWM1 = GPIO.PWM(PIN_STEP_1, 10) 	#starten 1Hz
PWM1.start(50) 					#DC 50%
PWM2 = GPIO.PWM(PIN_STEP_2, 10) 	#starten 1Hz
PWM2.start(50) 					#DC 50%

def set_motor_speed(motor_pwm, speed):
    motor_pwm.ChangeFrequency(speed)

def set_motor_direction(motor_dir, direction):
    GPIO.output(motor_dir, direction)

def stop_motors():
    GPIO.cleanup()

try:
    while True:
        speed_1 = float(input("Enter speed for Motor 1 (0-100): "))
        speed_2 = float(input("Enter speed for Motor 2 (0-100): "))

        set_motor_speed(PWM1, speed_1)
        set_motor_speed(PWM2, speed_2)

except KeyboardInterrupt:
    stop_motors()