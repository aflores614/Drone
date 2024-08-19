import RPi.GPIO as GPIO
import time

# Set GPIO numbering mode
GPIO.setmode(GPIO.BCM)

# Define GPIO pin for the servo signal
servo_pin = 26  # Change to the pin you're using

# Set up the GPIO pin for output
GPIO.setup(servo_pin, GPIO.OUT)

# Set up PWM on the servo pin, with a 50Hz frequency
pwm = GPIO.PWM(servo_pin, 50)  # 50Hz is the typical frequency for servos
pwm.start(0)  # Start PWM with 0% duty cycle (off)

def set_servo_angle(angle):
    duty = 2 + (angle / 18)  # Convert angle to duty cycle
    GPIO.output(servo_pin, True)
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.5)
    GPIO.output(servo_pin, False)
    pwm.ChangeDutyCycle(0)

try:
    while True:
        angle = float(input("Enter angle between 0 and 180: "))
        set_servo_angle(angle)

except KeyboardInterrupt:
    pass

pwm.stop()
GPIO.cleanup()
