import RPi.GPIO as GPIO
import time
import threading
from lidar_distance import get_distance

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
  try:
    duty = angle / 18 + 3  # Convert angle to duty cycle
    GPIO.output(servo_pin, True)
    pwm.ChangeDutyCycle(duty)
    time.sleep(1)
    GPIO.output(servo_pin, False)
    pwm.ChangeDutyCycle(duty)
  except Exception as e:
        print(f"Error in set_servo_angle: {e}")


def lidar_motor(): 
    angle = 180  # Move servo to 180 degrees
    set_servo_angle(angle)
    time.sleep(0.25)  # Small delay 
 
    while True:
            
         
            
            angle = 110  # Move servo to 180 degrees
            set_servo_angle(angle)
            time.sleep(0.25)  # Small delay
            
            angle = 90  # Move servo to 0 degrees
            set_servo_angle(angle)
            time.sleep(0.25)  # Small delay

            angle = 70  # Move servo to 0 degrees
            set_servo_angle(angle)
            time.sleep(0.25)  # Small delay
            
            angle = 90  # Move servo to 0 degrees
            set_servo_angle(angle)
            time.sleep(0.25)  # Small delay
            
            
servo_thread = threading.Thread(target=lidar_motor)
servo_thread.start()
#while True:
 #   dist_front = get_distance()
  #  print(dist_front)
