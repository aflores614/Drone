import RPi.GPIO as GPIO
import time
 
#set GPIO Pins
GPIO_TRIGGER = 23
GPIO_ECHO = 24
#GPIO Mode (BOARD / BCM) 
GPIO.setmode(GPIO.BCM)
#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
 
def distance():

   
    GPIO.output(GPIO_TRIGGER, True)
   
    # set Trigger after 0.01ms to LOW    
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
 
    StartTime = time.time()
    StopTime = time.time()
 
    # save StartTime
    while GPIO.input(GPIO_ECHO) == 0:
        StartTime = time.time()
        
    # save time of arrival
    while GPIO.input(GPIO_ECHO) == 1:
        StopTime = time.time()
        
        
    # time difference between start and arrival
    TimeElapsed = StopTime - StartTime
    
    # multiply with the sonic speed (34300 cm/s)
    # and divide by 2 then 100 to get meter
    distance = ((TimeElapsed * 34300) / 2) / 100
 
    return distance 
 
