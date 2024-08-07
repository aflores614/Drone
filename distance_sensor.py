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
    
    if distance > 4.5 or distance < 0.02:
        return 4.5
 
    return distance 
 
def avg_distance(num_samples = 5):
    distances = []
    for _ in range(num_samples):
        dist = distance()
        if dist != -1:
            distances.append(dist)
        time.sleep(0.1)
    if len(distances) == 0:
         return -1
    return sum(distances)/ len(distances)
 
if __name__ == "__main__":
    while True:
        dist = avg_distance()       
        if( dist == -1):
            print("Invaid Reading")
        elif ( dist == 4.5):
            print("Obstacle is outof range")         
        else:    
            print("Measured Distance = %.2f m" % dist)
           
                    
