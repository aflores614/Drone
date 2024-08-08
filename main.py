from pymavlink import mavutil
import time
import sys
from connect_to_vehicle import connect_to_vehicle
from arm_drone import arm_drone
from arm_drone import is_armed
from get_location import get_location
from takeoff import takeoff
from disarm_drone import disarm_drone
from set_mode import set_mode 
from return_home import return_home
from land import land
from fly_forward import fly_forward
from check_pre_arm import check_pre_arm
from distance_sensor import avg_distance
from set_movment import fly_movment


master = connect_to_vehicle()

if master:
    # Perform pre-arm check
    if check_pre_arm(master):
        # get take off  coordinate 
        Home_lat, Home_lon, Home_alt = get_location(master) 
        print("Home postion is set:")
        print(Home_lat, Home_lon, Home_alt) 
        # Arm drone 
        arm_drone(master)
        if is_armed(master):
            print("Drone is armed!")
        else:
            print("Drone is not armed.")      
            sys.exit()
        time.sleep(5)
        # take to a fix altitude  and hold for a fix time
        takeoff(master,1,5) 
        #scan for any obstacle before flying forward
        try:
            while True:
                dist = avg_distance()
                print ("Measured Distance = %.1f m" % dist)
                if( dist < 1):
                    print("Object too close")                        
                elif( dist > 1 or dist == 4.5):
                    print("Safe")
                    break
                else: # not safe to continue
                     print(" not safe to fly abort mission")
                     land(master)
                     disarm_drone(master)
                     sys.exit()
                
        except KeyboardInterrupt: # Reset by pressing CTRL + C
                print("Measurement stopped by User")

        target_distance = 2 # distance in meters
        current_distance = 0 # The distance the drone has traveled so far
        velocity_x = 0.5 # forward speed at 0.5 m/s
        neg_velocity_x = -velocity_x # backward speed at 0.5 m/s
        check_interval = 0.1 # The time interval between each check of the distance
        count = 0 # Counter to track how long the obstacle has been detected. 

       
        try:
            while current_distance < target_distance:
                dist = avg_distance()
                if( dist > 1 or dist == 4.5): 
                    fly_movment(master, velocity_x, 0, 0) 
                    time.sleep(check_interval)
                    current_distance += velocity_x * check_interval
                    print("Distance travel: ", current_distance)
                    count = 0
                
                elif( dist < 1):
                    fly_movment( master, neg_velocity_x , 0 ,0)                        
                    print("Obstacle detected")                     
                    time.sleep(check_interval)
                    current_distance += neg_velocity_x * check_interval
                    print("Distance travel: ", current_distance)             
                    count += 1 
                    if (count == 50 ): # obstacle doesn't move for 5 secounds
                        break
                else:
                    print("Invalid Distance sensor read abort mission")
                    land(master)
                    disarm_drone(master)
                    sys.exit()
                    
        except KeyboardInterrupt:
                    print("Measurement stopped by User")
        
        #return_home(master)

        land(master)     
        disarm_drone(master)
        print("Mission Complete")

    else:
        print("Pre-arm check failed. Cannot arm or take off.")
else:
        print("Can't connect to vehicle")
