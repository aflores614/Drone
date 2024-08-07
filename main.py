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
        Home_lat, Home_lon, Home_alt = get_location(master)
        print("Home postion is set")
        print(Home_lat, Home_lon, Home_alt)
        arm_drone(master)
        if is_armed(master):
            print("Drone is armed!")
        else:
            print("Drone is not armed.")      
            sys.exit()
        time.sleep(5)
        takeoff(master,1,5) 
        if True:
            try:
                while True:
                    dist = avg_distance()
                    print ("Measured Distance = %.1f m" % dist)
                    if( dist < 1):
                        print("Object too close")                        
                    elif( dist > 1 or dist == 4.5):
                        print("Safe")
                        break
                
            except KeyboardInterrupt: # Reset by pressing CTRL + C
                print("Measurement stopped by User")

        target_distance = 2
        current_distance = 0
        velocity_x = 0.5 #forward at 0.5 m/s
        neg_veloxity_x = -velocity_x #reverse at 0.5 m/s
        check_interval = 0.1
        count = 0

        if True:
            try:
                while current_distance < target_distance:
                    dist = distance()
                    if( dist > 1 or dist == 4.5): 
                        fly_movment(master, velocity_x, 0, 0) 
                    elif( dist == -1):
                        break
                    else: 
                        fly_movment( master, neg_veloxity_x , 0 ,0)                        
                        print("Obstacle detected")              
                        count += 1 
                        if (count == 50 ): # obstacle doesn't move for 5 secounds
                            break
                    time.sleep(check_interval)
                    current_distance += velocity_x * check_interval
                        
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
