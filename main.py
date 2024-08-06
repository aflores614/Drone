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
from distance_sensor import distance


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
        takeoff(master,1.5,5) 
        if True:
            try:
                while True:
                    dist = distance()
                    print ("Measured Distance = %.1f m" % dist)
                    if( dist > 3):
                        print("Safe distance")
                    else:    
                        print("Object too close")
                        break
            except KeyboardInterrupt: # Reset by pressing CTRL + C
                print("Measurement stopped by User")

        #fly_forward(master, 2) 
        #return_home(master)
        land(master)     
        disarm_drone(master)
        print("Mission Complete")

    else:
        print("Pre-arm check failed. Cannot arm or take off.")
else:
        print("Can't connect to vehicle")
