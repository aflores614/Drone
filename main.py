from pymavlink import mavutil
import time
import sys
import logging
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
from distance_sensor import get_distance
from set_movment import fly_movment, fly_to_postion
from travel_distance import distance_travel
from abort_mission import abort_mission
from Safe_Test import saftey_test_1,  saftey_test_2

logging.basicConfig(filename='drone_log.log', 
                        level=logging.INFO,
                        format='%(asctime)s - %(levelname)s - %(message)s',
                        filemode='w')  
logging.info("Start")

dist_front, dist_back, dist_right, dist_left = get_distance()
print(dist_front, dist_back, dist_right, dist_left) #testing if the distance sensor are reading before arming the drone

master = connect_to_vehicle()
Alt = 1 # fix altitude
Safe_Dist = 1.5 # safe distance

if master:
    # Perform pre-arm check
    if check_pre_arm(master):
        # get take off  coordinate 
        Home_lat, Home_lon, Home_alt = get_location(master) 
        print("Home postion is set:")
        print(Home_lat, Home_lon, Home_alt) 
        logging.info("Home Position: %f, %f, %f" % (Home_lat, Home_lon, Home_alt))
        # Arm drone 
        arm_drone(master)
        if is_armed(master):
            print("Drone is armed!")
        else:
            print("Drone is not armed.")      
            sys.exit()
        # let arm for a fix time
        time.sleep(5)
        # take to a fix altitude and hold for a fix time
        takeoff(master, Alt, 2) 
        
        #scan for any obstacle before flying forward
        try:
            saftey_test_1(master, Safe_Dist )                     
        except KeyboardInterrupt: # Reset by pressing CTRL + C
            abort_mission(master)
            logging.warning("Safty Test 1 fail")
            print("Not safe abort mission")
            print("Safty Test 1 fail")
                
                
        try:
            saftey_test_2(master, Home_lat, Home_lon, Alt ) 
        except KeyboardInterrupt: # Reset by pressing CTRL + C
            abort_mission(master)
            logging.warning("Safty Test 2 fail")
            print("Flying Test has be stopped by User")
            print("Safty Test 2 fail")
                
        target_distance = 1.15 # distance in meters
        current_distance = 0 # The distance the drone has traveled so far
        velocity_x = 0.5 # forward speed at 0.5 m/s
        
        neg_velocity_x = -velocity_x # backward speed at 0.5 m/s
        check_interval = 0.1 # The time interval between each check of the distance
        count = 0 # Counter to track how long the obstacle has been detected. 

       
        try:
            while current_distance < target_distance:

                dist_front, dist_back, dist_right, dist_left = get_distance()
                print("Distance front: ",dist_front)
                if( dist_front > Safe_Dist ): 
                    fly_movment(master, velocity_x, 0, 0,check_interval) 
                    current_distance += velocity_x * check_interval
                    print("Distance travel: ", current_distance)
                    logging.info("Distance traveled: %.2f meters" % current_distance)
                    count = 0
                
                elif( dist_front <= Safe_Dist ):
                    fly_movment( master, neg_velocity_x , 0 , 0,check_interval)                        
                    print("Obstacle detected")                     
                    current_distance += neg_velocity_x * check_interval
                    print("Distance travel: ", current_distance) 
                    logging.info("Distance traveled after obstacle: %.2f meters" % current_distance)            
                    count += 1 
                    if (count == 50 ): # obstacle doesn't move for 5 secounds
                        break
                else:
                    abort_mission(master)
                    logging.error("Invalid distance sensor reading, aborting mission")
                    print("Invalid Distance sensor read abort mission")
                    
                    
        except KeyboardInterrupt:            
            land(master)     
            disarm_drone(master)
            sys.exit()
            logging.warning("Measurement interrupted by user")
            print("Measurement stopped by User")
            print("Not safe abort mission")
            print("Mission fail")
                    
        
        #return_home(master)

        abort_mission(master)
        print("Mission Complete")

    else:
        print("Pre-arm check failed. Cannot arm or take off.")
else:
        print("Can't connect to vehicle")

