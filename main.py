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

#logging.basicConfig(filename='drone_log.log', 
 #                   level=logging.INFO,
  #                  format='%(asctime)s - %(levelname)s - %(message)s',
   #                 filemode='w')  

master = connect_to_vehicle()

Alt = 1.3 # fix altitude
Safe_Dist = 1.5 # safe distance

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
        # let arm for a fix time
        time.sleep(5)
        # take to a fix altitude and hold for a fix time
        takeoff(master, Alt, 2) 
        
        #scan for any obstacle before flying forward
        try:
            print("Stafey Test 1")
            while True:
                dist_front, dist_back, dist_right, dist_left = get_distance()
                #logging.info("Measured front Distance: %.1f m" % dist_front)
                print ("Measured front Distance = %.1f m" % dist_front)
                if( dist_front <= Safe_Dist):
                    print("Object too close")
                  #  logging.warning("Object too close")                        
                elif( dist_front > Safe_Dist ):
                    print("Safe")
                   # logging.info("Safe distance, proceeding")
                    break
                else: # not safe to continue
                     abort_mission(master)
                     print(" not safe to fly abort mission")
                     print("Safty Test 1 Fail")
                    # logging.error("Not safe to fly, aborting mission")
                     
        except KeyboardInterrupt: # Reset by pressing CTRL + C
            abort_mission(master)
            #logging.warning("Movement Test interrupted by user")
            print("Not safe abort mission")
            print("Safty Test 1 fail")
                
                
                
        try:
            print("Safty Test 2")
            current_lat, current_lon, current_alt = get_location(master)
            distance_travel_home = distance_travel(Home_lat, current_lat, Home_lon, current_lon)
            print("The distance from home is ", distance_travel_home,"meters")
            #logging.info("The distance from home is ", distance_travel_home,"meters")
            #fly_to_postion(master, Home_lat, Home_lon, Alt)
        except KeyboardInterrupt: # Reset by pressing CTRL + C
            abort_mission(master)
            print("Not safe abort mission")
            print("Safty Test 2 fail")
                
        try:
            print("Testing Movement")
            current_lat, current_lon, current_alt = get_location(master)
            print("It had travel", distance_travel(Home_lat, current_lat, Home_lon, current_lon),"meters")

            print("Flying Forward")
            fly_movment(master,0.5, 0, 0) 
            time.sleep(1)
            print("Flying Backward")
            fly_movment(master,-0.5, 0, 0)
            time.sleep(1)
            print("Flying Right")
            fly_movment(master,0, 0.5, 0)
            time.sleep(1)
            print("Flying Left")
            fly_movment(master,0, -0.5, 0)
            print("Flying up")
            time.sleep(1)
            fly_movment(master,0, 0, -0.5)
            print("Flying down")           
            fly_movment(master,0, 0, 0.5)
            time.sleep(1)
            print("Flying up")
            time.sleep(1)
            fly_movment(master,0, 0, -0.5)
        except KeyboardInterrupt: # Reset by pressing CTRL + C
            abort_mission(master)
            ##logging.warning("Movement Test interrupted by user")
            print("Flying Test has be stopped by User")
            print("Movement Test  fail")
                




        target_distance = 1 # distance in meters
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
                    fly_movment(master, velocity_x, 0, 0) 
                    time.sleep(check_interval)
                    current_distance += velocity_x * check_interval
                    print("Distance travel: ", current_distance)
              #      logging.info("Distance traveled: %.2f meters" % current_distance)
                    count = 0
                
                elif( dist_front <= Safe_Dist ):
                    fly_movment( master, neg_velocity_x , 0 , 0)                        
                    print("Obstacle detected")                     
                    time.sleep(check_interval)
                    current_distance -= neg_velocity_x * check_interval
                    print("Distance travel: ", current_distance) 
               #     logging.info("Distance traveled after obstacle: %.2f meters" % current_distance)            
                    count += 1 
                    if (count == 50 ): # obstacle doesn't move for 5 secounds
                        break
                else:
                    abort_mission(master)
                #    logging.error("Invalid distance sensor reading, aborting mission")
                    print("Invalid Distance sensor read abort mission")
                    
                    
        except KeyboardInterrupt:
            abort_mission(master)
            #logging.warning("Measurement interrupted by user")
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

