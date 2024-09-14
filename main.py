from pymavlink import mavutil
import time
import sys
import logging
import threading
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
from set_movment import fly_movment, fly_to_waypoint, fly_foward_meters
from travel_distance import distance_travel
from abort_mission import abort_mission
from Safe_Test import saftey_test_1,  saftey_test_2
from Battery_Info import Battery_Volatage
from motor_servo import  lidar_motor

target_distance = 7 # distance in meters
current_distance = 0 # The distance the drone has traveled so far
velocity_x = 0.5 # forward speed at 1 m/s
velocity_y = 0 # Right speed at 0.0 m/s
velocity_z = 0 # Down speed at 0.0 m/s
neg_velocity_x = -velocity_x # backward speed at 0.5 m/s
check_interval = 0.5 # The time interval between each check of the distance
ALT = 2.5 # fix altitude
Safe_Dist = 0.75 # safe distance

logging.basicConfig(filename='drone_log.log', 
                        level=logging.INFO,
                        format='%(asctime)s - %(levelname)s - %(message)s',
                        filemode='w')  
logging.info("Start")

master = connect_to_vehicle()

if master:
    # Perform pre-arm check
    if check_pre_arm(master):
        # get take off  coordinate 
        print("Getting Home Position")
        Home_lat, Home_lon, Home_alt = get_location(master) 
        print("Home postion is set:")
        print(Home_lat, Home_lon, Home_alt) 
        logging.info("Home Position: %f, %f, %f" % (Home_lat, Home_lon, Home_alt))

        # Arm drone 
        arm_drone(master)
        
        while not is_armed(master):            
            print("Drone is not armed.")      
            time.sleep(1)
            arm_drone(master)

        print("Drone is armed!")

        logging.info("Drone is Armed")
        time.sleep(5) #drone will be arm for 5 secounds
        
        takeoff(master, ALT, 2)  #takeoff to fix alitiude and hold for fix time
        try:
            logging.info("Flying to Target Distance Test Start")
            print("Flying to Target Distance \nTest Start")


            Start_lat, Start_lon, Start_alt = get_location(master) 
            print("Start postion is set:")
            logging.info("Start Position: %f, %f, %f" % (Start_lat, Start_lon, Start_alt))

            while current_distance < target_distance:            
                    print("Flying Forward")
                    logging.info("fly_movment called with vx=%.2f, vy=%.2f, vz=%.2f, ALT=%.2f", velocity_x, velocity_y, velocity_z, ALT)
                    fly_movment(master, velocity_x, velocity_y, velocity_z, ALT, Safe_Dist, current_distance, target_distance,Start_lat, Start_lon  ) 
                    Current_lat, Current_lon, Current_alt = get_location(master) 
                    current_distance = distance_travel(Home_lat, Current_lat, Home_lon, Current_lon)
                    print("Distance Travel: ", current_distance)           
                    

            logging.info("Target distance of %.2f meters reached.", target_distance) 
            print(f"Target distance of {target_distance} meters reached.")  
              
            print("Flying back home")
            fly_to_waypoint(master, Home_lat, Home_lon, ALT)            
             
                    
        except KeyboardInterrupt:            
            land(master)     
            disarm_drone(master)
            logging.warning("Measurement interrupted by user")
            print("Measurement stopped by User")
            print("Not safe abort mission")
            print("Mission fail")
            sys.exit()

        print("Mission Complete")        
        land(master)     
        disarm_drone(master)
        LAND_lat, LAND_lon, LAND_alt = get_location(master) 
        total_distance_travel = distance_travel(Home_lat, LAND_lat, Home_lon, LAND_lon)
        logging.info("Land Position: %f, %f, %f" % (LAND_lat, LAND_lon, LAND_alt))
        print("Total Distance Tavel: ", total_distance_travel )
        logging.info("Total Distance Travel (GPS) : %f" % (total_distance_travel))
        sys.exit()
        

    else:
        print("Pre-arm check failed. Cannot arm or take off.")
else:
        print("Can't connect to vehicle")

