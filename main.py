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
from lidar_distance import get_distance
from set_movment import fly_movment, fly_to_waypoint, fly_foward_meters
from travel_distance import distance_travel
from abort_mission import abort_mission
from Safe_Test import saftey_test_1,  saftey_test_2
from Battery_Info import Battery_Volatage
from motor_servo import  lidar_motor

target_distance = 6 # distance in meters
current_distance = 0 # The distance the drone has traveled so far
velocity_x = 0.5 # forward speed at 1 m/s
velocity_y = 0 # Right speed at 0.0 m/s
velocity_z = 0 # Down speed at 0.0 m/s
neg_velocity_x = -velocity_x # backward speed at 0.5 m/s
check_interval = 0.5 # The time interval between each check of the distance
ALT = 1.5 # fix altitude
Safe_Dist = 0.75 # safe distance

logging.basicConfig(filename='drone_log.log', 
                        level=logging.INFO,
                        format='%(asctime)s - %(levelname)s - %(message)s',
                        filemode='w')  
logging.info("Start")


#servo_thread = threading.Thread(target=lidar_motor)
#servo_thread.start()
dist_front = get_distance()

logging.info("Sensor readings before Takeoff - Front: %.2f",dist_front)
print(dist_front) #testing if the distance sensor are reading before arming the drone

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
        print("begin")
        #scan for any obstacle before flying forward


        try:
            logging.info("Flying to Target Distance Test Start")
            print("Flying to Target Distance \nTest Start")


            Start_lat, Start_lon, Start_alt = get_location(master) 
            print("Start postion is set:")
            logging.info("Start Position: %f, %f, %f" % (Start_lat, Start_lon, Start_alt))

            while current_distance < target_distance:
                dist_front = 8
                logging.info("Sensor readings - Front: %.2f",dist_front)
                print("Distance front: ",dist_front)
                if( dist_front is None ):
                    logging.warning("No reading from distance sensor")
                    print("No reading from distance sensor")
                    break
                elif( dist_front > Safe_Dist ): 
                    print("Flying Forward")
                    logging.info("fly_movment called with vx=%.2f, vy=%.2f, vz=%.2f, ALT=%.2f", velocity_x, velocity_y, velocity_z, ALT)
                    current_distance = fly_movment(master, velocity_x, velocity_y, velocity_z, ALT, Safe_Dist, current_distance, target_distance,Start_lat, Start_lon  ) 
                    print("Distance Travel: ", current_distance)
                elif( dist_front <= Safe_Dist ):
                    print("Flying Backward")
                    logging.warning("Obstacle detected at %.2f meters in front. Moving backward.", dist_front)
                    logging.info("fly_movment called with vx=%.2f, vy=%.2f, vz=%.2f, ALT=%.2f", neg_velocity_x, velocity_y, velocity_z, ALT)
                    current_distance = fly_movment(master, neg_velocity_x, velocity_y, velocity_z, ALT, Safe_Dist, current_distance, target_distance,Start_lat, Start_lon )                          
                else:
                    logging.error("Invalid distance sensor reading, aborting mission")
                    print("Invalid Distance sensor read abort mission")
                    abort_mission(master)
                    

            logging.info("Target distance of %.2f meters reached.", target_distance) 
            print(f"Target distance of {target_distance} meters reached.")    
            print("Flying back home")
            fly_to_waypoint(master, Home_lat, Home_lon, ALT)
            Start_lat, Start_lon, Start_alt = get_location(master) 
            print("Flying Foward pt2")
            fly_foward_meters(master, velocity_x, velocity_y, velocity_z, current_distance, target_distance, Start_lat, Start_lon ) 
                    
        except KeyboardInterrupt:            
            land(master)     
            disarm_drone(master)
            logging.warning("Measurement interrupted by user")
            print("Measurement stopped by User")
            print("Not safe abort mission")
            print("Mission fail")
            sys.exit()

        print("Mission Complete")
        LAND_lat, LAND_lon, LAND_alt = get_location(master) 
        total_distance_travel = distance_travel(Home_lat, LAND_lat, Home_lon, LAND_lon)
        logging.info("Land Position: %f, %f, %f" % (LAND_lat, LAND_lon, LAND_alt))
        print("Total Distance Tavel: ", total_distance_travel )
        logging.info("Total Distance Travel (GPS) : %f" % (total_distance_travel))
        abort_mission(master)
        

    else:
        print("Pre-arm check failed. Cannot arm or take off.")
else:
        print("Can't connect to vehicle")

