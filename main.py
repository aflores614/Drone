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
from Battery_Info import Battery_Volatage

target_distance = 4 # distance in meters
current_distance = 0 # The distance the drone has traveled so far
velocity_x = 1 # forward speed at 0.5 m/s
velocity_y = 0 # Right speed at 0.5 m/s
velocity_z = 0 # Down speed at 0.5 m/s
neg_velocity_x = -velocity_x # backward speed at 0.5 m/s
check_interval = 0.5 # The time interval between each check of the distance
count = 0 # Counter to track how long the obstacle has been detected. 
ALT = 1.5 # fix altitude
Safe_Dist = 1 # safe distance

logging.basicConfig(filename='drone_log.log', 
                        level=logging.INFO,
                        format='%(asctime)s - %(levelname)s - %(message)s',
                        filemode='w')  
logging.info("Start")


dist_front = get_distance()
logging.info("Sensor readings before Takeoff - Front: %.2f",
                            dist_front)
print(dist_front) #testing if the distance sensor are reading before arming the drone

master = connect_to_vehicle()

Battery_voltage = Battery_Volatage(master)
#logging.info("Drone  battery is at: %.2f V" % Battery_voltage)
print("Drone  battery is at ", Battery_voltage,"V")

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
            logging.info("Drone is Armed")
        else:
            print("Drone is not armed.")      
            #sys.exit()
        # let arm for a fix time
        time.sleep(5)
        # take to a fix altitude and hold for a fix time
        takeoff(master, ALT, 2) 
        
        #scan for any obstacle before flying forward
        try:
            saftey_test_1(master, Safe_Dist )                     
        except KeyboardInterrupt: # Reset by pressing CTRL + C
            logging.warning("Safty Test 1 fail")
            print("Not safe abort mission")
            print("Safty Test 1 fail")
            abort_mission(master)   
      
        try:
            #saftey_test_2(master, Home_lat, Home_lon, ALT ) 
            print("Skip test 2")
        except KeyboardInterrupt: # Reset by pressing CTRL + C
            abort_mission(master)
            logging.warning("Safty Test 2 fail")
            print("Flying Test has be stopped by User")
            print("Safty Test 2 fail")

        try:
            logging.info("Flying to Target Distance Test Start")
            print("Flying to Target Distance Test Start")
            while current_distance < target_distance:

                dist_front = get_distance()
                logging.info("Sensor readings - Front: %.2f",dist_front)
                print("Distance front: ",dist_front)

                if( dist_front is None or dist_front <= 0):
                    logging.warning("No reading from distance sensor")
                    print("No reading from distance sensor")
                    break
                elif( dist_front > Safe_Dist ): 
                    print("Flying Forward")
                    logging.info("fly_movment called with vx=%.2f, vy=%.2f, vz=%.2f, ALT=%.2f", velocity_x, velocity_y, velocity_z, ALT)
                    current_distance = fly_movment(master, velocity_x, velocity_y, velocity_z, ALT, Safe_Dist, current_distance, target_distance ) 
                    print("Distance Travel: ", current_distance)
                elif( dist_front <= Safe_Dist ):
                    print("Flying Backward")
                    logging.warning("Obstacle detected at %.2f meters in front. Moving backward.", dist_front)
                    logging.info("fly_movment called with vx=%.2f, vy=%.2f, vz=%.2f, ALT=%.2f", neg_velocity_x, velocity_y, velocity_z, ALT)
                    current_distance = fly_movment(master, neg_velocity_x, velocity_y, velocity_z, ALT, Safe_Dist, current_distance, target_distance )                          
                else:
                    logging.error("Invalid distance sensor reading, aborting mission")
                    print("Invalid Distance sensor read abort mission")
                    abort_mission(master)
                    

            logging.info("Target distance of %.2f meters reached.", target_distance) 
            print(f"Target distance of {target_distance} meters reached.")     
                    
        except KeyboardInterrupt:            
            land(master)     
            disarm_drone(master)
            logging.warning("Measurement interrupted by user")
            print("Measurement stopped by User")
            print("Not safe abort mission")
            print("Mission fail")
            sys.exit()

        print("Mission Complete")
        abort_mission(master)
        

    else:
        print("Pre-arm check failed. Cannot arm or take off.")
else:
        print("Can't connect to vehicle")

