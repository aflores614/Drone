from pymavlink import mavutil
import math
import time
from  distance_sensor import distance
from get_location import get_location

def fly_movment(master, vx, vy, vz, ALT,fly_time):
   
    # vx value for foward
    # vy value for right
    # vz value for down negative value for up 
    # Bitmask to indicate to use Velocity only and Z position (positive is down) : 0b110111000011
    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, master.target_system,  
                                                                                 master.target_component, 
                                                                                 mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                                                                                 int(0b110111000011 ), 
                                                                                 0, 0, -ALT, 
                                                                                 vx, vy , vz, 
                                                                                 0, 0, 0, 
                                                                                 0, 0 
                                                                                ))
    time.sleep(fly_time)
    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, master.target_system,  
                                                                                 master.target_component, 
                                                                                 mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                                                                                 int(0b110111000011 ), 
                                                                                 0, 0, -ALT, 
                                                                                 0, 0 , 0, 
                                                                                 0, 0, 0, 
                                                                                 0, 0 
                                                                                ))
def fly_to_postion(master, lat, lon, ALT):
    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, master.target_system,  
                                                                                 master.target_component, 
                                                                                 mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
                                                                                 int(0b110111111000), 
                                                                                 int(lat*1e7), int(lon* 1e7), ALT, 
                                                                                 0, 0 , 0, 
                                                                                 0, 0, 0, 
                                                                                 0, 0 
                                                                                ))
    tolerance=0.00001 # how close the drone needs to get to the target position before the loop breaks
    while True:
        current_lat, current_lon, current_alt = get_location(master)
        if(abs(abs(lat) - abs(current_alt)) < tolerance and
           abs(abs(lon) - abs(current_lon)) < tolerance ):
            print("Reach target position")
            break
        else:
            print("Enroute to target Position")
        time.sleep(0.1)
        
