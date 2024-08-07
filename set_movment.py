from pymavlink import mavutil
import math
from  distance_sensor import distance

def fly_movment(master, vx, vy, vz):
    # vx value for foward
    # vy value for right
    # vz value for down negative value for up 
    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, master.target_system,  
                                                                                 master.target_component, 
                                                                                 mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                                                                                 int(0b110111000111 ) , 
                                                                                 0, 0, 0, 
                                                                                 vx, vy , vz, 
                                                                                 0, 0, 0, 
                                                                                 0, 0 
                                                                                ))