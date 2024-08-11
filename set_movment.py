from pymavlink import mavutil
import math
from  distance_sensor import distance

def fly_movment(master, vx, vy, vz):
    # vx value for foward
    # vy value for right
    # vz value for down negative value for up 
    # Bitmask to indicate to use Velocity : 0b110111000111
    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_local_ned_message(10, master.target_system,  
                                                                                 master.target_component, 
                                                                                 mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
                                                                                 int(0b110111000111 ), 
                                                                                 0, 0, 0, 
                                                                                 vx, vy , vz, 
                                                                                 0, 0, 0, 
                                                                                 0, 0 
                                                                                ))
def fly_to_postion(master, lat, lon, alt):
    master.mav.send(mavutil.mavlink.MAVLink_set_position_target_global_int_message(10, master.target_system,  
                                                                                 master.target_component, 
                                                                                 mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
                                                                                 int(0b110111111000), 
                                                                                 lat*1e7, lat* 1e7, alt, 
                                                                                 0, 0 , 0, 
                                                                                 0, 0, 0, 
                                                                                 0, 0 
                                                                                ))