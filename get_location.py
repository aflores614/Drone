from pymavlink import mavutil
import time
from connect_to_vehicle import connect_to_vehicle

def get_location(master):    
    master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 1000000,
            0,0,0,0,0,0
    )
    while True:
        
        msg = master.recv_match(type = 'GPS_RAW_INT', blocking = True, timeout = 2)              
        if msg is not None:
            lat = msg.lat/1e7
            lon = msg.lon/1e7
            alt = msg.alt/1e3
            print("Got Location")                
            return lat, lon, alt 
            break
        else:
             print("Waiting ...")
if __name__ == "__main__":
    
      master = connect_to_vehicle()
      while True:
        lat,lon,alt = get_location(master)    
        print(lat,lon,alt)   
        time.sleep(2)
