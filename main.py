from pymavlink import mavutil
import time
from connect_to_vehicle import connect_to_vehicle
from arm_drone import arm_drone
from get_location import get_location
from disarm_drone import disarm_drone
from set_mode import set_mode 
from return_home import return_home
from land import land
from fly_forward import fly_forward
from check_pre_arm import check_pre_arm


def takeoff(master,altitude,hold_time):
    
    if not set_mode(master,'GUIDED'):
        print("Failed to set GUIDED mode. Check GPS signal, pre-arm checks, and parameters.")
        return

    
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0,
        altitude)
    
    print(f"Taking off to {altitude} meters")

    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION,
        1,
        1)
    start_time = time.time()
    timeout = 10
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            relative_altitude = msg.relative_alt/ 1000.0  # Altitude in meters
            print(f"Current altitude: {relative_altitude}")
            if relative_altitude >= altitude - 0.1:  # Allow a small margin                
                print(f"Reached target altitude of {altitude} meters")
                break
            if time.time() - start_time > timeout:
                print("Timeout reached. Unable to reach target altitude.")
                return   
        time.sleep(1)

    print("Holding Altitude")
    time.sleep(hold_time)
    print("Hold time complete")
 


master = connect_to_vehicle()

if master:
    # Perform pre-arm check
    if check_pre_arm(master):
        Home_lat, Home_lon, Home_alt = get_location(master)
        print("Home postion is set")
        print(Home_lat, Home_lon, Home_alt)
        arm_drone(master)      
        takeoff(master,1.5,10) 
        fly_forward(master, 2) 
        #return_home(master)
        land(master)     
        disarm_drone(master)
        print("Mission Complete")

    else:
        print("Pre-arm check failed. Cannot arm or take off.")
else:
        print("Can't connect to vehicle")
