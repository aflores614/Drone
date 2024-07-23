from pymavlink import mavutil
def arm_drone(master):
    # Send a command to arm the drone
    master.mav.command_long_send(
        master.target_system,                
        master.target_component,             
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,  
        0,                                  
        1,                                   # param1 (1 to arm, 0 to disarm)
        0, 0, 0, 0, 0, 0                     
    )

    print("Drone Arm")