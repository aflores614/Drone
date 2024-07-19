from pymavlink import mavutil
import time

# Connect to the vehicle on the specified serial port and baud rate
def connect_to_vehicle():
    try:
        print("Connecting to vehicle on /dev/ttyAMA0 with baud rate 57600...")
        master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
        print("Waiting for heartbeat...")
        master.wait_heartbeat()
        print("Heartbeat received")
        return master
    except Exception as e:
        print(f"Failed to connect: {e}")
        return None


# Function to arm the drone
def arm_drone(master):
    print("Arming drone...")
    master.arducopter_arm()
    print("Motor Arming")
    master.motors_armed_wait()
    print("Drone armed")

# Function to change the mode
def set_mode(master,mode):
    if mode not in master.mode_mapping():
        print(f"Mode {mode} not supported")
        return False

    mode_id = master.mode_mapping()[mode]
    print(f"Setting mode: {mode}")
    master.set_mode(mode_id)

    # Confirm the mode change
    ack = False
    while not ack:
        msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if msg and msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            if msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                print(f"Mode {mode} set successfully")
                ack = True
            else:
                print(f"Failed to set mode {mode}: {msg.result}")
                return False
        time.sleep(1)
    return True

# Function to take off
def takeoff(master,altitude):
    if not set_mode(master,'GUIDED'):
        print("Failed to set GUIDED mode. Check GPS signal, pre-arm checks, and parameters.")
        return

    print(f"Taking off to {altitude} meters")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0,
        altitude)

    # Wait until the vehicle reaches the target altitude
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            current_altitude = msg.relative_alt / 1000.0  # Altitude in meters
            print(f"Current altitude: {current_altitude}")
            if current_altitude >= altitude - 0.1:  # Allow a small margin
                print(f"Reached target altitude of {altitude} meters")
                break
        time.sleep(1)

# Function to check pre-arm status
def check_pre_arm(master):
    # Add specific checks here based on your setup
    print("Checking pre-arm status...")
    # For example, check GPS lock
    gps_lock = False
    while not gps_lock:
        print("Waiting ")
        msg = master.recv_match(blocking=True, timeout = 10)
        if msg:
            print(f"Received message:{msg.get_type()}")
            if msg.get_type() == 'GPS_RAW_INT' and msg.fix_type >= 3:
                gps_lock = True
                print("GPS lock acquired")
                
            elif msg.get_type () == 'GLOBAL_POSITION_INT':
                gps_lock = True
                print("Position information received")
            gps_lock = True
        else:
            print("No message received, retrying ...")
            time.sleep(1)
            
    print("Pre-arm check passed")
    return True

master = connect_to_vehicle()

if master:
    # Perform pre-arm check
    if check_pre_arm(master):
        arm_drone(master)
        takeoff(master,0.4)
    else:
        print("Pre-arm check failed. Cannot arm or take off.")
else:
        print("Can't connect to vehicle")
