from pymavlink import mavutil
import time
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

def read(master):
    while True:
        msg = master.recv_match(type = 'ATTITUDE' , blocking = True)
        print(msg)

master = connect_to_vehicle()

if master:
    read(master)
else:
        print("Can't connect to vehicle")

    
