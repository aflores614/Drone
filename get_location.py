from pymavlink import mavutil

def get_home_location(master):
    msg = master.recv_match(type = "GPS_RAW_INT")
    lat = msg.lat/1e7
    lon = msg.lon/1e7
    alt = msg.alt/1e3
    
    master.mav.set_home_position.send(
        master.target_system,
        lat, lon, alt,
        0,0,0,1,0,0,0,0,0
    )
    print("Home postion is set")
