from pymavlink import mavutil
import time
from get_location import get_home_location
from connect_to_vehicle import connect_to_vehicle

master = connect_to_vehicle
get_home_location(master)
