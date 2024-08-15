from pymavlink import mavutil

def Battery_Volatage(master):
    while True:
        msg = master.recv_match(type='BATTERY_STATUS', blocking=True)
        if msg:
            # The `voltages` field is a list of voltages in millivolts (mV)
            voltage = msg.voltages[0] / 1000.0  # Convert from mV to Volts for the first battery
            return voltage
            