import serial
import struct

# Open serial port
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

# Function to send a command to set the sample rate
def set_sample_rate(rate):
    if rate < 1 or rate > 250:
        print("Sample rate must be between 1Hz and 250Hz.")
        return

    # Construct the command to set the sample rate
    command = bytearray([0x5A, 0x06, 0x03, rate, 0, 0, 0, 0])
    
    # Calculate the checksum
    checksum = sum(command) % 256
    command[-1] = checksum

    # Send the command
    ser.write(command)
    print(f"Sample rate set to {rate}Hz")

# Function to read and decode data from TF-Luna
def read_lidar_data():
    while True:
        # Read 9 bytes (typical TF-Luna data frame length)
        data = ser.read(9)
        
        # Check for valid data frame starting with 0x59 0x59
        if data[0] == 0x59 and data[1] == 0x59:
            # Extract the distance value (2 bytes: data[2] and data[3])
            distance = data[2] + (data[3] << 8)
            print(f"Distance: {distance} cm")

try:
    # Set the desired sample rate
    set_sample_rate(100)  # Example: Set to 100Hz

    # Start reading data from the TF-Luna
    read_lidar_data()
except KeyboardInterrupt:
    ser.close()
    print("Serial port closed.")
