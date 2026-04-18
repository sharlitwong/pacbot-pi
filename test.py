import serial
import time

port = "/dev/serial0"
baud = 9600

import struct

def send_motors(ser, front, right, back, left):
    # scale from -1000:1000 to -32767:32767
    def scale(val):
        return int(val * 32767 / 1000)
    print("Sent\n")
    data = struct.pack('>hhhh', scale(front), scale(right), scale(back), scale(left))
    ser.write(data)
    print(f"Sent {data}")


try:
    ser = serial.Serial(port, baud, timeout=1)
    # Allow the port to initialize
    time.sleep(2)
    while True:
        if ser.is_open:
            # Write the letter 's' as bytes
                #which direction/speed
                #66 is front; 62 is back; 6C is left; 72 is right
                # send_motors(ser, 1000, 0, -1000, 0)

                send_motors(ser, 1000, 1000, 0, 0) 
                # send_motors(ser, 0, 1000, 0, -1000) # forward: right clockwise, left counterclockwise
                # ser.write(b'\x73\x01\xF4\x00\x00\x00\x00\x00') 
                time.sleep(0.1)
         #   print(fSent s over {port} at {baud} baud.)
        
except Exception as e:
    print("ERROR")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
