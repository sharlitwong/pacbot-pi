import serial
import time

port = "/dev/serial0"
baud = 9600

try:
    ser = serial.Serial(port, baud, timeout=1)
    # Allow the port to initialize
    time.sleep(2)
    while True:
        if ser.is_open:
            # Write the letter 's' as bytes
                #which direction/speed
                ser.write(b'\x66\x01\xF4\x00\x00\x00\x00\x00') 
                time.sleep(0.1)

                # ser.write(b'\0x01')
                # time.sleep(0.1)
         #   print(fSent s over {port} at {baud} baud.)
        
except Exception as e:
    print("ERROR")
finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
