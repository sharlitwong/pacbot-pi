import board
import busio
import time
import adafruit_bno055
import numpy as np
# Initialize I2C and Sensor
i2c = busio.I2C(board.SCL, board.SDA)
sensor = adafruit_bno055.BNO055_I2C(i2c)

def get_yaw_from_quat(sensor):
    # BNO055 returns (w, x, y, z)
    w, x, y, z = sensor.quaternion
    
    # Formula for yaw (heading) using z-axis component
    yaw = np.atan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))
    return np.degrees(yaw) % 360

# Alternatively, the BNO055 provides this directly:
# degree = sensor.euler[0]i

while True:
    print(get_yaw_from_quat(sensor))
    time.sleep(0.05)
