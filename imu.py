import time
import board
import busio
from adafruit_bno08x import (
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C

i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
# i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
bno = BNO08X_I2C(i2c)
time.sleep(1)

bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

while True:
    try: 
        time.sleep(0.5)
        print("Rotation Vector Quaternion:")
        quat_i, quat_j, quat_k, quat_real = bno.quaternion # pylint:disable=no-member
        print("I: %0.6f J: %0.6f K: %0.6f Real: %0.6f" % (quat_i, quat_j, quat_k, quat_real))
        print("")
    
    except RuntimeError as e:
        if "Unprocessable Batch bytes" in str(e):
            pass   # BASE_TIMESTAMP fragment — safe to skip
        else:
            raise 