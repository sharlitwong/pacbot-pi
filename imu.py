import time
import board
import busio
import math
from adafruit_bno08x import (
    BNO_REPORT_ROTATION_VECTOR,
)
from adafruit_bno08x.i2c import BNO08X_I2C



i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
bno = BNO08X_I2C(i2c)
time.sleep(1)

bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

def get_yaw(bno):
    quat_i, quat_j, quat_k, quat_real = bno.quaternion
    yaw = math.atan2(2 * quat_real * quat_k, 1 - 2 * quat_k**2)
    return math.degrees(yaw) % 360


while True:
    try: 
        time.sleep(0.5)
        print("Rotation Vector Quaternion:")
        quat_i, quat_j, quat_k, quat_real = bno.quaternion # pylint:disable=no-member
        #K component indicates angle from north
        # print("I: %0.6f J: %0.6f K: %0.6f Real: %0.6f" % (quat_i, quat_j, quat_k, quat_real))
        degree = get_yaw(bno)
        print("Angle: %0.6f" %(degree))
        print("")
    
    except RuntimeError as e:
        if "Unprocessable Batch bytes" in str(e):
            pass   # BASE_TIMESTAMP fragment — safe to skip
        else:
            raise 

    except OSError as e:
        if e.errno == 5:
            print("I2C error, reinitializing...")
            time.sleep(0.5)
            try:
                i2c.deinit()
            except Exception:
                pass
            i2c, bno = init_sensor()
        else:
            raise