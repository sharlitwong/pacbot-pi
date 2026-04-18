import math
import time
import struct
import serial
import board
import busio
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
from adafruit_bno08x.i2c import BNO08X_I2C


# ── motor serial ────────────────────────────────────────
def init_serial(port="/dev/serial0", baud=9600):
    ser = serial.Serial(port, baud, timeout=1)
    time.sleep(2)
    return ser


def send_motors(ser, front, right, back, left):
    def scale(val):
        return int(max(-32767, min(32767, val * 32767 / 1000)))
    data = struct.pack('>hhhh', scale(front), scale(right), scale(back), scale(left))
    ser.write(data)


# ── IMU ─────────────────────────────────────────────────
def init_sensor():
    i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
    bno = BNO08X_I2C(i2c)
    time.sleep(1)
    bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    return i2c, bno


def get_yaw(bno):
    quat_i, quat_j, quat_k, quat_real = bno.quaternion
    yaw = math.atan2(2 * quat_real * quat_k, 1 - 2 * quat_k**2)
    return math.degrees(yaw) % 360


def angle_error(setpoint, current):
    return (setpoint - current + 180) % 360 - 180


# ── PID ─────────────────────────────────────────────────
class PID:
    def __init__(self, kp, ki, kd, integral_limit=50):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self._integral = 0
        self._last_error = None
        self._last_time = None

    def compute(self, error):
        now = time.monotonic()
        dt = 0 if self._last_time is None else now - self._last_time
        self._integral = max(-self.integral_limit,
                             min(self.integral_limit, self._integral + error * dt))
        derivative = ((error - self._last_error) / dt
                      if self._last_error is not None and dt > 0 else 0)
        self._last_error = error
        self._last_time = now
        return self.kp * error + self.ki * self._integral + self.kd * derivative


# ── config ──────────────────────────────────────────────
BASE_SPEED = 600     # out of 1000
TARGET_YAW = None    # set to None to lock to heading at startup
# KP, KI, KD = 5.0, 0.01, 0.1
KP, KI, KD = 1.0, 0, 0
# ────────────────────────────────────────────────────────

i2c, bno = init_sensor()
ser = init_serial()
pid = PID(KP, KI, KD)

# lock heading at startup if no target set
if TARGET_YAW is None:
    TARGET_YAW = get_yaw(bno)
    print("Locked heading: %.1f°" % TARGET_YAW)

while True:
    try:
        time.sleep(0.05)  # 20 Hz

        yaw = get_yaw(bno)
        error = angle_error(TARGET_YAW, yaw)
        correction = pid.compute(error)

        # clamp correction so it doesn't exceed motor range
        correction = max(-400, min(400, correction))

        right =  BASE_SPEED + correction
        left  = -(BASE_SPEED - correction)  # left motor runs opposite for forward

        # send_motors(ser, front=0, right=right, back=0, left=left)\
        send_motors(ser, front=0, right=0, back=0, left=0)
        print("Yaw: %0.1f°  Error: %0.1f°  Correction: %0.1f" % (yaw, error, correction))

    except RuntimeError as e:
        if "Unprocessable Batch bytes" in str(e):
            pass
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

    except serial.SerialException as e:
        print("Serial error, reinitializing: %s" % e)
        send_motors(ser, front=0, right=0, back=0, left=0)
        time.sleep(0.5)
        ser = init_serial()


