#COPY of control.py

import math
import time
import struct
import serial
import board
import busio
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
from adafruit_bno08x.i2c import BNO08X_I2C

port = "/dev/serial0"
baud = 9600

import struct

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

# ────────────────────────────────────────────────────────
class test_mov():
    def __init__(self, m1_s=1000,m2_s=1000,m3_s=1000,m4_s=890):
        #m4 = front
        self.ser = serial.Serial(port, baud, timeout=1)
        self.m1_speed=m1_s
        self.m2_speed=m2_s
        self.m3_speed=m3_s
        self.m4_speed=m4_s
        self.command_list=["s1","s2","s3","s4","f","b","r","l","s","tr"]
        pass
    
    def send_motors(self,sp1,sp2,sp3,sp4):
        # scale from -1000:1000 to -32767:32767
        def scale(val):
            return int(val * 32767 / 1000)
 #       print("Sent\n")
        data = struct.pack('>hhhh', scale(sp1), scale(sp2), scale(sp3), scale(sp4))
        self.ser.write(data)
        self.ser.write(data)
        print(f"Sent {data}")
    
    def try_error(self, len,command_list, num):
        if(command_list<len):
            return True
        return False
    #left front right back
    def take_command(self):
        print("input command")
        in2=input()
        command_components=in2.split()
        command=command_components[0]
        if(command_components[0] in self.command_list):
            if(command=="s1"):
                try:
                    self.m1_speed=int(command_components[1])
                except Exception as e:
                    print(e)
                
                pass
            elif(command=="s2"):
                try:
                    self.m2_speed=int(command_components[1])
                except Exception as e:
                    print(e)
                pass
            elif(command=="s3"):
                try:
                    self.m3_speed=int(command_components[1])
                except Exception as e:
                    print(e)
                pass
            elif(command=="s4"):
                try:
                    self.m4_speed=int(command_components[1])
                except Exception as e:
                    print(e)
                pass
            elif(command=="f"):
                self.send_motors(self.m1_speed,0,-1*self.m3_speed,0)
                pass
            elif(command=="b"):
                self.send_motors(-1*self.m1_speed,0,self.m3_speed,0)
                pass
            elif(command=="r"): 
                self.send_motors(0,self.m2_speed,0,-1*self.m4_speed)
                pass
            elif(command=="l"):
                self.send_motors(0,-1*self.m2_speed,0,self.m4_speed)
                pass
            elif(command=="s"):
                self.send_motors(0,0,0,0)
                pass
            elif(command=="a"):
                self.send_motors(self.m1_speed,self.m2_speed,self.m3_speed,self.m4_speed)
            elif(command=="tr"):
                self.send_motors(-1*self.m1_speed,self.m2_speed,self.m3_speed,-1*self.m4_speed)
            else:
                print("command not recognized")
    def close(self):
        if 'ser' in locals() and self.ser.is_open:
            self.ser.close()


if __name__ == "__main__":
    bot_mov=test_mov()
    bot_mov.send_motors(0,0,0,0)
    try:
        while(True):
            bot_mov.take_command()
            pass
    except KeyboardInterrupt as e:

        bot_mov.send_motors(0,0,0,0)
        bot_mov.close()
        exit



