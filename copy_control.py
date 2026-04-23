# COPY of control.py by Spencer, Phuong & Charlotte, for Tufts Pacbot
# How to use:
# what to type in terminal:
    # cd pacbot
    # source .venv/bin/activate
    # python3 copy_control.py
# now the program is running so you can give it f, b, l, r, s commands (forward,
# back, left, right, stop) commands to make the robot move
# stop the program with Ctrl + "C"

import math
import time
import struct
import serial
import board
import busio
import threading
from adafruit_bno055 import BNO055_I2C
import adafruit_tca9548a

#DOF BNO 0055

#FOR NEW IMU
def init_imu():
    i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
    tca = adafruit_tca9548a.TCA9548A(i2c)   # mux at 0x70
    bno = BNO055_I2C(tca[0])                # channel 0
    time.sleep(1)
    return i2c, bno

#FOR NEW IMU
def get_yaw(bno):
    while True:
        try:
            heading, _, _ = bno.euler  # heading in degrees 0-360
            if heading is not None:
                return heading
        except Exception as e:
            print("I2C error, reinitializing...")
            time.sleep(0.05)
            i2c, bno = init_imu()

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

    def reset(self):
        self._integral = 0
        self._last_error = None
        self._last_time = None

    def compute(self, error):
        now = time.monotonic() #returns current time in seconds as a float, always increasing
        dt = 0 if self._last_time is None else now - self._last_time #small delta time
        #integral approximation
        self._integral = max(-self.integral_limit,
                             min(self.integral_limit, self._integral + error * dt))
        derivative = ((error - self._last_error) / dt
                      if self._last_error is not None and dt > 0 else 0)
        self._last_error = error
        self._last_time = now
        return self.kp * error + self.ki * self._integral + self.kd * derivative
# ────────────────────────────────────────────────────────

class test_mov():
    PORT = "/dev/serial0"
    BAUD = 9600
    KP, KI, KD = 10.0, 0.1, 0.5
    MAX_CORRECTION = 400

    def __init__(self, m1_s=800,m2_s=800,m3_s=800,m4_s=800):
       
        #m4 = front
        self.ser = serial.Serial(self.PORT, self.BAUD, timeout=1)
        time.sleep(2)

        self.m1_speed=m1_s
        self.m2_speed=m2_s
        self.m3_speed=m3_s
        self.m4_speed=m4_s

        # IMU + PID
        self.i2c, self.bno = init_imu()
        self.pid = PID(self.KP, self.KI, self.KD)
        self.target_yaw = get_yaw(self.bno)
        self.use_pid = False   # only active during movement
        print("Locked heading: %.1f°" % self.target_yaw)

        self.command_list=["s1","s2","s3","s4","f","b","r","l","s","tr"]
        pass
    
    def send_motors(self,sp1,sp2,sp3,sp4):
        # scale from -1000:1000 to -32767:32767
        def scale(val):
            val = max(-1000, min(1000, val))
            return int(val * 32767 / 1000)
        data = struct.pack('>hhhh', scale(sp1), scale(sp2), scale(sp3), scale(sp4))
        self.ser.write(data)
        self.ser.write(data)
        print(f"Sent {data}")

    # ── PID-corrected send ───────────────────────────────
    def send_with_correction(self, left, front, right, back, reverse = False):
        """
        Apply yaw PID correction to the left/right pair.
        Positive correction = turn right → speed up right, slow down left.
        Motor order passed to send_motors: (left, front, right, back)
        matching your existing convention.
        """
        try:
            yaw = get_yaw(self.bno)
        except Exception:
            yaw = self.target_yaw   # fall back gracefully

        error = angle_error(self.target_yaw, yaw)
        correction = self.pid.compute(error)
        correction = max(-self.MAX_CORRECTION, min(self.MAX_CORRECTION, correction))

        if reverse:
            correction = -correction  # flip correction direction for backward movement

        print("Yaw: %.1f°  Error: %.1f°  Correction: %.1f" % (yaw, error, correction))

        if left != 0 and right != 0:
            left  += correction
            right -= correction
        elif front != 0 and back != 0:
            front += correction
            back  -= correction
        
        print("front: %.1f, back: %.1f, right: %.1f, left: %.1f" % (front, back, right, left))

        self.send_motors(left, front, -1*right, back)
    
    # ── lock heading & reset PID on each new move ────────
    def _start_move(self):
        # Stop any existing movement thread first
        self.use_pid = False
        if hasattr(self, '_move_thread') and self._move_thread is not None:
            self._move_thread.join(timeout=0.2)  # wait for old thread to die
        
        self.target_yaw = get_yaw(self.bno)
        self.pid.reset()
        self.use_pid = True

    def _stop_move(self):
        self.use_pid = False
        self.send_motors(0, 0, 0, 0)

    #THREADING
    def _movement_loop(self, left, front, right, back, reverse=False):
        while self.use_pid:
            self.send_with_correction(left, front, right, back, reverse=reverse)
            time.sleep(0.05)

    #left front right back
    def take_command(self):
        print("input command")
        in2=input()
        command_components=in2.split()
        command=command_components[0]
        if(command_components[0] in self.command_list):
            if(command == "c"):
                self.target_yaw = command_components[1]
                self.pid.reset()
            elif(command=="s1"):
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
            elif command == "f":
                self._start_move()
                self._move_thread = threading.Thread(
                    target=self._movement_loop,
                    args=(self.m1_speed, 0, self.m3_speed, 0),
                    daemon=True
                )
                self._move_thread.start()

            elif command == "b":
                self._start_move()
                print("back target yaw: %.1f" % self.target_yaw)
                self._move_thread = threading.Thread(
                    target=self._movement_loop,
                    args=(-1*self.m1_speed, 0, -1*self.m3_speed, 0),
                    kwargs={"reverse": True},
                    daemon=True
                )
                self._move_thread.start()

            elif command == "r":
                self._start_move()
                self._move_thread = threading.Thread(
                    target=self._movement_loop,
                    args=(0, self.m2_speed, 0, -self.m4_speed),
                    daemon=True
                )
                self._move_thread.start()

            elif command == "l":
                self._start_move()
                self._move_thread = threading.Thread(
                    target=self._movement_loop,
                    args=(0, -self.m2_speed, 0, self.m4_speed),
                    daemon=True
                )
                self._move_thread.start()
            elif(command=="s"):
                self._stop_move()
                pass
            elif(command=="a"):
                self._start_move()
                self.send_with_correction(
                    left=self.m1_speed, front=self.m2_speed,
                    right=self.m3_speed, back=self.m4_speed
                )
            elif(command=="tr"):
                self.send_motors(-1*self.m1_speed,self.m2_speed,self.m3_speed,-1*self.m4_speed)
            else:
                print("command not recognized")
    def close(self):
        self.send_motors(0, 0, 0, 0)
        if self.ser.is_open:
            self.ser.close()
        try:
            self.i2c.deinit()
        except Exception:
            pass

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



