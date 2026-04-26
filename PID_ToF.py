# COPY of control.py by Spencer, Phuong & Charlotte, for Tufts Pacbot
import math
import time
import struct
import serial
import board
import busio
import threading
from adafruit_bno055 import BNO055_I2C
import adafruit_tca9548a
import adafruit_vl53l4cd

OBSTACLE_THRESHOLD_CM = 2.0  # stop if any sensor reads <= this

def init_imu(i2c):
    bno = BNO055_I2C(i2c)
    time.sleep(1)
    return bno

def get_yaw(bno):
    while True:
        heading, _, _ = bno.euler
        if heading is not None and -360 <= heading <= 360:
            return heading % 360

def angle_error(setpoint, current):
    return (current - setpoint + 180) % 360 - 180

def setup_tof_sensors(tca, channels, timing_budget=200, inter_measurement=0):
    """Initialize VL53L4CD sensors on given TCA9548A channels."""
    sensors = []
    for ch in channels:
        sensor = adafruit_vl53l4cd.VL53L4CD(tca[ch])
        sensor.timing_budget = timing_budget
        sensor.inter_measurement = inter_measurement
        sensor.start_ranging()
        sensors.append(sensor)
        print(f"ToF sensor on channel {ch} initialized.")
    return sensors

def read_tof_sensors(sensors):
    """Read distance in cm from each sensor. Returns list; None on error."""
    readings = []
    for sensor in sensors:
        try:
            while not sensor.data_ready:
                pass
            dist = sensor.distance
            sensor.clear_interrupt()
            readings.append(dist)
        except Exception as e:
            print(f"ToF read error: {e}")
            readings.append(None)
    return readings

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
        now = time.monotonic()
        dt = 0 if self._last_time is None else now - self._last_time
        self._integral = max(-self.integral_limit,
                             min(self.integral_limit, self._integral + error * dt))
        derivative = ((error - self._last_error) / dt
                      if self._last_error is not None and dt > 0 else 0)
        self._last_error = error
        self._last_time = now
        return self.kp * error + self.ki * self._integral + self.kd * derivative


class test_mov():
    PORT = "/dev/serial0"
    BAUD = 9600
    KP, KI, KD = 150, 0, 0.0
    MAX_CORRECTION = 150
    # Maps direction command → which sensor indices are "in front of" motion
    # Sensor order: [front=0, left=1, back=2, right=3]  (matches your channels)
    DIR_SENSORS = {
        "f": [0],       # moving forward → check front sensor
        "b": [2],       # moving backward → check back sensor
        "r": [3],       # moving right → check right sensor
        "l": [1],       # moving left → check left sensor
        "a": [0,1,2,3], # arbitrary → check all
    }

    def __init__(self, i2c, m1_s=761, m2_s=800, m3_s=800, m4_s=800):
        self.ser = serial.Serial(self.PORT, self.BAUD, timeout=1)
        time.sleep(2)
        self.i2c = i2c

        self.m1_speed = m1_s
        self.m2_speed = m2_s
        self.m3_speed = m3_s
        self.m4_speed = m4_s

        # IMU + PID
        self.bno = init_imu(self.i2c)
        self.pid = PID(self.KP, self.KI, self.KD)
        self.target_yaw = get_yaw(self.bno)
        self.use_pid = False
        print("Locked heading: %.1f°" % self.target_yaw)

        # ToF sensors — same I2C bus, TCA on default address 0x70
        self.tca = adafruit_tca9548a.TCA9548A(self.i2c)
        self.tof_sensors = setup_tof_sensors(self.tca, channels=[0, 1, 2, 3])
        self._tof_lock = threading.Lock()  # guard sensor reads across threads

        self._active_direction = None  # tracks which way we're currently moving
        self._move_thread = None

        self.command_list = ["s1","s2","s3","s4","f","b","r","l","s","tr"]

    # ── ToF obstacle check ───────────────────────────────
    def _obstacle_in_direction(self, direction):
        """
        Returns True if any sensor relevant to `direction` reads <= threshold.
        Always returns False if a sensor read fails (fail-open so we don't
        false-stop, but you can flip this to True for a fail-safe policy).
        """
        sensor_indices = self.DIR_SENSORS.get(direction, list(range(4)))
        with self._tof_lock:
            readings = read_tof_sensors(self.tof_sensors)
        for idx in sensor_indices:
            dist = readings[idx]
            if dist is not None and dist <= OBSTACLE_THRESHOLD_CM:
                print(f"⚠️  Obstacle! Sensor {idx} = {dist:.1f} cm — STOPPING")
                return True
        return False

    def send_motors(self, sp1, sp2, sp3, sp4):
        def scale(val):
            val = max(-1000, min(1000, val))
            return int(val * 32767 / 1000)
        data = struct.pack('>hhhh', scale(sp1), scale(sp2), scale(sp3), scale(sp4))
        self.ser.write(data)
        self.ser.write(data)
        print(f"Sent {data}")

    def send_with_correction(self, left, front, right, back):
        try:
            yaw = get_yaw(self.bno)
        except Exception:
            yaw = self.target_yaw

        error = angle_error(self.target_yaw, yaw)
        DEADBAND = 0.1
        if abs(error) < DEADBAND:
            correction = 0.0
            self.pid._integral = 0
        else:
            correction = self.pid.compute(error)
            correction = max(-self.MAX_CORRECTION, min(self.MAX_CORRECTION, correction))

        print("Yaw: %.1f°  Error: %.1f°  Correction: %.1f" % (yaw, error, correction))

        if left != 0 and right != 0:
            left  += correction
            right += correction
        elif front != 0 and back != 0:
            front += correction
            back  += correction

        print("front: %.1f, back: %.1f, right: %.1f, left: %.1f" % (front, back, right, left))
        self.send_motors(left, front, right, back)

    def _start_move(self, direction):
        self.use_pid = False
        if self._move_thread is not None:
            self._move_thread.join(timeout=0.2)
        self._active_direction = direction
        self.pid.reset()
        self.use_pid = True

    def _stop_move(self):
        self.use_pid = False
        self._active_direction = None
        self.send_motors(0, 0, 0, 0)

    def _movement_loop(self, left, front, right, back, direction):
        while self.use_pid:
            # ── OBSTACLE CHECK (runs every loop iteration) ──
            if self._obstacle_in_direction(direction):
                self._stop_move()
                return          # exit thread — robot is now stopped
            # ── Normal PID-corrected move ───────────────────
            self.send_with_correction(left, front, right, back)
            time.sleep(0.05)

    def take_command(self, command2):
        print("input command")
        command_components = command2.split()
        command = command_components[0]

        if command == "c":
            self.target_yaw = command_components[1]
            self.pid.reset()
        elif command == "kp":
            try: self.pid.kp = float(command_components[1])
            except Exception as e: print(e)
        elif command == "ki":
            try: self.pid.ki = float(command_components[1])
            except Exception as e: print(e)
        elif command == "kd":
            try: self.pid.kd = float(command_components[1])
            except Exception as e: print(e)
        elif command == "ph":
            print("getting current heading:", get_yaw(self.bno))
        elif command == "sh":
            self.target_yaw = get_yaw(self.bno)
        elif command == "s1":
            try: self.m1_speed = int(command_components[1])
            except Exception as e: print(e)
        elif command == "s2":
            try: self.m2_speed = int(command_components[1])
            except Exception as e: print(e)
        elif command == "s3":
            try: self.m3_speed = int(command_components[1])
            except Exception as e: print(e)
        elif command == "s4":
            try: self.m4_speed = int(command_components[1])
            except Exception as e: print(e)

        elif command == "f":
            self._start_move("f")
            self._move_thread = threading.Thread(
                target=self._movement_loop,
                args=(self.m1_speed, 0, -self.m3_speed, 0, "f"),
                daemon=True
            )
            self._move_thread.start()

        elif command == "b":
            self._start_move("b")
            print("back target yaw: %.1f" % self.target_yaw)
            self._move_thread = threading.Thread(
                target=self._movement_loop,
                args=(-1*self.m1_speed, 0, self.m3_speed, 0, "b"),
                daemon=True
            )
            self._move_thread.start()

        elif command == "r":
            self._start_move("r")
            self._move_thread = threading.Thread(
                target=self._movement_loop,
                args=(0, self.m2_speed, 0, -self.m4_speed, "r"),
                daemon=True
            )
            self._move_thread.start()

        elif command == "l":
            self._start_move("l")
            self._move_thread = threading.Thread(
                target=self._movement_loop,
                args=(0, -self.m2_speed, 0, self.m4_speed, "l"),
                daemon=True
            )
            self._move_thread.start()

        elif command == "s":
            self._stop_move()

        elif command == "a":
            self._start_move("a")
            self.send_with_correction(
                left=self.m1_speed, front=self.m2_speed,
                right=self.m3_speed, back=self.m4_speed
            )

        elif command == "tr":
            self.send_motors(-1*self.m1_speed, self.m2_speed, self.m3_speed, -1*self.m4_speed)

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
    i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
    bot_mov = test_mov(i2c)
    bot_mov.send_motors(0, 0, 0, 0)
    try:
        while True:
            command = input()
            bot_mov.take_command(command)
    except KeyboardInterrupt:
        bot_mov.send_motors(0, 0, 0, 0)
        bot_mov.close()
