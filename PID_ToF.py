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
from concurrent.futures import ThreadPoolExecutor

OBSTACLE_THRESHOLD_CM = 4.0

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

def setup_tof_sensors(tca, channels, timing_budget=50, inter_measurement=0):
    """
    timing_budget=50ms is much faster than 200ms while still being reliable
    at short distances. inter_measurement=0 = continuous mode (no idle gap).
    """
    sensors = []
    for ch in channels:
        sensor = adafruit_vl53l4cd.VL53L4CD(tca[ch])
        sensor.timing_budget = timing_budget
        sensor.inter_measurement = inter_measurement
        sensor.start_ranging()
        sensors.append(sensor)
        print(f"ToF sensor on channel {ch} initialized.")
    return sensors

def read_single_sensor(sensor):
    """Read one sensor — called in parallel via ThreadPoolExecutor."""
    try:
        while not sensor.data_ready:
            time.sleep(0.001)   # 1ms sleep instead of busy-spin
        dist = sensor.distance
        sensor.clear_interrupt()
        return dist
    except Exception as e:
        print(f"ToF read error: {e}")
        return None


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
    KP, KI, KD = 105, 525, 7.875
    MAX_CORRECTION = 150

    # which sensor indices to check per direction
    # sensor order: [front=0, left=1, back=2, right=3]
    DIR_SENSORS = {
        "f": [0],   # front
        "b": [1],   # back
        "r": [2],   # right
        "l": [3],   # left
        "a": [0, 1, 2, 3],
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

        # ToF sensors
        self.tca = adafruit_tca9548a.TCA9548A(self.i2c)
        self.tof_sensors = setup_tof_sensors(self.tca, channels=[0, 1, 2, 3])
        self._executor = ThreadPoolExecutor(max_workers=4)  # one thread per sensor

        # Latest distances — updated continuously by _obstacle_monitor_loop
        # Index: [front=0, left=1, back=2, right=3]
        self._latest_distances = [float('inf')] * 4
        self._dist_lock = threading.Lock()

        # Emergency stop flag — set by monitor thread, checked by motor thread
        self._emergency_stop = threading.Event()

        # Active movement direction (used by monitor to know which sensors matter)
        self._active_direction = None
        self._move_thread = None

        # Start the dedicated obstacle monitor thread immediately
        self._monitor_running = True
        self._monitor_thread = threading.Thread(
            target=self._obstacle_monitor_loop, daemon=True
        )
        self._monitor_thread.start()
        print("Obstacle monitor started.")

        self.command_list = ["s1","s2","s3","s4","f","b","r","l","s","tr"]

    # ── Dedicated obstacle monitor — runs independently, always ─────────────
    def _obstacle_monitor_loop(self):
        """
        Runs in its own thread at full speed.
        Reads all 4 sensors in parallel, updates _latest_distances,
        and sets _emergency_stop if anything is too close in the active direction.
        """
        while self._monitor_running:
            # Fire all 4 sensor reads in parallel
            futures = [self._executor.submit(read_single_sensor, s)
                       for s in self.tof_sensors]
            readings = [f.result() for f in futures]  # blocks until all done

            with self._dist_lock:
                for i, r in enumerate(readings):
                    if r is not None:
                        self._latest_distances[i] = r

            # Check if anything is blocking the current movement direction
            direction = self._active_direction
            if direction is not None:
                indices = self.DIR_SENSORS.get(direction, list(range(4)))
                for idx in indices:
                    dist = readings[idx]
                    if dist is not None and dist <= OBSTACLE_THRESHOLD_CM:
                        print(f"⚠️  OBSTACLE: sensor {idx} = {dist:.1f} cm → emergency stop")
                        self._emergency_stop.set()
                        break

            # No sleep here — run as fast as the sensors allow (limited by
            # timing_budget=50ms, so this naturally loops ~every 50ms per sensor,
            # but in parallel so total latency is ~50ms not 200ms)

    # ── Motor send ───────────────────────────────────────────────────────────
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

    # ── Movement control ─────────────────────────────────────────────────────
    def _start_move(self, direction):
        self.use_pid = False
        if self._move_thread is not None:
            self._move_thread.join(timeout=0.2)
        self._emergency_stop.clear()        # clear any previous stop
        self._active_direction = direction  # monitor thread starts watching this direction
        self.pid.reset()
        self.use_pid = True

    def _stop_move(self):
        self.use_pid = False
        self._active_direction = None       # monitor stops caring about direction
        self._emergency_stop.clear()
        self.send_motors(0, 0, 0, 0)

    def _movement_loop(self, left, front, right, back):
        while self.use_pid:
            # Check the emergency stop flag — set by the monitor thread
            if self._emergency_stop.is_set():
                print("Movement thread saw emergency stop — halting.")
                self._stop_move()
                return
            self.send_with_correction(left, front, right, back)
            time.sleep(0.05)

    # ── Command handler ──────────────────────────────────────────────────────
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
                args=(self.m1_speed, 0, -self.m3_speed, 0),
                daemon=True
            )
            self._move_thread.start()

        elif command == "b":
            self._start_move("b")
            print("back target yaw: %.1f" % self.target_yaw)
            self._move_thread = threading.Thread(
                target=self._movement_loop,
                args=(-1*self.m1_speed, 0, self.m3_speed, 0),
                daemon=True
            )
            self._move_thread.start()

        elif command == "r":
            self._start_move("r")
            self._move_thread = threading.Thread(
                target=self._movement_loop,
                args=(0, self.m2_speed, 0, -self.m4_speed),
                daemon=True
            )
            self._move_thread.start()

        elif command == "l":
            self._start_move("l")
            self._move_thread = threading.Thread(
                target=self._movement_loop,
                args=(0, -self.m2_speed, 0, self.m4_speed),
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
        elif command == "cal":
            print("=== Sensor Calibration Mode ===")
            print("Wave your hand in front of each physical sensor.")
            print("Press Ctrl+C to exit calibration.\n")
            try:
                while True:
                    futures = [self._executor.submit(read_single_sensor, s)
                               for s in self.tof_sensors]
                    readings = [f.result() for f in futures]
                    print(f"  [0]={readings[0]:.1f}cm  [1]={readings[1]:.1f}cm  "
                          f"[2]={readings[2]:.1f}cm  [3]={readings[3]:.1f}cm", end="\r")
            except KeyboardInterrupt:
                print("\nCalibration done. Update DIR_SENSORS with correct indices.")
        else:
            print("command not recognized")

    def close(self):
        self._monitor_running = False
        self._monitor_thread.join(timeout=1)
        self._executor.shutdown(wait=False)
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
