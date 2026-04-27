import time
import board
import busio
import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
from adafruit_bno08x import BNO_REPORT_ROTATION_VECTOR
from adafruit_bno08x.i2c import BNO08X_I2C

# ── Config ───────────────────────────────────────────────
MAX_POINTS = 200        # how many samples to show on the rolling window
SAMPLE_INTERVAL = 0.05  # seconds between reads (~20Hz)

# ── IMU init ─────────────────────────────────────────────
def init_sensor():
    i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
    bno = BNO08X_I2C(i2c)
    time.sleep(1)
    bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
    return i2c, bno

def quaternion_to_euler(quat_i, quat_j, quat_k, quat_real):
    """Convert quaternion to roll, pitch, yaw in degrees."""
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (quat_real * quat_i + quat_j * quat_k)
    cosr_cosp = 1 - 2 * (quat_i ** 2 + quat_j ** 2)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))

    # Pitch (y-axis rotation)
    sinp = 2 * (quat_real * quat_j - quat_k * quat_i)
    sinp = max(-1.0, min(1.0, sinp))  # clamp for safety
    pitch = math.degrees(math.asin(sinp))

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (quat_real * quat_k + quat_i * quat_j)
    cosy_cosp = 1 - 2 * (quat_j ** 2 + quat_k ** 2)
    yaw = math.degrees(math.atan2(siny_cosp, cosy_cosp)) % 360

    return roll, pitch, yaw

# ── Data buffers ──────────────────────────────────────────
times  = deque(maxlen=MAX_POINTS)
rolls  = deque(maxlen=MAX_POINTS)
pitches = deque(maxlen=MAX_POINTS)
yaws   = deque(maxlen=MAX_POINTS)
start_time = time.monotonic()

# ── Plot setup ────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(10, 5))
ax.set_title("BNO08X — Roll / Pitch / Yaw")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Degrees")
ax.set_ylim(-180, 360)
ax.grid(True, alpha=0.3)

line_roll,  = ax.plot([], [], label="Roll",  color="tab:blue")
line_pitch, = ax.plot([], [], label="Pitch", color="tab:orange")
line_yaw,   = ax.plot([], [], label="Yaw",   color="tab:green")
ax.legend(loc="upper left")

print("time_s,roll,pitch,yaw")   # CSV header

i2c, bno = init_sensor()

def update(_frame):
    global i2c, bno

    try:
        qi, qj, qk, qr = bno.quaternion
        if None in (qi, qj, qk, qr):
            return line_roll, line_pitch, line_yaw

        roll, pitch, yaw = quaternion_to_euler(qi, qj, qk, qr)
        t = time.monotonic() - start_time

        times.append(t)
        rolls.append(roll)
        pitches.append(pitch)
        yaws.append(yaw)

        # CSV output — pipe this to a file if you want to save it:
        # python3 imu_plot.py | tee imu_log.csv
        print(f"{t:.3f},{roll:.3f},{pitch:.3f},{yaw:.3f}")

        line_roll.set_data(times, rolls)
        line_pitch.set_data(times, pitches)
        line_yaw.set_data(times, yaws)

        ax.set_xlim(max(0, t - MAX_POINTS * SAMPLE_INTERVAL), t + 1)

    except RuntimeError as e:
        if "Unprocessable Batch bytes" not in str(e):
            raise
    except OSError as e:
        if e.errno == 5:
            print("# I2C error, reinitializing...")
            time.sleep(0.5)
            try:
                i2c.deinit()
            except Exception:
                pass
            i2c, bno = init_sensor()
        else:
            raise

    return line_roll, line_pitch, line_yaw

ani = animation.FuncAnimation(
    fig, update,
    interval=int(SAMPLE_INTERVAL * 1000),  # ms
    blit=True,
    cache_frame_data=False
)

plt.tight_layout()
plt.show()
