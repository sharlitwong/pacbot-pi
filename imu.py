import time
import board
import busio
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
from adafruit_bno055 import BNO055_I2C

# ── Config ───────────────────────────────────────────────
MAX_POINTS = 200
SAMPLE_INTERVAL = 0.05

# ── IMU init ─────────────────────────────────────────────
def init_sensor():
    i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
    bno = BNO055_I2C(i2c)
    time.sleep(1)
    return i2c, bno

# ── Data buffers ──────────────────────────────────────────
times   = deque(maxlen=MAX_POINTS)
rolls   = deque(maxlen=MAX_POINTS)
pitches = deque(maxlen=MAX_POINTS)
yaws    = deque(maxlen=MAX_POINTS)
start_time = time.monotonic()

# ── Plot setup ────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(10, 5))
ax.set_title("BNO055 — Roll / Pitch / Yaw")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Degrees")
ax.set_ylim(-180, 360)
ax.grid(True, alpha=0.3)

line_roll,  = ax.plot([], [], label="Roll",  color="tab:blue")
line_pitch, = ax.plot([], [], label="Pitch", color="tab:orange")
line_yaw,   = ax.plot([], [], label="Yaw",   color="tab:green")
ax.legend(loc="upper left")

print("time_s,roll,pitch,yaw")  # CSV header

i2c, bno = init_sensor()

def update(_frame):
    global i2c, bno
    try:
        yaw, roll, pitch = bno.euler  # BNO055 returns (heading, roll, pitch)
        if None in (yaw, roll, pitch):
            return line_roll, line_pitch, line_yaw

        t = time.monotonic() - start_time
        times.append(t)
        rolls.append(roll)
        pitches.append(pitch)
        yaws.append(yaw)

        print(f"{t:.3f},{roll:.3f},{pitch:.3f},{yaw:.3f}")

        line_roll.set_data(times, rolls)
        line_pitch.set_data(times, pitches)
        line_yaw.set_data(times, yaws)
        ax.set_xlim(max(0, t - MAX_POINTS * SAMPLE_INTERVAL), t + 1)

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
    interval=int(SAMPLE_INTERVAL * 1000),
    blit=True,
    cache_frame_data=False
)

plt.tight_layout()
plt.show()
