import time
import board
import busio
import matplotlib.pyplot as plt
from adafruit_bno055 import BNO055_I2C

# ── Config ───────────────────────────────────────────────
RECORD_SECONDS = 20
SAMPLE_INTERVAL = 0.05

# ── IMU init ─────────────────────────────────────────────
i2c = busio.I2C(board.SCL, board.SDA)
bno = BNO055_I2C(i2c)
time.sleep(1)

# ── Record ───────────────────────────────────────────────
times, rolls, pitches, yaws = [], [], [], []

print(f"Recording for {RECORD_SECONDS} seconds...")
start = time.monotonic()

while time.monotonic() - start < RECORD_SECONDS:
    try:
        yaw, roll, pitch = bno.euler
        if None not in (yaw, roll, pitch) and all(-360 <= v <= 360 for v in (yaw, roll, pitch)):
            t = time.monotonic() - start
            times.append(t)
            rolls.append(roll)
            pitches.append(pitch)
            yaws.append(yaw)
            print(f"{t:.2f}s  roll={roll:.1f}  pitch={pitch:.1f}  yaw={yaw:.1f}")
    except OSError:
        pass
    time.sleep(SAMPLE_INTERVAL)

print("Done recording. Saving plot...")

# ── Plot ─────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(12, 5))
ax.plot(times, rolls,   label="Roll",  color="tab:blue")
ax.plot(times, pitches, label="Pitch", color="tab:orange")
ax.plot(times, yaws,    label="Yaw",   color="tab:green")
ax.set_title(f"BNO055 — Roll / Pitch / Yaw ({RECORD_SECONDS}s)")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Degrees")
ax.legend(loc="upper left")
ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig("imu_plot.png")
print("Saved to imu_plot.png")
