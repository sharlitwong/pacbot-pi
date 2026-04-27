import time
import board
import busio
import matplotlib.pyplot as plt
import adafruit_tca9548a
import adafruit_vl53l4cd

# ── Config ───────────────────────────────────────────────
RECORD_SECONDS = 20
SAMPLE_INTERVAL = 0.05
CHANNELS = [0, 1, 2, 3]
LABELS = ["Front (0)", "Back (1)", "Right (2)", "Left (3)"]
COLORS = ["tab:blue", "tab:orange", "tab:green", "tab:red"]

# ── Sensor init ──────────────────────────────────────────
i2c = busio.I2C(board.SCL, board.SDA)
tca = adafruit_tca9548a.TCA9548A(i2c)

sensors = []
for ch in CHANNELS:
    sensor = adafruit_vl53l4cd.VL53L4CD(tca[ch])
    sensor.timing_budget = 50
    sensor.inter_measurement = 0
    sensor.start_ranging()
    sensors.append(sensor)
    print(f"Sensor {ch} initialized.")

# ── Record ───────────────────────────────────────────────
times = []
distances = [[] for _ in CHANNELS]

print(f"\nRecording for {RECORD_SECONDS} seconds... move objects in front of sensors now!")
start = time.monotonic()

while time.monotonic() - start < RECORD_SECONDS:
    t = time.monotonic() - start
    remaining = RECORD_SECONDS - t
    readings = []

    for sensor in sensors:
        try:
            while not sensor.data_ready:
                time.sleep(0.001)
            dist = sensor.distance
            sensor.clear_interrupt()
            readings.append(dist)
        except Exception:
            readings.append(None)

    # only record if all sensors returned valid data
    if None not in readings and all(0 < r <= 400 for r in readings):
        times.append(t)
        for i, r in enumerate(readings):
            distances[i].append(r)
        print(f"[{remaining:4.1f}s left]  "
              f"front={readings[0]:.1f}cm  "
              f"back={readings[1]:.1f}cm  "
              f"right={readings[2]:.1f}cm  "
              f"left={readings[3]:.1f}cm")

print("Done! Saving plot...")

# ── Plot ─────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(12, 5))
for i in range(len(CHANNELS)):
    ax.plot(times, distances[i], label=LABELS[i], color=COLORS[i])

ax.set_title(f"VL53L4CD — Distance per Sensor ({RECORD_SECONDS}s)")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Distance (cm)")
ax.legend(loc="upper left")
ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig("tof_plot.png")
print("Saved to tof_plot.png")
