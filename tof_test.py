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

# Per-sensor max reliable range in cm
MAX_RANGE = {
    0: 60,   # front
    1: 200,  # back
    2: 40,   # right
    3: 60,   # left
}

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
# each sensor gets its own time and distance list since they may
# have different numbers of valid readings
times     = [[] for _ in CHANNELS]
distances = [[] for _ in CHANNELS]

print(f"\nRecording for {RECORD_SECONDS} seconds... move objects in front of sensors now!")
start = time.monotonic()

while time.monotonic() - start < RECORD_SECONDS:
    t = time.monotonic() - start
    remaining = RECORD_SECONDS - t
    readings = []

    for i, sensor in enumerate(sensors):
        try:
            while not sensor.data_ready:
                time.sleep(0.001)
            dist = sensor.distance
            sensor.clear_interrupt()
            if dist is not None and 0 < dist <= MAX_RANGE[i]:
                readings.append(dist)
            else:
                readings.append(None)
        except Exception:
            readings.append(None)

    # record each sensor independently
    if any(r is not None for r in readings):
        for i, r in enumerate(readings):
            if r is not None:
                times[i].append(t)
                distances[i].append(r)

        # build status string, show -- for invalid readings
        status = "  ".join(
            f"{LABELS[i].split()[0].lower()}={readings[i]:.1f}cm"
            if readings[i] is not None else
            f"{LABELS[i].split()[0].lower()}=--"
            for i in range(len(CHANNELS))
        )
        print(f"[{remaining:4.1f}s left]  {status}")

print("Done! Saving plot...")

# ── Plot ─────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(12, 5))
for i in range(len(CHANNELS)):
    ax.plot(times[i], distances[i], label=LABELS[i], color=COLORS[i])

ax.set_title(f"VL53L4CD — Distance per Sensor ({RECORD_SECONDS}s)")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Distance (cm)")
ax.legend(loc="upper left")
ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.savefig("tof_plot.png")
print("Saved to tof_plot.png")
