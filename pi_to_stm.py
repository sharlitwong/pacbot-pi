#Purpose: Reading from Time-of-Flight sensors & sending data over UART to stm32

import serial
import time
import adafruit_vl53l4cd
import board
import adafruit_tca9548a

def setup_sensors(i2c, channels, timing_budget=200, inter_measurement=0):
    """Initialize multiple VL53L4CD sensors on specific TCA9548A channels."""
    tca = adafruit_tca9548a.TCA9548A(i2c)
    sensors = []
    for ch in channels:
        sensor = adafruit_vl53l4cd.VL53L4CD(tca[ch])
        sensor.timing_budget = timing_budget
        sensor.inter_measurement = inter_measurement
        sensor.start_ranging()
        sensors.append(sensor)
        print(f"Sensor on channel {ch} initialized.")

    return sensors


def read_sensors(sensors):
    """Read distance (in cm) from each sensor and return list of readings."""
    readings = []
    for i, sensor in enumerate(sensors):
        while not sensor.data_ready:
            pass
        distance = sensor.distance
        sensor.clear_interrupt()
        readings.append(distance)
    return readings

# Configure serial port
ser = serial.Serial(
    port='/dev/ttyS0',
    baudrate=9600,
    timeout=1
)

# Create I2C bus
i2c = board.I2C()

# Initialize sensors on channels 0, 1, 2, 3
channels = [0,1,2,3]
sensors = setup_sensors(i2c, channels)

print("VL53L4CD sensors and UART initialized")
print("--------------------")

try:
    while True:
        # Read all sensors - returns array with 4 elements
        sensor_data = read_sensors(sensors)

        # Access individual sensors by index:
        front_sensor = sensor_data[0]   # Sensor 0 (channel 0)
        left_sensor = sensor_data[1]    # Sensor 1 (channel 1)
        back_sensor = sensor_data[2]    # Sensor 2 (channel 2)
        right_sensor = sensor_data[3]   # Sensor 3 (channel 3)

        # Print sensor readings
        print(f"Front: {front_sensor} cm")
        print(f"Left:  {left_sensor} cm")
        print(f"Back:  {back_sensor} cm")
        print(f"Right: {right_sensor} cm")
        print("--------------------")

        # Send front sensor distance to STM32 (in mm as 2 bytes)
        distance_mm = []
        for d in sensor_data:
            mm = int(d * 10)  # Convert cm to mm

            # Clamp to 16-bit range
            if mm > 65535:
                mm = 65535
            elif mm < 0:
                mm = 0

            distance_mm.append(mm)

        # Send as 8 bytes: MSB first, then LSB
        packet = bytearray()
        for mm in distance_mm:
            packet.append((mm >> 8) & 0xFF)
            packet.append(mm & 0xFF)

        ser.write(packet)
        print(f"Sent to STM32 (mm): ", distance_mm)
        print(packet)
        time.sleep(0.1)

except KeyboardInterrupt:
    ser.close()
    print("Serial port closed.")
