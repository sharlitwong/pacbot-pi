import copy_control as ctrl

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
    return readings# Create I2C bus

if __name__ == "__main__":

    i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
    bot_mov=ctrl.test_mov(i2c)
    bot_mov.send_motors(0,0,0,0)
    channels = [0,1,2,3]
    sensors = setup_sensors(i2c, channels) 
    try:
        while(True):
            command=input()
            bot_mov.take_command(command)
            pass
    except KeyboardInterrupt as e:
        bot_mov.send_motors(0,0,0,0)
        bot_mov.close()
        exit

