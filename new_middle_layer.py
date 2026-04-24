import threading
import busio
import board
import time
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

import copy_control as ctrl
current_command = None
def input_thread():
    global current_command
    while True:
        # Get the full string (e.g., "kp 2.5" or "f")
        cmd = input("Enter command: ").strip()
        if cmd:
            current_command = cmd
        if cmd.lower() == 'quit':
            break

        # Inside your Main Loop:
        if current_command is not None:
            if current_command.lower() == 'quit':
                break
            
            # Process the command through your existing logic
            # bot_mov.take_command should handle the splitting of command_components
            bot_mov.take_command(current_command)
            current_command = None
if __name__ == "__main__":
    i2c = busio.I2C(board.SCL, board.SDA, frequency=400000)
    
    bot_mov = ctrl.test_mov(i2c)
    bot_mov.send_motors(0, 0, 0, 0)
    
    channels = [0, 1, 2, 3]
    sensors = setup_sensors(i2c, channels) 
    
    THRESHOLD = 2.0  # cm
    is_overridden = False

    cmd_thread = threading.Thread(target=input_thread, daemon=True)
    cmd_thread.start()
    try:
        while True:
            # 1. Get filtered sensor data
            distances = read_sensors(sensors)
            print(f"Distances: {distances}") 

            # Identify threats (including the 0.0 maze-safety case)
            active_threats = [i for i, d in enumerate(distances) 
                              if d is not None and (d < THRESHOLD or d == 0.0)]

            if active_threats:
                if not is_overridden:
                    print(f"!!! ALERT: Obstacle at sensor {active_threats[0]} !!!")
                    bot_mov._stop_move()
                    is_overridden = True

                # Reverse Logic
                threat = active_threats[0]
                # We use take_command or send_motors to back away
                if threat == 0: bot_mov.take_command("b")
                elif threat == 1: bot_mov.take_command("f")
                elif threat == 2: bot_mov.take_command("l")
                elif threat == 3: bot_mov.take_command("r")
                
                time.sleep(0.3) # Move away for a short burst
                bot_mov._stop_move() # STOP after backing off
                current_command = None 

            else:
                # 2. NO OBSTACLES ZONE
                if is_overridden:
                    # If we just finished a back-off, ensure we are stopped
                    bot_mov._stop_move()
                    is_overridden = False

                if current_command is not None:
                    if current_command.lower() == 'quit':
                        break
                    bot_mov.take_command(current_command)
                    current_command = None
                
                # OPTIONAL: If you want the bot to stop immediately when you 
                # aren't giving it a command, add an else: bot_mov._stop_move() here.

            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        bot_mov.close()
