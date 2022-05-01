import dynamixel
from dynamixel.model.xm430_w210_t_r import XM430_W210_T_R
import dynamixel.channel
import time
import numpy as np
from simple_pid import PID
import serial

def clamp(n, smallest, largest):
    return max(smallest, min(n, largest))

def init():
    channel = dynamixel.channel.Channel(57600, device='/dev/ttyACM0')
    servos = [
        XM430_W210_T_R(channel, 1),
        XM430_W210_T_R(channel, 2),
        XM430_W210_T_R(channel, 3)
        ]
    return channel, servos

def get_rot_speed(direction):
    r = 4.8/2 # (cm) radius of wheel
    R = 16.5  # (cm) distance from center to wheels
    transform = 1/r * np.array([[0, 1, R], [-np.sqrt(3)/2, -1/2, R], [np.sqrt(3)/2, -1/2, R]])
    return 120 * np.pi * (transform @ direction) / 0.229 # convertion factor between controll value
                                                         # and rpm is 0.229

def apply_speed(servos, speeds):
    for i, s in enumerate(servos):
        s.goal_velocity.write(int(speeds[i]))


def run(channel, servos):
    for s in servos:
        s.torque_enable.write(0)
        print(s.model_number.read(), s.id.read())
        s.velocity_limit.write(1023)
        s.operating_mode.write(1)
        s.bus_watchdog.write(0) # Clear old watchdog error
        s.bus_watchdog.write(100) # 2 second timeout
        s.torque_enable.write(1)
        s.profile_acceleration.write(0)

    serialport = '/dev/ttyACM1'
    baudrate = 115200

    pid = PID(400, 0, 0, setpoint=0, sample_time=0.1)
    ser = serial.Serial(serialport, baudrate)

    servs = np.array([1, 0,  -1])
    i = 0
    zero_offset = None

    try:
        while 1:
            raw = ser.readline().rstrip()
            data = float(raw)

            # Stop if it falls over
            if abs(data) > 30:
                apply_speed(servos, [0,0,0])
                break

            # First iteration will get setpoint
            if zero_offset == None:
                zero_offset = data
                print(f"Pole is offset by {zero_offset}")
                i += 1
                continue

            data = data - zero_offset
            pos_output = pid(data)
            print(f"Input: {data}, PID output: {pos_output}")
            i += 1
            if i % 1 == 0:
                apply_speed(servos, clamp(pos_output, -1023, 1023) * servs )
    except KeyboardInterrupt:
        apply_speed(servos, [0,0,0])

if __name__ == '__main__':
    run(*init())

