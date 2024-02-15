import signal
from xbox360controller import Xbox360Controller
import serial

def on_axis_moved(axis):
    print(axis)
    if "Raw" in str(axis):
        print(f"Axis {axis.name} moved to {axis.value}")
    else:
        print(f"Axis {axis.name} moved to {axis.x} {axis.y}")    

try:
    with Xbox360Controller(0, axis_threshold=0.2) as controller:
        controller.info()

        print(controller.axis_l._value_x)
        print(controller.axis_l._value_y)

        ser = serial.Serial('/dev/cu.usbserial-0001')  # open serial port
        ser.baudrate = 115200
        command = b"M50"        # Command to send
        ser.write(command)      # write to ESP32

        ser.close()             # close por

        # Left and right axis move event
        # controller.axis_l.when_moved = on_axis_moved
        # controller.axis_r.when_moved = on_axis_moved
        
        signal.pause()
except KeyboardInterrupt:
    pass