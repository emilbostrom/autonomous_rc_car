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
        
        ser = serial.Serial('/dev/ttyUSB0')  # open serial port
        ser.baudrate = 115200
        
        while True:
            steering_reference =controller.axis_l.x
            motor_reference = controller.axis_l.y
            motor_reference = motor_reference if motor_reference > 0.1 else 0
            
            steering_command = "S" + str(steering_reference)
            ser.write(steering_command.encode())      # write to ESP32
            print(steering_command)

            motor_command = "M" + str(motor_reference)
            ser.write(motor_command.encode())      # write to ESP32 
            print(motor_command)           

        
        ser.close()             # close por
        
except KeyboardInterrupt:
    pass