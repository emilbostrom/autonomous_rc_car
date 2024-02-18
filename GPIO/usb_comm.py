import serial

ser = serial.Serial('/dev/cu.usbserial-0001')  # open serial port
ser.baudrate = 115200
command = b"M50"        # Command to send
ser.write(command)      # write to ESP32

ser.close()             # close por