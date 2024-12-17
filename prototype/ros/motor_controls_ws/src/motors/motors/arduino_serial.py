import serial

arduino = serial.Serial(port='/dev/arduino_mega', baudrate=115200, timeout=.1)