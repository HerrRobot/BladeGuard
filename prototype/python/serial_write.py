import serial
import time

arduino = serial.Serial(port='/dev/ttyACM0', baudrate=115200, timeout=.1)

time.sleep(2) # INCREASE THIS IF SOMETHING GOES WRONGG

def write_read(x):
    x = x + "\n"
    arduino.write(bytes(x, 'ascii'))
    # time.sleep(0.5)
    # data = arduino.readline()
    # return   data


while True:
    num = input("Enter a number: ")
    if(num == '0'):
        break

    data = write_read(num)
    # print(data)


arduino.close()