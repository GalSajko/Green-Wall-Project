import serial
import time

def initSerial():
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout = 1)
    ser.reset_input_buffer()
    return ser 

def openGripper(ser):
    ser.write(b"o\n")

def closeGripper(ser):
    ser.write(b"c\n")

if __name__ == '__main__':
    ser = initSerial()

    while True:
        command = input("Enter command (o or c): ")
        if command == 'o':
            print("Open gripper")
            openGripper(ser)
        elif command == 'c':
            print("Close gripper")
            closeGripper(ser)
        time.sleep(1)
        
        