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

def readFeedback(ser):
    line = ser.readline().decode("utf-8").rstrip()
    return line

if __name__ == '__main__':
    ser = initSerial()

    while True:
        command = input("Enter command (o or c): ")
        if command == 'o':
            print("Open gripper")
            openGripper(ser)
            line = readFeedback(ser)
            while not line:
                line = readFeedback(ser)
            print(line)

        elif command == 'c':
            print("Close gripper")
            closeGripper(ser)
            line = readFeedback(ser)
            while not line:
                line = readFeedback(ser)
            print(line)
            
        time.sleep(1)
        
        