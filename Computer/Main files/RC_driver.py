
import cv2
import math 
import serial


class RCControl(object):

    def __init__(self, serial_port):
        self.serial_port = serial.Serial(serial_port, 115200)

    def steer(self, prediction):
        if prediction == 2:
            self.serial_port.write('f'.encode())
            print("Forward")
        elif prediction == 0:
            self.serial_port.write('l'.encode())
            print("Left")
        elif prediction == 1:
            self.serial_port.write('r'.encode())
            print("Right")
        else:
            self.stop()
    
    def steer_low_speed(self, prediction):
        if prediction == 2:
            self.serial_port.write('2'.encode())
            print("Forward")
        elif prediction == 0:
            self.serial_port.write('0'.encode())
            print("Left")
        elif prediction == 1:
            self.serial_port.write('1'.encode())
            print("Right")
        else:
            self.stop()

    def stop(self):
        self.serial_port.write('s'.encode())

