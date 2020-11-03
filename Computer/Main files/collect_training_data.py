
import numpy as np
import cv2
import serial
import pygame
from pygame.locals import *
import socket
import time
import os

import matplotlib.pyplot as plt

class CollectTrainingData(object):
    
    def __init__(self, host, port, serial_port, input_size):
        
        # Create server 
        self.server_socket = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        self.server_socket.bind((host, port))
        self.server_socket.listen(0)

        # accept a single connection
        self.connection = self.server_socket.accept()[0].makefile('rb')

        # connect to a serial port
        self.ser = serial.Serial(serial_port, 115200)
        self.send_inst = True

        self.input_size = input_size

        # initail pygame for driving the car 
        pygame.init()
        pygame.display.set_mode((250, 250))


    # Detect the rigion of interest in image
    def region_of_interest(self,image):
    
        #take the hight and width of the image which mean y coordinate
        hight = image.shape[0]
        width = image.shape[1]

        #detect the parameters of polygon
        polys = np.array([[(0, 50), (0, hight),
             (width, hight), (width, 50) ,(250,25),(70,25)]])

        #make a black image and this size equal the oraginal image
        mask = np.zeros_like(image)

        #draw a polygon and fill inside it by a white color
        cv2.fillPoly(mask, polys, 1)

        #do an AND operator between the mask and the orginal image
        #the output will be only the line in the street
        image_AND = cv2.bitwise_and(image, mask)

        return image_AND


    # process the image 
    def process_img(self,img):
        
        # reshape the image
        img = img.reshape(100,320,1)

        # minus max pixel value form other pixels
        # the output will make the black region to be white and the wight region to be black
        img = np.max(img)-img

        # normalize the image by divided it on the max pixel value
        img = img/np.max(img)


        # do threshhold on the image to convert it to black and wight
        ret, img = cv2.threshold(img, 0.66, 1, cv2.THRESH_BINARY)

        # take region of interest from image
        img = self.region_of_interest(img)

        return img
        

    def collect(self):

        saved_frame = 0
        total_frame = 0

        # collect images for training
        print("Start collecting images...")
        print("Press 'q' or 'x' to finish...")
        start = cv2.getTickCount()

        X = np.empty((0, self.input_size))
        y = np.empty((0, 1))
        print(y)

        # stream video frames one by one
        try:
            stream_bytes = b' '
            frame = 1
            while self.send_inst:
                stream_bytes += self.connection.read(1024)
                first = stream_bytes.find(b'\xff\xd8')
                last = stream_bytes.find(b'\xff\xd9')

                if first != -1 and last != -1:
                    jpg = stream_bytes[first:last + 2]
                    stream_bytes = stream_bytes[last + 2:]
                    image_gray = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
                    # image_gray = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)

                    
                    

                    # select lower half of the image
                    height, width = image_gray.shape
                    roi = image_gray[int(height/2):height-20, :]
                    
                    cv2.imshow("img",roi)


                    roi = self.process_img(roi)
                    cv2.imshow('roi', roi)
                    
                    
                    # reshape the roi image into a vector
                    temp_array = roi.reshape(1, int(height/2)-20 * width).astype(np.float32)
                    
                    frame += 1
                    total_frame += 1

                    # get input from human driver
                    for event in pygame.event.get():
                        if event.type == KEYDOWN:
                            key_input = pygame.key.get_pressed()

                        
                            # simple orders
                            if key_input[pygame.K_UP]:
                                print("Forward")
                                saved_frame += 1
                                X = np.vstack((X, temp_array))
                                y = np.vstack((y, 2 )) #self.k[2]
                                self.ser.write(b'f')

                            elif key_input[pygame.K_DOWN]:
                                print("Reverse")
                                self.ser.write(b'b')

                            elif key_input[pygame.K_RIGHT]:
                                print("Right")
                                X = np.vstack((X, temp_array))
                                y = np.vstack((y, 1)) #self.k[1]
                                saved_frame += 1
                                self.ser.write(b'r')

                            elif key_input[pygame.K_LEFT]:
                                print("Left")
                                X = np.vstack((X, temp_array))
                                y = np.vstack((y, 0)) #self.k[0]
                                saved_frame += 1
                                self.ser.write(b'l')

                            elif key_input[pygame.K_x] or key_input[pygame.K_q]:
                                print("exit")
                                self.send_inst = False
                                self.ser.write(b's')
                                self.ser.close()
                                break

                        elif event.type == pygame.KEYUP:
                            self.ser.write(b's')

                    if cv2.waitKey(1) & 0xFF == ord('q'):
                        break

            # save data as a numpy file
            file_name = str(int(time.time()))
            directory = "training_data"
            if not os.path.exists(directory):
                os.makedirs(directory)
            try:
                np.savez(directory + '/' + file_name + '.npz', train=X, train_labels=y)
            except IOError as e:
                print(e)

            end = cv2.getTickCount()
            # calculate streaming duration
            print("Streaming duration: , %.2fs" % ((end - start) / cv2.getTickFrequency()))

            print(X.shape)
            print(y.shape)
            print("Total frame: ", total_frame)
            print("Saved frame: ", saved_frame)
            print("Dropped frame: ", total_frame - saved_frame)

            print(y)
            print(y.shape)


        finally:
            self.connection.close()
            self.server_socket.close()


if __name__ == '__main__':
    # host, port
    h, p = "", 8000

    # serial port
    serial_port = "COM8"

    # vector size, half of the image
    s = 100 * 320

    ctd = CollectTrainingData(h, p, serial_port, s)
    ctd.collect()
