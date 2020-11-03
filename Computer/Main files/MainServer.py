# import the base library

import socket
import numpy as np
import cv2
import threading

from keras.models import load_model
import tensorflow  

# import the own library
from Distance import *
from Detection import *
from RC_driver import *



# initail default graph for tensorflow beacause it causes some problems without initial tthis
global graph
graph = tensorflow.get_default_graph()

# define the default distance from any object
ultra_value = 40

# load drive model
nn_model = load_model("saved_model/model_drive.xml")

# this class for init connect between ultrasonic(Raspberry) and computer 
class UltraSensorServer(object):
    def __init__(self, host, port):

        self.server_socket = socket.socket(socket.AF_INET ,socket.SOCK_STREAM)
        
        self.server_socket.bind((host, port))
        self.server_socket.listen(0)
        self.connection, self.client_address = self.server_socket.accept()
        print("Connection from: ", self.client_address)

        self.data=" "
       
    # recieve data from ultrasonic
    def streaming(self):
        
        global ultra_value 

        try:

            while True:
                
                try:

                    self.data = self.connection.recv(1024)
                    ultra_value = self.data.decode("utf-8")
                    ultra_value = round(float(ultra_value[0:6]))
                    # print("ultra value :",ultra_value)
                
                except:
                    pass
                
                
        finally:
            self.connection.close()
            self.server_socket.close()


# this class for init connect between Camera(Raspberry) and computer  
class VideoServer(object):


    def __init__(self,host,port_video,port_serial):

        self.server_socket = socket.socket()

        self.server_socket.bind(("", port_video))

        self.server_socket.listen(1)
        self.connection, self.client_address = self.server_socket.accept()
        self.connection = self.connection.makefile('rb')
        self.host_name = socket.gethostname()
        self.host_ip = socket.gethostbyname(self.host_name)


        self.send_inst = True
        


        # take object from ObjectDetection Clqass to detect the traffic,Stop,Limit sings
        self.obj_detection = ObjectDetection()

        # inital the CascadeClassifier by using method from OpenCV
        self.stop_cascade = cv2.CascadeClassifier("saved_model/stop_sign_cascade.xml")
        self.light_cascade = cv2.CascadeClassifier("saved_model/traffic_sign_cascade.xml")
        self.speed_limit_cascade = cv2.CascadeClassifier("saved_model/limit_sign_cascade.xml")

        # initial objcet from RCControl class for driving the car
        self.rc_car = RCControl(port_serial)
        
        
        # h1: stop sign, measured manually
        # h2: traffic light, measured manually
        # h3: limit sign, measured manually
        self.h1 = 5.5  # cm
        self.h2 = 5.5
        self.h3 = 5.5

        # initial objcet from DistanceToCamera to measure the distance from stop,traffic,limit signs
        self.distance_to_camera = DistanceToCamera()

        # default distance between sings and camera
        self.distance_stop_sign = 0
        self.distance_light_sign = 0
        self.distance_limit_sign = 0
                        
        
        # start time when stop at the stop sign
        self.stop_start = 0  
        self.stop_finish = 0
        self.stop_time = 0
        self.drive_time_after_stop = 0

        # start time when limit sign is shown
        self.limit_sign_start = 0

        # limit sign finish after 3 second
        # this variable is a counter for count time to 3 second
        self.limit_sign_finish = 0
        

    # take the region of interest from the camera 
    def region_of_interest(self,image):
        #detect the rigion of interest

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


    # process an image 
    def process_img(self,img):
        
        # reshape the image
        img = img.reshape(100,320,1)

        # minus max pixel value form other pixels
        # the output will make the black region to be white and the wight region to be black
        img = np.max(img)-img

        # normalize the image by divided it on the max pixel value
        img = img/np.max(img)

        # img[img<0.1] = 1
        # cv2.imshow("gra",img)

        # do threshhold on the image to convert it to black and wight
        ret, img = cv2.threshold(img, 0.55, 1, cv2.THRESH_BINARY)

        # take region of interest from image
        img = self.region_of_interest(img)

        return img


    # this method do the streaming to camera
    def stream(self):

        # make sensor value global to access it form all the programe
        global ultra_value
        
        # default value for enable flag for stop sign 
        stop_sign_active = True

        stop_flag = False

        stream_bytes = b' '

        try:
            print("Host: ", self.host_name + ' ' + self.host_ip)
            print("Connection from: ", self.client_address)
            print("Streaming...")
            print("Press 'q' to exit")

            # need bytes here
            stream_bytes = b' '

            
            while self.send_inst:

                try:

                    stream_bytes += self.connection.read(1024)
                    first = stream_bytes.find(b'\xff\xd8')
                    last = stream_bytes.find(b'\xff\xd9')
                    if first != -1 and last != -1:
                        jpg = stream_bytes[first:last + 2]
                        stream_bytes = stream_bytes[last + 2:]

                        # receive gray image from stream
                        gray_image = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_GRAYSCALE)
                        # receive color image from stream
                        image = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                        
                        
                        
                        
                        # shape of the gray image
                        height, width = gray_image.shape

                        # take the half of gray image
                        roi = gray_image[int(height/2):height-20, :]

                    

                        # object detection
                        # to detect the signs 
                        v_param1 = self.obj_detection.detect(self.stop_cascade, gray_image, image)
                        v_param2 = self.obj_detection.detect(self.light_cascade, gray_image, image)
                        v_param3 = self.obj_detection.detect(self.speed_limit_cascade, gray_image, image)
                        

                        # distance measurement
                        if (v_param1 > 0 or v_param2 > 0 or v_param3 > 0):
                            d1 = self.distance_to_camera.calculate(v_param1, self.h1, 300, image)
                            d2 = self.distance_to_camera.calculate(v_param2, self.h2, 100, image)
                            d3 = self.distance_to_camera.calculate(v_param3, self.h3, 100, image)
                            
                            self.distance_stop_sign = d1
                            self.distance_light_sign = d2
                            self.distance_limit_sign = d3

                        
                    

                        # process roi of image
                        proc_img = self.process_img(roi)
                        cv2.imshow("proc",proc_img)
                        cv2.imshow("image",image)

                        # reshape proc image to convenient the model 
                        # because th model take the image shape(1,100, 320, 1)
                        proc_img = proc_img.reshape(1,100, 320, 1)

                        
                        # neural network makes prediction
                        # by use the default graph to make prediction road for the image 
                        with graph.as_default():
                            prediction = int(nn_model.predict_classes(proc_img))


                        # collision avoidance
                        if ultra_value  < 30:
                            # print("\nStop, obstacle in front")
                            self.rc_car.stop()
                            #ultra_value = None

                        # stop condition 
                        elif 0 < self.distance_stop_sign < 45 and stop_sign_active:
                            print("Stop sign ahead: ",self.distance_stop_sign)
                            self.rc_car.stop()
                        
                            # stop for 5 seconds
                            if stop_flag is False:
                                self.stop_start = cv2.getTickCount()
                                stop_flag = True
                            self.stop_finish = cv2.getTickCount()

                            self.stop_time = (self.stop_finish - self.stop_start) / cv2.getTickFrequency()
                            print("Stop time: %.2fs" % self.stop_time)

                            # 5 seconds later, continue driving
                            if self.stop_time > 5:
                                stop_flag = False
                                stop_sign_active = False
                                self.obj_detection.red_light = False
                                print("Waited for 5 seconds")
                                
                            
                            

                        # if distance from traffic,limit signs lower than 30 
                        elif (0 < self.distance_light_sign < 30 or 0 < self.distance_limit_sign < 30):
                            # print("Traffic light ahead")

                            if self.obj_detection.red_light:
                                print("Red light")
                                self.rc_car.stop()
                                
                                # when red light is ON we alwyes enter to this if
                                # and for make red light to be false !!!!
                                # should the greenlight to be ON
                                # when the green light is ON make the red light to be flase
                                if self.obj_detection.green_light:
                                    print("Green light")
                                    self.obj_detection.red_light = False
                                    
                            
                            if self.obj_detection.green_light:
                                    print("Green light")
                                    self.obj_detection.red_light = False
                                    
                            # if determine the limit sign 
                            # start_time variable will be start
                            elif self.obj_detection.limit_sign:
                                print("limit")
                                self.limit_sign_start = cv2.getTickCount()
                                
                                
                            # make the distance to be 30 
                            self.distance_light_sign = 30
                            self.distance_limit_sign = 30
                            
                            # return green light to be false
                            self.obj_detection.green_light = False
                        

                        # if there isn't any red light fornt the car
                        elif(self.obj_detection.red_light is False):

                            # if limit_sign is shown front the car 
                            # the car will decrease its speed for 3 second
                            # and after that will return the main speed
                            if(self.obj_detection.limit_sign is True):
                                # print(prediction)

                                # make prediction by using steer_low_speed() method
                                self.rc_car.steer_low_speed(prediction)

                                # this counter always increase because the condition is true
                                self.limit_sign_finish = cv2.getTickCount()

                    
                                self.stop_start = cv2.getTickCount()
                                self.distance_stop_sign = 55

                                # calculate the time between start_time and counter 
                                time_drive = (self.limit_sign_finish - self.limit_sign_start) / cv2.getTickFrequency()      
                                
                                # if time_drive is bigger than 3 second
                                if(time_drive > 3):
                                    self.obj_detection.limit_sign = False

                                if stop_sign_active is False:
                                    self.drive_time_after_stop = (self.stop_start - self.stop_finish) / cv2.getTickFrequency()
                                    if self.drive_time_after_stop > 2:
                                        stop_sign_active = True
                                        self.obj_detection.limit_sign = False
                                

                            # if there isn't any limit sign front the car
                            elif(self.obj_detection.limit_sign is False):
                                # print(prediction)

                                # make prediction by using steer() method 
                                self.rc_car.steer(prediction)
                                self.stop_start = cv2.getTickCount()
                                self.distance_stop_sign = 55
        
                                
                                if stop_sign_active is False:
                                    self.drive_time_after_stop = (self.stop_start - self.stop_finish) / cv2.getTickFrequency()                    
                                    if self.drive_time_after_stop > 5:
                                        stop_sign_active = True
                            


                        if cv2.waitKey(1) & 0xFF ==ord('q'):
                            print("car stopped")
                            self.rc_car.stop()
                            break
                    
                except:
                    pass

                     

        finally:
            self.connection.close()
            self.server_socket.close()
            self.rc_car.stop()



if __name__=="__main__":

    video = VideoServer("",8000 ,"COM8")

    ultra = UltraSensorServer("",8001)
 
    
    #initail threads for synchronize the conncetions between the camera and ultraSonic 
    video_thread = threading.Thread(target= video.stream , args=())
    ultra_thread = threading.Thread(target=ultra.streaming , args=())
    
    # start the threads
    video_thread.start()
    ultra_thread.start()
    
