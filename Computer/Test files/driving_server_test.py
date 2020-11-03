
import socket
import time
import pygame

import threading 




class RCcarStream(object):

    def __init__(self,host,port):

        self.server_socket = socket.socket(socket.AF_INET ,socket.SOCK_STREAM)
        #self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((host,port))
        self.server_socket.listen(0)

        self.connection , self.address = self.server_socket.accept()

        self.host_name = socket.gethostname()
        self.host_ip = socket.gethostbyname(self.host_name)
        
        self.send_state=True
        self.streaming_steer()



    def streaming_steer(self):
        pygame.init()
        pygame.display.set_mode((250,250))
        
        print("Host: ", self.host_name + ' ' + self.host_ip)
        print("Connection from: ", self.address)

        self.connection.sendall(str.encode("Start"))
        
        while self.send_state:
            for event in pygame.event.get():
                if event.type == pygame.KEYDOWN:
                    key_input = pygame.key.get_pressed()

                    # complex orders
                    if key_input[pygame.K_UP] and key_input[pygame.K_RIGHT]:
                        print("Forward Right")
                        self.connection.sendall(str.encode("Forward Right"))
                        

                    elif key_input[pygame.K_UP] and key_input[pygame.K_LEFT]:
                        print("Forward Left")
                        self.connection.sendall(str.encode("Forward Left"))
                        

                    elif key_input[pygame.K_DOWN] and key_input[pygame.K_RIGHT]:
                        print("Reverse Right")
                        self.connection.sendall(str.encode("Reverse Right"))
                        

                    elif key_input[pygame.K_DOWN] and key_input[pygame.K_LEFT]:
                        print("Reverse Left")
                        self.connection.sendall(str.encode("Reverse Left"))
                       

                    # simple orders
                    elif key_input[pygame.K_UP]:
                        print("Forward")
                        self.connection.sendall(str.encode("Forward"))

                    elif key_input[pygame.K_DOWN]:
                        print("Reverse")
                        self.connection.sendall(str.encode("Reverse"))

                    elif key_input[pygame.K_RIGHT]:
                        print("Right")
                        self.connection.sendall(str.encode("Right"))
                       

                    elif key_input[pygame.K_LEFT]:
                        print("Left")
                        self.connection.sendall(str.encode("Left"))
                        

                    # exit
                    elif  key_input[pygame.K_q]:
                        print("Exit")
                        self.connection.sendall(str.encode("Exit"))
                        self.send_inst = False
                        break


                    distance_data_receive = self.connection.recv(1024)
                    distance_data_receive = distance_data_receive.decode("utf-8")
                    

                    print(distance_data_receive)

                        
                    #print("""the distacne is :{0:.1f} 
                    #and the water level is :{1}""".format(distance_data_receive,water_data_receive))

                elif event.type == pygame.KEYUP:
                    self.connection.sendall(str.encode("Stop"))

                    

if __name__=="__main__":
    h, p = "", 8002

    RCcarStream(h,p)





