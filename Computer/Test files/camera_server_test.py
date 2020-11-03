
import numpy as np
import cv2
import socket
import time

import matplotlib.pyplot as plt

class init_server():

    
    def __init__(self, host, port):

        self.server_socket = socket.socket()
        print("the server start")

        self.server_socket.bind(("", port))
        print("the host bind")

        self.server_socket.listen(1)
        self.connection, self.client_address = self.server_socket.accept()
        self.connection = self.connection.makefile('rb')
        self.host_name = socket.gethostname()
        self.host_ip = socket.gethostbyname(self.host_name)

        #define the variable form canny_Track class
        self.canny_tracking = canny_Track()

        #passing canny_tracking to the method
        self.streaming(self.canny_tracking)

       

    def streaming(self,canny_tracking):

        try:
            print("Host: ", self.host_name + ' ' + self.host_ip)
            print("Connection from: ", self.client_address)
            print("Streaming...")
            print("Press 'q' to exit")

            # need bytes here
            stream_bytes = b' '
            while True:
                stream_bytes += self.connection.read(1024)
                first = stream_bytes.find(b'\xff\xd8')
                last = stream_bytes.find(b'\xff\xd9')
                if first != -1 and last != -1:
                    jpg = stream_bytes[first:last + 2]
                    stream_bytes = stream_bytes[last + 2:]
                    image = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
                    
                    
                    cv2.imshow('image',  image2)
                    #plt.imshow(image)
                    #plt.show()
                    if cv2.waitKey(1)==ord('q'):
                        break
                
        finally:
            self.connection.close()
            self.server_socket.close()


if __name__ == '__main__':
    # host, port
    h, p = "", 8000
    camera_server = init_server(h, p)
    camera_server.streaming()
    
