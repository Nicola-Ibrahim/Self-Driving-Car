import cv2
from keras.models import load_model
import tensorflow  

# initail default graph for tensorflow beacause it causes some problems without initial tthis
global graph
graph = tensorflow.get_default_graph()


class ObjectDetection():

    def __init__(self):
        self.red_light = False
        self.green_light = False
        self.yellow_light = False
        self.limit_sign = False
        
       
        self.model = load_model("saved_model/model_signs.xml")

    # process the image 
    def process_img(self,roi):

        #resize the image to (32,32) for the traffic model 
        img = cv2.resize(roi, (32, 32))

        # normalize the image 
        img = img/255   

        return img

    def detect(self, cascade_classifier, gray_image, image):

        # y camera coordinate of the target point 'P'
        v = 0

        # minimum value to proceed traffic light state validation
        threshold = 90

        # detection 
        cascade_obj = cascade_classifier.detectMultiScale(gray_image,scaleFactor=1.1,minNeighbors=5,minSize=(30, 30))

        
        # draw a rectangle around the objects
        for (x_pos, y_pos, width, height) in cascade_obj:
            cv2.rectangle(image, (x_pos -5 , y_pos -5), (x_pos + width + 5, y_pos + height + 5), (255, 255, 255), 2)
            v = y_pos + height + 5
            # print(x_pos+5, y_pos+5, x_pos+width-5, y_pos+height-5, width, height)

            
            # take tegion of interest rom image 
            # this region is refer to the rectangle of the object detection
            roi = gray_image[y_pos -5:y_pos + height + 5 ,  x_pos -5:x_pos + width + 5]
            # mask = cv2.GaussianBlur(roi, (25, 25), 0)

            # process the image
            proc_img = self.process_img(roi)


            # cv2.imshow("proc_img",proc_img)

            # reshape the image for the signs model 
            # because it take the image shape(1,32,32,1)
            proc_img = proc_img.reshape(1,32,32,1)


            # neural network makes prediction
            with graph.as_default():
                prediction = int(self.model.predict_classes(proc_img))

            
            if(prediction==0):
                cv2.putText(image, 'Red', (x_pos + 5, y_pos - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                self.red_light = True

            elif(prediction==1):
                cv2.putText(image, 'green', (x_pos + 5, y_pos - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                self.green_light = True

            elif(prediction==2):
                cv2.putText(image, 'STOP', (x_pos + 5, y_pos - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
                    
                
            elif(prediction==3):
                cv2.putText(image, 'Limit 30', (x_pos + 5, y_pos - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 255, 200), 2)
                self.limit_sign = True

            elif(prediction==4):
                cv2.putText(image, ' ', (x_pos + 5, y_pos - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 255, 200), 2)

        return v
