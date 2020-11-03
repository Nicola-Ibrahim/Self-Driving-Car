
import socket
import time
import RPi.GPIO as GPIO


GPIO.setwarnings(False)

# create a socket and bind socket to the host
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client_socket.connect(('192.168.1.7', 8001))

def measure():
    """
    measure distance
    """
    GPIO.output(GPIO_TRIGGER, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)
    start = time.time()

    while GPIO.input(GPIO_ECHO)==0:
        start = time.time()

    while GPIO.input(GPIO_ECHO)==1:
        stop = time.time()

    elapsed = stop-start
    distance = (elapsed * 34300)/2

    return distance

# referring to the pins by GPIO numbers
GPIO.setmode(GPIO.BOARD)

# define pi GPIO
GPIO_TRIGGER = 36
GPIO_ECHO    = 38

# output pin: Trigger
GPIO.setup(GPIO_TRIGGER,GPIO.OUT)
# input pin: Echo
GPIO.setup(GPIO_ECHO,GPIO.IN)
# initialize trigger pin to low
GPIO.output(GPIO_TRIGGER, False)

try:
    while True:
        distance = measure()
        print ("Distance : %.1f cm" % distance)

        distance = str(distance)
        # send data to the host every 0.5 sec
        client_socket.send(str.encode(distance))
        time.sleep(0.5)
        
finally:
    client_socket.close()
    GPIO.cleanup()