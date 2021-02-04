from gpiozero import LED
from time import sleep
from rplidar import RPLidar
import numpy as np

PORT_NAME = '/dev/ttyUSB0'
DMAX = 1500 # range
IMIN = 0 
IMAX = 50
left = LED(17) #gpio
right = LED(27) #gpio

#for GPS
import serial
# option 1, using a lib
ser = serial.Serial(
    port='/dev/ttyS0',
    baudrate = 9600,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=0
)
# option 2, manual - try this first
#port = "/dev/ttyAMA0"
#ser = serial.Serial(port, baudrate = 9600, timeout = 0.5)
while(True):
    data = ser.readline()
    print(data)
#for buzzer
"""
import RPi.GPIO as GPIO
buzzpin = 22
GPIO.setmode(GPIO.BCM)
GPIO.setup(buzzpin,GPIO.OUT
for x in range(3):
    GPIO.output(buzzpin,GPIO.HIGH)
    sleep(0.5)
    GPIO.output(buzzpin,GPIO.LOW)
    sleep(0.5)
"""