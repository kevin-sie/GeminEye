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
# ser = serial.Serial(
    # port='/dev/ttyS0',
    # baudrate = 9600,
    # parity=serial.PARITY_NONE,
    # stopbits=serial.STOPBITS_ONE,
    # bytesize=serial.EIGHTBITS,
    # timeout=1
# )
# option 2, manual - try this first
port = "/dev/serial0"
ser = serial.Serial(port, baudrate = 9600, timeout = 0.5)
data = ser.readline()

#for buzzer
import RPi.GPIO as GPIO
buzzpin = 18
GPIO.setmode(GPIO.BCM)
while(true)
    GPIO.output(buzzpin,GPIO.HIGH)
    sleep(0.5)
    GPIO.output(buzzpin,GPIO.LOW)
    sleep(0.5)