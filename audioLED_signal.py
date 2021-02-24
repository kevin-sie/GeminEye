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

# for audio
import RPi.GPIO as GPIO
buzzpin = 22
GPIO.setmode(GPIO.BCM) # gpio numbering
GPIO.setup(buzzpin,GPIO.OUT)

#for GPS
import serial
# option 1, using a lib
ser = serial.Serial(
    port='/dev/ttyS0',
    baudrate = 4800,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=0.5
)

#for buzzer
def beep():
    for x in range(1100): # has to be a lot of loops
        GPIO.output(buzzpin,GPIO.HIGH)
        sleep(0.00025) # mimic frequency, period of corresponding freq
        GPIO.output(buzzpin,GPIO.LOW)
        sleep(0.00025)

# option 2, manual - try this first
#port = "/dev/ttyAMA0"
#ser = serial.Serial(port, baudrate = 9600, timeout = 0.5)
while(True):
    strdata = str(ser.readline())
    if (strdata[2:8] == "$GPRMC"):
        splitData = strdata.split(",")
        speedMPH = float(splitData[7]) * 1.15
        print(str(speedMPH) + " MPH")
        
        #if (speedMPH > 0.0):
         #   beep()


