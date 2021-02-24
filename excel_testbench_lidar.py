'''Animates distances and measurment quality'''
from rplidar import RPLidar
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import serial
import math
import csv

# SET UP TO READ FROM CSV ON DESKTOP TO GIVE US SINGLE OUTPUT


# ser = serial.Serial()
# ser.port = 'COM4'
# ser.baudrate = 9600
PORT_NAME = 'COM4'
# PORT_NAME = '/dev/ttyUSB0'

DMAX = 5000 # was 5000, 5 meters
IMIN = 0
IMAX = 50

cartesian_arrL = []  # x,y format
cartesian_arrR = []  # x,y format

filename = "cartesian_arr.csv"
with open(filename, 'r') as csvfile:
    data = [tuple(line) for line in csv.reader(csvfile)]  # converts to array of tuples

def tuple_float(x):
    r = tuple((float(x[0]), float(x[1])))
    return r

def conversion(x):
    a = float(x[0])
    b = float(x[1])
    #rho = np.sqrt(a**2 + b**2)
    phi = np.arctan2(b,a)
    return(phi)

for x in data:
    if(conversion(x) <= 1.5708):
        cartesian_arrL.append(tuple_float(x))
    if(conversion(x) >= 4.71239):
        cartesian_arrR.append(tuple_float(x))

def updateline():
    line_threshold = 200  # how long can you deviate from the original point
    object_threshold = 15  # how many points to be an object, probably has to be higher // was 15
    object_wall = 40
    object_countL = 0
    object_countR = 0

    bufferL = 0
    bufferR = 0
    buffer_amt = 7

    conditionL = 1
    if len(cartesian_arrL) > 0:
        start_pointL = cartesian_arrL[0][1]  # x value, maybe fix cause what if the first point is bad, [item1][x or y]
        compare_minusL = start_pointL - line_threshold
        compare_plusL = start_pointL + line_threshold
        xL = 1

        while(conditionL and xL < len(cartesian_arrL)): # while the next x value is not +- 200 of the original
            #  somehow this index gets out of range here, when i do something too close to the lidar?
            if cartesian_arrL[xL][1] > compare_minusL and cartesian_arrL[xL][1] < compare_plusL:
                object_countL = object_countL + 1
            else:
                bufferL = bufferL + 0
            xL = xL + 1
            if bufferL > buffer_amt:
                conditionL = 0
            #  getting a lot of conflicts, the bottom string keeps printing no matter what, works meh but there is a lot of flickering
            #  zone feature might have to come in since i think it happens when it sees something else (i.e object 2)


        last_pointL = cartesian_arrL[object_countL + bufferL-1][0] #this only works if there is a data point before the last buffer point


        if object_countL >= object_threshold and object_countL < object_wall:
            print("LEFT !!! OBJECT DETECTED, LED ON !!!")
        elif object_countL >= object_wall:
            print("LEFT UUU WALL PROBABLY, LED CAN TURN OFF NOW UUU")
        else:
            print("\n")
            #print("LEFT ??? NO OBJECT DETECTED, LED OFF ???")
            #  this prints too often... it flickers

    ###########################################copy starts#####################################################
    conditionR = 1
    if len(cartesian_arrR) > 0:
        start_pointR = cartesian_arrR[0][1]  # x value, maybe fix cause what if the first point is bad
        xR = 1
        compare_minusR = start_pointR - line_threshold
        compare_plusR = start_pointR + line_threshold

        while(conditionR and xR < len(cartesian_arrR)): # while the next x value is not +- 200 of the original
            #  somehow this index gets out of range here, when i do something too close to the lidar?
            if cartesian_arrR[xR][1] > compare_minusR and cartesian_arrR[xR][1] < compare_plusR:
                object_countR = object_countR + 1
            else:
                bufferR = bufferR + 0
            xR = xR + 1
            if bufferR > buffer_amt:
                conditionR = 0
            #  getting a lot of conflicts, the bottom string keeps printing no matter what, works meh but there is a lot of flickering
            #  zone feature might have to come in since i think it happens when it sees something else (i.e object 2)


        last_pointR = cartesian_arrR[object_countR + bufferR-1][0]


        if object_countR >= object_threshold and object_countR < object_wall:
            print("RIGHT !!! OBJECT DETECTED, LED ON !!!")
        elif object_countR >= object_wall:
            print("RIGHT UUU WALL PROBABLY, LED CAN TURN OFF NOW UUU")
        else:
            print("\n")
            # print("RIGHT ??? NO OBJECT DETECTED, LED OFF ???")
            #  this prints too often... it flickers


    #  30 = 0.523599
    #  330 = 5.75959
    #  print left or right
    #  greater than 30, less than 90
    '''
    if(real_offsets[0][0] > 0.523599 and real_offsets[0][0] < 1.5708 and real_offsets[object_count + buffer -1][0] > 0.523599 and real_offsets[object_count+buffer-1][0] < 1.5708):
        print("LEFT SIDE LED")
    #  greater than 270, less than 330
    if(real_offsets[0][0] > 4.71239 and real_offsets[0][0] < 5.75959 and real_offsets[object_count + buffer -1][0] > 4.71239 and real_offsets[object_count+buffer-1][0] < 5.75959):
        print("RIGHT SIDE LED")
    '''

    return 0

updateline()