'''Assesses CSV LIDAR data for presence of car/motorcycle'''
import os
import matplotlib.pyplot as plt
import matplotlib.axes as axes
import numpy as np
import matplotlib.animation as animation
import serial
import math
import csv
import re

'''
Directory that stores LIDAR data
'''
path = r'C:\Users\Ian\Desktop\SDP\Testbench\Data\left1'


'''
Parameters for object detection algorithm
'''

DMAX = 5000 #units in mm
IMIN = 0
IMAX = 50

GLOBAL_WALL_L = 0
GLOBAL_WALL_R = 0
GLOBAL_OBJ_L = 0
GLOBAL_OBJ_R = 0


'''
Figure setup and animation helper functions
'''
fig = plt.figure()
ax = plt.axes(xlim = (0, 500), ylim = (-3500, 3500))
ax.grid(True)
line = ax.scatter([], [])

def tuple_float(x):
    r = tuple((float(x[0]), float(x[1])))
    return r

def sorted_alphanumeric(data):
    convert = lambda text: int(text) if text.isdigit() else text.lower()
    alphanum_key = lambda key: [ convert(c) for c in re.split('([0-9]+)', key) ]
    return sorted(data, key=alphanum_key)

def conversion(x):
    a = float(x[0])
    b = float(x[1])
    #rho = np.sqrt(a**2 + b**2)
    phi = np.arctan2(b,a)
    return(phi)

def update_line(num, arrL, arrR, line):
    print('update_line')
    #print(arrL)
    #print(arrR)

    global GLOBAL_WALL_L
    global GLOBAL_WALL_R
    global GLOBAL_OBJ_L
    global GLOBAL_OBJ_R
    global GLOBAL_CLOSE

    line_threshold = 200  # how long can you deviate from the original point
    object_threshold = 5  # how many points to be an object, probably has to be higher // was 15
    object_wall = 40
    object_countL = 0
    object_countR = 0

    bufferL = 0
    bufferR = 0
    buffer_amt = 7
    close = 0
    OBJ_CLOSE = 0
    conditionL = 1
    if len(arrL) > 0:
        start_pointL = arrL[0][1]  # x value, maybe fix cause what if the first point is bad, [item1][x or y]
        compare_minusL = start_pointL - line_threshold
        compare_plusL = start_pointL + line_threshold
        xL = 1

        while(conditionL and xL < len(arrL)): # while the next x value is not +- 200 of the original
            #  somehow this index gets out of range here, when i do something too close to the lidar?
            if arrL[xL][1] > compare_minusL and arrL[xL][1] < compare_plusL:
                object_countL = object_countL + 1
            else:
                bufferL = bufferL + 0
            xL = xL + 1
            if bufferL > buffer_amt:
                conditionL = 0


        if object_countL >= object_threshold and object_countL < object_wall:
            if (GLOBAL_OBJ_L < 7):
                GLOBAL_OBJ_L = GLOBAL_OBJ_L + 1

                if (GLOBAL_OBJ_L > 3):
                    print("LEFT !!! OBJECT DETECTED, LED ON !!!")
                    if (OBJ_CLOSE == 1):
                        print("BEEP")
            if (GLOBAL_WALL_L > 0):
                GLOBAL_WALL_L = GLOBAL_WALL_L - 1
        elif object_countL >= object_wall:

            if (GLOBAL_WALL_L < 7):
                GLOBAL_WALL_L = GLOBAL_WALL_L + 1

                if (GLOBAL_WALL_L > 3):
                    print("LEFT WALL PROBABLY, LED CAN TURN OFF NOW")
            if (GLOBAL_OBJ_L > 0):
                GLOBAL_OBJ_L = GLOBAL_OBJ_L - 1
        else:

            if (GLOBAL_WALL_L > 0):
                GLOBAL_WALL_L = GLOBAL_WALL_L - 1

            if (GLOBAL_OBJ_L > 0):
                GLOBAL_OBJ_L = GLOBAL_OBJ_L - 1

            if (GLOBAL_WALL_L < 4 and GLOBAL_OBJ_L < 4):
                print("\n")


    ###########################################copy starts#####################################################
    conditionR = 1
    if len(arrR) > 0:
        start_pointR = arrR[0][1]  # x value, maybe fix cause what if the first point is bad
        xR = 1
        compare_minusR = start_pointR - line_threshold
        compare_plusR = start_pointR + line_threshold

        while(conditionR and xR < len(arrR)): # while the next x value is not +- 200 of the original
            #  somehow this index gets out of range here, when i do something too close to the lidar?
            if arrR[xR][1] > compare_minusR and arrR[xR][1] < compare_plusR:
                object_countR = object_countR + 1
            else:
                bufferR = bufferR + 0
            xR = xR + 1
            if bufferR > buffer_amt:
                conditionR = 0


        last_pointR = arrR[object_countR + bufferR-1][0]

        if object_countR >= object_threshold and object_countR < object_wall:

            if (GLOBAL_OBJ_R < 7):
                GLOBAL_OBJ_R = GLOBAL_OBJ_R + 1

                if (GLOBAL_OBJ_R > 3):
                    print("RIGHT !!! OBJECT DETECTED, LED ON !!!")
                    if (OBJ_CLOSE == 1):
                        print("BEEP")
            if (GLOBAL_WALL_R > 0):
                GLOBAL_WALL_R = GLOBAL_WALL_R - 1
        elif object_countR >= object_wall:

            if (GLOBAL_WALL_R < 7):
                GLOBAL_WALL_R = GLOBAL_WALL_R + 1

                if (GLOBAL_WALL_R > 3):
                    print("RIGHT WALL, LED CAN TURN OFF NOW")
            if (GLOBAL_OBJ_R > 0):
                GLOBAL_OBJ_R = GLOBAL_OBJ_R - 1
        else:
            #print("\n")

            if (GLOBAL_WALL_R > 0):
                GLOBAL_WALL_R = GLOBAL_WALL_R - 1

            if (GLOBAL_OBJ_R > 0):
                GLOBAL_OBJ_R = GLOBAL_OBJ_R - 1

            if (GLOBAL_WALL_R < 4 and GLOBAL_OBJ_R < 4):
                print("\n")

    '''
    if(real_offsets[0][0] > 0.523599 and real_offsets[0][0] < 1.5708 and real_offsets[object_count + buffer -1][0] > 0.523599 and real_offsets[object_count+buffer-1][0] < 1.5708):
        print("LEFT SIDE LED")
    #  greater than 270, less than 330
    if(real_offsets[0][0] > 4.71239 and real_offsets[0][0] < 5.75959 and real_offsets[object_count + buffer -1][0] > 4.71239 and real_offsets[object_count+buffer-1][0] < 5.75959):
        print("RIGHT SIDE LED")
    '''
    line.set_array(arrL.extend(arrR))

    # For some reason this function calls itself again after returning 'line' and appears to keep looping back to the top
    #Commenting out the return line does not change this...
    return line


'''
Make a list of all the file names in the working directory. Create a variable that tracks which we have to read next
'''
fileNumber = 0 #Tracks which file the program will read from next
fileNames = []
for filename in sorted_alphanumeric(os.listdir(path)):
    fileNames.append(filename)
    #print(filename)

#print(fileNames)

'''
Reads a single CSV file from working directory
'''
def plotFn(frame):
    global fileNumber

    if len(fileNames) <= fileNumber: #This is here to protect for the case of you reading the last file and then the animation wanting to start over again. If the animation wants to do that then we need to reset the value of fileNumber
        fileNumber = 0

    print("Begin CSV file read of file number:" + str(fileNumber))
    currentFile = fileNames[fileNumber]
    with open(os.path.join(path, currentFile), 'r') as csvfile:  # open in read-only mode
        data = [tuple(line) for line in csv.reader(csvfile)]  # converts to array of tuples

        #I think we want a 2 x N array instead of a list of N tuples
        points = [] # 2 x N array that holds x values in position 0, y values in position 1
        '''
        Need to build an array to pass into line.set_offsets()
        '''

        for pts in filter(None, data): #Not sure what this filter thing does but I'm going to keep it and trust that whoever wrote this put it there for a good reason
            points.append(tuple_float(pts))  #Don't think this really makes sense but lets see
            print('Index 0: '+ str(pts[0]))
            print('Index 1: '+ str(pts[1]))
            print(points)

    #The example passes an array that is 20x2 into this...
    line.set_offsets(points)
    fileNumber+=1 #increment fileNumber so that next time anim calls this function we will read the next CSV file in the folder
    print("End CSV file read")

'''
#Stuff from Noah down here:
for filename in sorted_alphanumeric(os.listdir(path)):
    print("Begin CSV file read")
    with open(os.path.join(path, filename), 'r') as csvfile: # open in read-only mode
        data = [tuple(line) for line in csv.reader(csvfile)]  # converts to array of tuples

        cartesian_arrL = []  # x,y format, left side
        cartesian_arrR = []  # x,y format, right side
        cartesian_arrX = []  # x value array
        cartesian_arrY = []  # y value array
        cartesian_arr = []   # unsorted tuple array

        for x in filter(None,data):
                cartesian_arr.append(x)
                cartesian_arrX.append(x[0])
                cartesian_arrY.append(x[1])
                if(float(x[1]) >= 0):
                    cartesian_arrL.append(tuple_float(x))
                if(float(x[1]) <= 0):
                    cartesian_arrR.append(tuple_float(x))

    print("End CSV file read")
'''

'''
Animation of the data read from the CSV
'''
anim = animation.FuncAnimation(fig, plotFn, interval = 50)
plt.show()
