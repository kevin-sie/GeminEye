from gpiozero import LED
from time import sleep
'''Animates distances and measurment quality'''
'''Animates distances and measurment quality'''
from rplidar import RPLidar
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation

PORT_NAME = '/dev/ttyUSB0'
DMAX = 1500
IMIN = 0 
IMAX = 50
left = LED(17)
right = LED(27)

GLOBAL_ISAWSOMETHING_L = 0 #LED
GLOBAL_ISAWSOMETHING_R = 0 #LED

def update_line(num, iterator):
    global GLOBAL_ISAWSOMETHING_L
    global GLOBAL_ISAWSOMETHING_R
    scan = next(iterator)
    
    boundaryLeft = 5.2359
    boundaryRight = 1.0472
    iterator = 0
    offsets = np.array([(meas[0],np.radians(meas[1]), meas[2]) for meas in scan]) # if meas[1] < 225 and meas[1] > 315])
    min_size = 10
    
    leftOffsets = [] # stores angle and distance for the left side
    index = 0 
    buffer = 0
    prev = 0
    leftNothing = 0
    
    rightOffsets = [] # stores angle and distance for the right side
    rightIndex = 0
    rightBuffer = 0
    rightPrev = 0
    rightNothing = 0
    rightIterator = 0
    
    for i in offsets:
        if i[1] >= boundaryLeft and i[1] <= 6.28:  #object detection for left side
                
            leftOffsets.append([i[1],i[2]])
            #leftIntens.append(i[0])
            if len(leftOffsets) > 1:
                prev = index - 1
                #print(len(leftOffsets), index, prev)
                if leftOffsets[prev][1] > 0:
                    iterator = iterator+ 1
                else:
                    buffer = buffer + 1
                if iterator >= min_size:
                    GLOBAL_ISAWSOMETHING_L = 1
                if i[2] == 0 and buffer == 3:
                    iterator = 0
                    buffer = 0
                if GLOBAL_ISAWSOMETHING_L == 1:
                    left.on()
                if i[2] == 0 or i[2] > 1500:
                    leftNothing = leftNothing + 1
            index = index + 1
        
        if i[1] <= boundaryRight and i[1] > 0:  #object detection for right side
            
            rightOffsets.append([i[1],i[2]])
            #real_intens.append(i[0])
            if len(rightOffsets) > 0:
                #red.n()
                rightPrev = rightIndex - 1
                #if index < length:
                    #nexti = index + 1    
                #print(real_offsets[prev][1])
                if rightOffsets[rightPrev][1] > 0:# and real_offsets[nexti][1] <= 500: 
                    rightIterator = rightIterator+ 1
                else:
                    rightBuffer = rightBuffer + 1
                if rightIterator >= min_size:
                    GLOBAL_ISAWSOMETHING_R = 1
                if i[2] == 0 and rightBuffer == 3:
                    rightIterator = 0
                    rightBuffer = 0
                if GLOBAL_ISAWSOMETHING_R == 1:
                    right.on()
                if i[2] == 0 or i[2] > 1500:
                    rightNothing = rightNothing + 1
            rightIndex = rightIndex + 1
            
    if leftNothing > len(leftOffsets)*.9:
        left.off()
        GLOBAL_ISAWSOMETHING_L = 0
    if rightNothing > len(rightOffsets)*.9:
        right.off()
        GLOBAL_ISAWSOMETHING_R = 0
    
    # 225 3.92 315 5.49

    return 0

def run():
    lidar = RPLidar(PORT_NAME)
    iterator = lidar.iter_scans(400)
    while(1):
        update_line(50,iterator)
    lidar.stop()
    lidar.disconnect()

if __name__ == '__main__':
    run()

#red = LED(17)
#while(1):
 #   red.on()
  #  sleep(2)
   # red.off()
    #sleep(2)

    