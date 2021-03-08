'''Animates distances and measurment quality'''
from gpiozero import LED
from rplidar import RPLidar
import numpy as np
import serial
import math
from time import sleep
# import csv
left = LED(5)
right = LED(6)
# for audio
# for audio
import RPi.GPIO as GPIO
from time import sleep
buzzpin = 13 # 22 og, 26 new, 13
GPIO.setmode(GPIO.BCM) # gpio numbering
GPIO.setup(buzzpin,GPIO.OUT)
# new
GPIO.output(buzzpin, GPIO.LOW)
pwm = GPIO.PWM(buzzpin, 1400) # Set Frequency to 1 KHz
pwm.start(0) # Set the starting Duty Cycle

#for buzzer
def beep():
    """
    for dc in range(0, 101, 1):
        pwm.ChangeDutyCycle(dc)
        sleep(0.001)
    for dc in range(100, -1, -1):
        pwm.ChangeDutyCycle(dc)
        sleep(0.001)
    """
    dc = 0
    pwm.ChangeDutyCycle(50)
    while dc < 700000: # 101
        dc = dc + 1
        #pwm.ChangeDutyCycle(dc)
        #time.sleep(0.001)
    pwm.ChangeDutyCycle(0)


#for GPS
# import serial
# option 1, using a lib
ser = serial.Serial(
    port='/dev/ttyAMA0', #ttyS0 old, ttyAMA1 new
    baudrate = 4800,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=0.5
)

def speedCheck():
    strdata = str(ser.readline())
    if (strdata[2:8] == "$GPRMC"):
        splitData = strdata.split(",")
        speedMPH = float(splitData[7]) * 1.15
        # print(str(speedMPH) + " MPH")
        
        if (speedMPH >= 0.0):
            return 1
            # update_line(50,iterator)
        else:
            return 0

PORT_NAME = '/dev/ttyUSB0'

DMAX = 5000 # was 5000, 5 meters
IMIN = 0
IMAX = 50

GLOBAL_WALL_L = 0
GLOBAL_WALL_R = 0
GLOBAL_OBJ_L = 0
GLOBAL_OBJ_R = 0

# filename = "cartesian_arr.csv"

def update_line(num, iterator):
    scan = next(iterator)
    
    global GLOBAL_WALL_L
    global GLOBAL_WALL_R
    global GLOBAL_OBJ_L
    global GLOBAL_OBJ_R
    global GLOBAL_CLOSE
    # print(np.radians(meas[1]) for meas in scan)
    # intens = np.array([meas[0] for meas in scan])

    offsets = np.array(
        [(meas[0], np.radians(meas[1]), meas[2]) for meas in scan])  # if meas[1] < 225 and meas[1] > 315])
    # real_offsets = np.array([])
    # print(offsets)
    real_offsets = []
    real_intens = []

    cartesian_arr = []
    cartesian_arrL = []  # x,y format
    cartesian_arrR = []  # x,y format
    degrees_arr = []

    close = 0
    OBJ_CLOSE = 0
    
    for i in offsets:
        if i[2] < 1000: # is something really close
            close = close + 1
            if close > 5:
                OBJ_CLOSE = 1
        if i[2] < 5000: # TESTING PLEASE REMOVE THIS, distance
            if i[1] >= 4.71239 or i[1] <= 1.5708: # 5.23 is 300, 1.04 is 60
                # print(i)
                # 30 = 0.523599
                # 330 = 5.75959
                real_offsets.append([i[1], i[2]])
                # degrees_arr.append((math.degrees(i[1]), i[2]))
                real_intens.append(i[0])

                cartesian_tuple = (i[2] * math.cos(i[1]), i[2] * math.sin(i[1]))  # (r*cos(degrees),r*sin(degrees))
                cartesian_arr.append(cartesian_tuple)  # add to cartesian array

                if i[1] <= 1.5708: # this is the left 90
                    cartesian_tupleL = (i[2]*math.cos(i[1]), i[2]*math.sin(i[1]))  # (r*cos(degrees),r*sin(degrees))
                    cartesian_arrL.append(cartesian_tupleL)  # add to cartesian array
                if i[1] >= 4.71239: # this is the right 270
                    cartesian_tupleR = (i[2]*math.cos(i[1]), i[2]*math.sin(i[1]))  # (r*cos(degrees),r*sin(degrees))
                    cartesian_arrR.append(cartesian_tupleR)  # add to cartesian array



    # do something with cartesian array now

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
            
            if (GLOBAL_OBJ_L < 7):
                GLOBAL_OBJ_L = GLOBAL_OBJ_L + 1
                
                if (GLOBAL_OBJ_L > 3):
                    left.on()
                    if(OBJ_CLOSE == 1 and speedCheck() == 1):
                        beep()
            if (GLOBAL_WALL_L > 0):    
                GLOBAL_WALL_L = GLOBAL_WALL_L - 1
        elif object_countL >= object_wall:
            print("LEFT UUU WALL PROBABLY, LED CAN TURN OFF NOW UUU")
            
            if (GLOBAL_WALL_L < 7):
                GLOBAL_WALL_L = GLOBAL_WALL_L + 1
                
                if (GLOBAL_WALL_L > 3):
                    left.off()
            if (GLOBAL_OBJ_L > 0):    
                GLOBAL_OBJ_L = GLOBAL_OBJ_L - 1
        else:
            print("\n")
            
            if (GLOBAL_WALL_L > 0):
                GLOBAL_WALL_L = GLOBAL_WALL_L - 1
                
            if (GLOBAL_OBJ_L > 0):
                GLOBAL_OBJ_L = GLOBAL_OBJ_L - 1
            
            if (GLOBAL_WALL_L < 4 and GLOBAL_OBJ_L < 4):
                left.off()

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
            
            if (GLOBAL_OBJ_L < 7):
                GLOBAL_OBJ_R = GLOBAL_OBJ_R + 1
                
                if (GLOBAL_OBJ_R > 3):
                    right.on()
                    if(OBJ_CLOSE == 1 and speedCheck() == 1):
                        beep()
            if (GLOBAL_WALL_R > 0):   
                GLOBAL_WALL_R = GLOBAL_WALL_R - 1
        elif object_countR >= object_wall:
            print("RIGHT UUU WALL PROBABLY, LED CAN TURN OFF NOW UUU")
            
            if (GLOBAL_WALL_R < 7):
                GLOBAL_WALL_R = GLOBAL_WALL_R + 1
                
                if (GLOBAL_WALL_R > 3):
                    right.off()
            if (GLOBAL_OBJ_R > 0):    
                GLOBAL_OBJ_R = GLOBAL_OBJ_R - 1
        else:
            print("\n")
            
            if (GLOBAL_WALL_R > 0):
                GLOBAL_WALL_R = GLOBAL_WALL_R - 1
                
            if (GLOBAL_OBJ_R > 0):
                GLOBAL_OBJ_R = GLOBAL_OBJ_R - 1
            
            if (GLOBAL_WALL_R < 4 and GLOBAL_OBJ_R < 4):
                right.off()


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
    offsets = np.array(real_offsets)
    intens = np.array(real_intens)
    # 225 3.92 315 5.49

    return 0


def run():
    lidar = RPLidar(PORT_NAME)
    iterator = lidar.iter_scans(1000) #400
    while(1):
        update_line(50,iterator)
                    
    lidar.stop()
    lidar.disconnect()


if __name__ == '__main__':
    run()