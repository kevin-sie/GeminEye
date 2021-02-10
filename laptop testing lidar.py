'''Animates distances and measurment quality'''
from rplidar import RPLidar
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import serial
import math
import csv

# ser = serial.Serial()
# ser.port = 'COM4'
# ser.baudrate = 9600
PORT_NAME = 'COM4'
# PORT_NAME = '/dev/ttyUSB0'

DMAX = 2500 # was 5000, 5 meters
IMIN = 0
IMAX = 50

filename = "cartesian_arr.csv"

def update_line(num, iterator, line):
    scan = next(iterator)
    # print(np.radians(meas[1]) for meas in scan)
    # intens = np.array([meas[0] for meas in scan])

    offsets = np.array(
        [(meas[0], np.radians(meas[1]), meas[2]) for meas in scan])  # if meas[1] < 225 and meas[1] > 315])
    # real_offsets = np.array([])
    # print(offsets)
    real_offsets = []
    real_intens = []

    cartesian_arr = []  # x,y format
    degrees_arr = []



    for i in offsets:
        if i[2] < 2000:
            if i[1] >= 4.71239 or i[1] <= 1.5708: # 5.23 is 300, 1.04 is 60, 1.5708 is 90
                # print(i)
                # 30 = 0.523599
                # 330 = 5.75959
                real_offsets.append([i[1], i[2]])
                # degrees_arr.append((math.degrees(i[1]), i[2]))
                real_intens.append(i[0])

                cartesian_tuple = (i[2]*math.cos(i[1]), i[2]*math.sin(i[1]))  # (r*cos(degrees),r*sin(degrees))
                cartesian_arr.append(cartesian_tuple)  # add to cartesian array
                # print(cartesian_arr)
                # print(degrees_arr)


    # do something with cartesian array now

    line_threshold = 200  # how long can you deviate from the original point
    object_threshold = 15  # how many points to be an object, probably has to be higher
    object_wall = 40
    object_count = 0
    buffer = 0

    start_point = cartesian_arr[0][0]  # x value, maybe fix cause what if the first point is bad


    x = 1
    compare_minus = start_point - line_threshold
    compare_plus = start_point + line_threshold
    condition = 1
    while(condition and x < len(cartesian_arr)): # while the next x value is not +- 200 of the original
        #  somehow this index gets out of range here, when i do something too close to the lidar?
        if cartesian_arr[x][0] > compare_minus and cartesian_arr[x][0] < compare_plus:
            object_count = object_count + 1
        else:
            buffer = buffer + 1
        x = x + 1
        if buffer > 7:
            condition = 0
        #  getting a lot of conflicts, the bottom string keeps printing no matter what, works meh but there is a lot of flickering
        #  zone feature might have to come in since i think it happens when it sees something else (i.e object 2)


    last_point = cartesian_arr[object_count + buffer-1][0]


    if object_count > object_threshold and object_count < object_wall:
        print("!!! OBJECT DETECTED, LED ON !!!")
    elif object_count > object_wall:
        print("UUU WALL PROBABLY, LED CAN TURN OFF NOW UUU")
    else:
        print("??? NO OBJECT DETECTED, LED OFF ???")
        #  this prints too often... it flickers

    #  30 = 0.523599
    #  330 = 5.75959
    #  print left or right
    #  greater than 30, less than 90
    if(real_offsets[0][0] > 0.523599 and real_offsets[0][0] < 1.5708 and real_offsets[object_count + buffer -1][0] > 0.523599 and real_offsets[object_count+buffer-1][0] < 1.5708):
        print("LEFT SIDE LED")
    #  greater than 270, less than 330
    if(real_offsets[0][0] > 4.71239 and real_offsets[0][0] < 5.75959 and real_offsets[object_count + buffer -1][0] > 4.71239 and real_offsets[object_count+buffer-1][0] < 5.75959):
        print("RIGHT SIDE LED")


    with open(filename, 'w') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerows(cartesian_arr)

    offsets = np.array(real_offsets)
    intens = np.array(real_intens)
    line.set_array(intens)
    line.set_offsets(offsets)

    # 225 3.92 315 5.49

    return line,


def run():
    lidar = RPLidar(PORT_NAME)
    fig = plt.figure()
    ax = plt.subplot(111, projection='polar')
    line = ax.scatter([0, 0], [0, 0], s=5, c=[IMIN, IMAX],
                      cmap=plt.cm.Greys_r, lw=0)
    ax.set_rmax(DMAX)
    ax.grid(True)

    iterator = lidar.iter_scans()
    ani = animation.FuncAnimation(fig, update_line,
                                  fargs=(iterator, line), interval=50)
    plt.show()
    lidar.stop()
    lidar.disconnect()


if __name__ == '__main__':
    run()