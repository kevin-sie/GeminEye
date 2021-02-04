import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import time
from rplidar import RPLidar

lidar = RPLidar('/dev/ttyUSB0')

# info = lidar.get_info()
# print(info)

health = lidar.get_health()
print(health)

fig = plt.figure()
ax = fig.add_subplot(1,1,1)
xs = [];
ys = [];

def animate(i,xs,ys):

    for z, scan in enumerate(lidar.iter_scans()):
        # print('%d: Got %d measurments' % (i, len(scan)))
        #print(scan)
        #coords = []
        #fig = plt.figure()
        for j in scan:
            print("Data:", j[1], j[2])
            angle = math.radians(j[1])
            x = j[2] * np.cos(angle)
            y = j[2] * np.sin(angle)
            xs.append(x)
            ys.append(y)
        
        #points = np.array(coords)
        print("WE HERE")
        #xs = xs[-20:]
        #ys = ys[-20:]
        ax.clear()
        #ax.scatter(points[:,0], points[:, 1]) # was plot.scatter
        
        ax.scatter(xs, ys)
        #plt.draw()
        #if i == 0:
        #   plt.show()
        #else:
        #    plt.draw()
       
        #plt.close('all')
        
        if z > 2:
           print("BROKEN", z)
           break


ani = animation.FuncAnimation(fig,animate,fargs=(xs,ys),interval=1000)
plt.show()

lidar.stop()
lidar.stop_motor()
lidar.disconnect()

