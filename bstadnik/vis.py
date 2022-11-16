import math
import pickle
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

from time import sleep

loop_count=25  # drawing loop iterations
beam_half_angle=7.5 # haf of sonar angular beam width

# 
# A function to calculate Cartesian coordinates to polar
#  result: a tuple (rho,phi)
#  rho - radius, phi - angle in degrees
def cart2pol(x, y):
    rho = np.sqrt(x**2 + y**2)
    phi = np.arctan2(y, x)
    return(rho, phi)

# A function to transform polar  coordinates to Cartesian
# input angle in degrees
# returns a tuple (x,y)
def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return(x, y)

# plotting data
def plotsonars(ax, sonarreads):
    pol=[cart2pol(x[0],x[1]) for x in sonarreads ]
    for item in pol:
        #print item[0],item[1]
        wedge = mpatches.Wedge([0,0], item[0], item[1]-beam_half_angle, item[1]+beam_half_angle, alpha=0.4, ec="black",fc="CornflowerBlue")
        ax.add_patch(wedge)

def plotarrows(ax,arrlist):
    y=[[0,0]+x for x in arrlist ]
    soa =np.array(y) 
    X,Y,U,V = zip(*soa)
    ax.quiver(X,Y,U,V,angles='xy',scale_units='xy',scale=1)


plt.figure()
ax = plt.gca()
ax.set_aspect('equal')
ax.set_xlim([-6,6])
ax.set_ylim([-6,6])
plt.ion()
plt.show()

with open("sonar_scans", "rb") as file:
    laser_scans = pickle.load(file)
    print(laser_scans)
    
for current_scan in laser_scans:
    skan = current_scan
    ax.cla()
    plotsonars(ax,skan)  
    # plotarrows(ax,skan)
    ax.set_xlim([-6,6])
    ax.set_ylim([-6,6])
    plt.draw()
    plt.pause(0.0001)
    sleep(0.2)