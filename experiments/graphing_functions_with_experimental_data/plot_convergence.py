import random
import math
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
import sys
import csv
from textwrap import wrap
from mpl_toolkits.axes_grid1 import make_axes_locatable
sys.path.append("utils")
import rosbag_utils
import figure_utils
import orienteering_utils

figure_utils.configure_latex_fonts()


log_file = "results/results.log"
SAVE_TO_FIGURE = "convergence1.pdf"

SHOW_FIGURE = True

#mpl.rc('font', size=15)
#fig1 = plt.figure(figsize = (6,5))

fig,ax = plt.subplots()

ax.set_title("\n".join(wrap("Convergence of relative translation to optimum.", 50)), fontsize=19)
ax.set_xlabel('Iteration',fontsize=14)
ax.set_ylabel('Translation error in number of cells', fontsize=14, color="darkblue")
plt.axis(aspect='equal')

filename = '/home/maderja1/workspace/src/occupancy_grid_merger_package/experiments/graphing_functions_with_experimental_data/data/maze_zero_convergence/attempt1.csv'
with open(filename, 'rb') as csvfile:
    csvreader = csv.reader(csvfile)
    next(csvreader)
    x = []
    y = []
    yaw = []
    fitVal = []
    iter = []
    zero = []
    i = 1
    for row in csvreader:
        iter.append(int(i))
        zero.append(int(0))
        i = i + 1 
        x.append(float(row[0]))
        y.append(float(row[1]))
        yaw.append(float(row[2]) * 180/math.pi)
        fitVal.append(int(row[3]))
ax.plot(iter, x, color="blue", label="x") 
ax.plot(iter, y, color="darkblue", label="y") 
#ax.plot(iter, yaw, color="darkblue") 
ax.plot(iter, zero, color="black") 

ax2 = ax.twinx()
ax2.plot(iter, fitVal, label='fitVal', color="red") 
ax2.set_ylabel("fitness function value", fontsize=14, color="red");
#plt.plot(iter, fit, color='blue') 
#ax.legend(framealpha=1, loc='lower center')
#ax.axis('off')
#fig.subplots_adjust(left=0.05, right=0.95, top=0.90, bottom=0.05)

plt.savefig(SAVE_TO_FIGURE)
if SHOW_FIGURE:
    plt.show()  
