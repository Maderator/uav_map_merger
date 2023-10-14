import random
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
SAVE_TO_FIGURE = "dependence.pdf"

SHOW_FIGURE = True

#mpl.rc('font', size=15)
#fig1 = plt.figure(figsize = (6,5))

fig,ax = plt.subplots()

ax.set_title("\n".join(wrap("Dependence of convergence and epochs duration on number of iterations.", 50)), fontsize=19)
ax.set_xlabel('Iteration',fontsize=14)
ax.set_ylabel('Fitness value', fontsize=14, color="blue")
plt.axis(aspect='equal')

filename = '/home/maderja1/workspace/src/occupancy_grid_merger_package/experiments/graphing_functions_with_experimental_data/data/dependence.csv'
with open(filename, 'rb') as csvfile:
    csvreader = csv.reader(csvfile)
    next(csvreader)
    iter = []
    fit = []
    cumTime = []
    for row in csvreader:
        iter.append(int(row[0]))
        fit.append(int(row[1]))
        cumTime.append(float(row[2]))
ax.plot(iter, fit, label='fitness value', color="blue", marker=".") 

ax2 = ax.twinx()
ax2.plot(iter, cumTime, label='cumulative time', color="red", marker=".") 
ax2.set_ylabel("cumulative time in seconds", fontsize=14, color="red");
#plt.plot(iter, fit, color='blue') 
#plt.legend(framealpha=1, loc='lower center')
#ax.axis('off')
#fig.subplots_adjust(left=0.05, right=0.95, top=0.90, bottom=0.05)

plt.savefig(SAVE_TO_FIGURE)
if SHOW_FIGURE:
    plt.show()  
