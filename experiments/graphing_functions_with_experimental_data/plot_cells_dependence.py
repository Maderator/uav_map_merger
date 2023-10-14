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
SAVE_TO_FIGURE = "cellsDependence.pdf"

SHOW_FIGURE = True

#mpl.rc('font', size=15)
#fig1 = plt.figure(figsize = (6,5))

fig,ax = plt.subplots()

ax.set_title("\n".join(wrap("Effect of number of occupied cells on iteration duration.", 50)), fontsize=19)
ax.set_xlabel('Number of occupied cells',fontsize=14)
ax.set_ylabel('Iteration duration in seconds', fontsize=14, color="blue")
plt.axis(aspect='equal')

filename = '/home/maderja1/workspace/src/occupancy_grid_merger_package/experiments/graphing_functions_with_experimental_data/data/cells_iter_dependence.csv'
with open(filename, 'rb') as csvfile:
    csvreader = csv.reader(csvfile)
    next(csvreader)
    cells = []
    iterTime = []
    for row in csvreader:
        cells.append(int(row[0]))
        iterTime.append(float(row[1]))
ax.scatter(cells, iterTime, color="blue", marker="o") 

#plt.plot(iter, fit, color='blue') 
#plt.legend(framealpha=1, loc='lower center')
#ax.axis('off')
#fig.subplots_adjust(left=0.05, right=0.95, top=0.90, bottom=0.05)

plt.savefig(SAVE_TO_FIGURE)
if SHOW_FIGURE:
    plt.show()  
