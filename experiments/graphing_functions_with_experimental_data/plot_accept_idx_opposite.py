import random
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
SAVE_TO_FIGURE = "accept_maze_opposite.pdf"

SHOW_FIGURE = True

mpl.rc('font', size=15)
fig1 = plt.figure(figsize = (7,6))

plt.title("\n".join(wrap("Acceptance index of UAVs starting from opposite sides of maze.", 80)))
plt.xlabel('Iteration')
plt.ylabel('Acceptance index value')
plt.axis(aspect='equal')

#for i in range(1,8):
#    filename = '/home/maderja1/workspace/src/occupancy_grid_merger_package/experiments/fitness_val_plot/data/maze_opposite_60/merging{}.csv'.format(i)
#    with open(filename, 'rb') as csvfile:
#        csvreader = csv.reader(csvfile)
#        next(csvreader)
#        iter = []
#        fit = []
#        for row in csvreader:
#            iter.append(int(row[0]))
#            fit.append(float(row[3]))
#        print(iter)
#    print(fit)
#    plt.plot(iter, fit, color='red', label='$\pm49$ meters') 
for i in range(1,8):
    filename = '/home/maderja1/workspace/src/occupancy_grid_merger_package/experiments/fitness_val_plot/data/maze_opposite_45/merging{}.csv'.format(i)
    with open(filename, 'rb') as csvfile:
        csvreader = csv.reader(csvfile)
        next(csvreader)
        iter = []
        fit = []
        for row in csvreader:
            iter.append(int(row[0]))
            fit.append(float(row[3]))
        print(iter)
    print(fit)
    plt.plot(iter, fit) 

#ax.axis('off')

plt.savefig(SAVE_TO_FIGURE)
if SHOW_FIGURE:
    plt.show()  
