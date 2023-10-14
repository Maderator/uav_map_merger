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
SAVE_TO_FIGURE = "accept_maze_center.pdf"

SHOW_FIGURE = True

mpl.rc('font', size=15)
fig1 = plt.figure(figsize = (6,5))

plt.title("\n".join(wrap("Acceptance index of UAVs starting from identical spot.", 80)))
plt.xlabel('Iteration')
plt.ylabel('Acceptance index value')
plt.axis(aspect='equal')

for i in range(1,8):
    filename = '/home/maderja1/workspace/src/occupancy_grid_merger_package/experiments/fitness_val_plot/data/maze_zero_100/merging{}.csv'.format(i)
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
    plt.plot(iter, fit, color='red', label='$\pm41$ meters') 
for i in range(1,8):
    filename = '/home/maderja1/workspace/src/occupancy_grid_merger_package/experiments/fitness_val_plot/data/maze_zero_30/merging{}.csv'.format(i)
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
    plt.plot(iter, fit, color='blue', label='$\pm12$ meters') 
for i in range(1,8):
    filename = '/home/maderja1/workspace/src/occupancy_grid_merger_package/experiments/fitness_val_plot/data/maze_zero_10/merging{}.csv'.format(i)
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
    plt.plot(iter, fit, color='green', label='$\pm4$ meters') 
plt.legend(framealpha=1, loc='lower right')

#ax.axis('off')

plt.savefig(SAVE_TO_FIGURE)
if SHOW_FIGURE:
    plt.show()  
