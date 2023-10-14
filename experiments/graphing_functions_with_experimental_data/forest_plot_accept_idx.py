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

#plt.rcParams.update({'font.size': 12})

log_file = "results/results.log"
SAVE_TO_FIGURE = "forest_accept.pdf"

SHOW_FIGURE = True

mpl.rc('font', size=15)
fig1 = plt.figure(figsize = (6,5))
for i in range(1,8):
    filename = '/home/maderja1/workspace/src/occupancy_grid_merger_package/experiments/fitness_val_plot/data/forest_5/merging{}.csv'.format(i)
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
    plt.plot(iter, fit, color='green', label='$\pm8$ meters') 
for i in range(1,8):
    filename = '/home/maderja1/workspace/src/occupancy_grid_merger_package/experiments/fitness_val_plot/data/forest_15/merging{}.csv'.format(i)
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
    plt.plot(iter, fit, color='blue', label='$\pm25$ meters') 
for i in range(1,3):
    filename = '/home/maderja1/workspace/src/occupancy_grid_merger_package/experiments/fitness_val_plot/data/forest_30/merging{}.csv'.format(i)
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
    plt.plot(iter, fit, color='red', label='$\pm50$ meters') 
plt.legend(framealpha=1, loc='lower right')

plt.title("\n".join(wrap("Acceptance index value of UAVs in the forest envionment.", 80)))
plt.xlabel('Iteration')
plt.ylabel('Acceptance index value')
plt.axis(aspect='equal')

#ax.axis('off')

plt.savefig(SAVE_TO_FIGURE)
if SHOW_FIGURE:
    plt.show()  
