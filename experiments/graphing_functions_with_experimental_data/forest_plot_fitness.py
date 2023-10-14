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
SAVE_TO_FIGURE = "forest_fitness_maze.pdf"

SHOW_FIGURE = True
mpl.rc('font', size=15)
fig1 = plt.figure(figsize = (6,5))

plt.title("\n".join(wrap("Fitness function value of UAVs  in the forest environment.", 80)))
plt.xlabel('Iteration')
plt.ylabel('Fitness value')
plt.axis(aspect='equal')

for i in range(1,8):
    filename = '/home/maderja1/workspace/src/occupancy_grid_merger_package/experiments/fitness_val_plot/data/forest_5/merging{}.csv'.format(i)
    with open(filename, 'rb') as csvfile:
        csvreader = csv.reader(csvfile)
        next(csvreader)
        iter = []
        fit = []
        for row in csvreader:
            iter.append(int(row[0]))
            fit.append(int(row[2]))
        print(iter)
    print(fit)
    plt.plot(iter, fit, color='green', label='$\pm8$ meters') 
    #plt.plot(iter, fit, color='blue') 
for i in range(1,8):
    filename = '/home/maderja1/workspace/src/occupancy_grid_merger_package/experiments/fitness_val_plot/data/forest_15/merging{}.csv'.format(i)
    with open(filename, 'rb') as csvfile:
        csvreader = csv.reader(csvfile)
        next(csvreader)
        iter = []
        fit = []
        for row in csvreader:
            iter.append(int(row[0]))
            fit.append(int(row[2]))
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
            fit.append(int(row[2]))
        print(iter)
    print(fit)
    plt.plot(iter, fit, color='red', label='$\pm50$ meters') 
plt.legend(framealpha=1, loc='upper left')
#ax.axis('off')

plt.savefig(SAVE_TO_FIGURE)
if SHOW_FIGURE:
    plt.show()  
