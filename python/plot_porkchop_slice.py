'''
Copyright (c) 2014-2016, Kartik Kumar (me@kartikkumar.com)
Copyright (c) 2016, Enne Hekma (ennehekma@gmail.com)
All rights reserved.
'''

# Set up modules and packages
# Plotting
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import gridspec
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import axes3d
from nlcmap import nlcmap
from matplotlib import rcParams
from mpl_toolkits.axes_grid1.inset_locator import inset_axes
import math

# I/O
import commentjson
import json
from pprint import pprint
import sqlite3

# Numerical
import numpy as np
import pandas as pd

# System
import sys
import time

print ""
print "------------------------------------------------------------------"
print "                               D2D                                "
print "                              0.0.2                               "
print "      Copyright (c) 2015-2016, K. Kumar (me@kartikkumar.com)      "
print "      Copyright (c) 2016, E.J. Hekma (ennehekma@gmail.com)        "
print "------------------------------------------------------------------"
print ""

# Start timer.
start_time = time.time( )

print ""
print "******************************************************************"
print "                          Input parameters                        "
print "******************************************************************"
print ""

# Parse JSON configuration file
# Raise exception if wrong number of inputs are provided to script
if len(sys.argv) != 2:
    raise Exception("Only provide a JSON config file as input!")

json_data = open(sys.argv[1])
config = commentjson.load(json_data)
json_data.close()
pprint(config)

print ""
print "******************************************************************"
print "                            Operations                            "
print "******************************************************************"
print ""

print "Fetching scan data from database ..."

# Connect to SQLite database.
try:
    database = sqlite3.connect(config['database'])

except sqlite3.Error, e:

    print "Error %s:" % e.args[0]
    sys.exit(1)

# Find the best transfer to select the objects.
if config['objects']==[]:
    best = pd.read_sql(" SELECT DISTINCT departure_object_id, arrival_object_id, time_of_flight,  \
                                         departure_epoch, transfer_delta_v                        \
                         FROM lambert_scanner_results                                             \
                         ORDER BY transfer_delta_v ASC                                            \
                         LIMIT 10 ",                                                              \
                         database )

    a = best['departure_object_id'][0]
    b = best['arrival_object_id'][0]
else:
    a = config['objects'][0]
    b = config['objects'][1]

# Set departue epoch to zeor or user input.
if config['departure_epoch']==0:
    c = best['departure_epoch'][0]
else:
    c = config['departure_epoch']

print "Porkchop plot slice figure being generated for transfer between TLE objects",a,"and",b,"..."
print ""
cmap = plt.get_cmap('jet')


if config['plot_lambert']==True:
    print "Plot of only Lambert being generated ..."
    times_of_flight = pd.read_sql_query(" SELECT DISTINCT time_of_flight                          \
                    FROM lambert_scanner_results",
                    database)

    transfer_delta_vs = pd.read_sql_query(" SELECT transfer_delta_v                               \
                    FROM lambert_scanner_results                                                  \
                    WHERE departure_object_id =" + a + "                                          \
                    and arrival_object_id =" + b + "                                              \
                    and departure_epoch BETWEEN " + str(c-0.00001) +" AND                         \
                                        "+str(c+0.00001), # Between 0.864 seconds before and
                                                            # after the given departure epoch
                    database)
    fig=plt.figure()
    ax1 = fig.add_subplot(111)
    ax1.scatter(times_of_flight,transfer_delta_vs, color='black')
    formatter = matplotlib.ticker.ScalarFormatter(useOffset=False)
    ax1.xaxis.set_major_formatter(formatter)
    ax1.yaxis.set_major_formatter(formatter)
    plt.ylim([0,math.ceil(transfer_delta_vs.max(0)[0]*1.01)])
    if config['cutoff']!=0:
      plt.ylim(0,config['cutoff'])
    ax1.set_xlabel('T$_{ToF}$ [s]', fontsize=13)
    ax1.set_ylabel('Total transfer $\Delta V$ [km/s]', fontsize=13)
    plt.tight_layout()
    plt.grid()
    plt.savefig(config["output_directory"] + "/" + config["scan_figure"] + ".png",            
        dpi=config["figure_dpi"])

    print "Lambert plot generated."
    print ""


if config['plot_difference_atom']==True:
    print "Lambert, Atom and comparison plots being generated ..."
    # Plot porkchop plot slice
    atomdata = pd.read_sql_query("  SELECT DISTINCT (lambert_scanner_results.time_of_flight),     \
                                           transfer_delta_v,                                      \
                                           atom_transfer_delta_v                                  \
                                    FROM lambert_scanner_results                                  \
                                    INNER JOIN atom_scanner_results                               \
                                    ON lambert_scanner_results.transfer_id                        \
                                       = atom_scanner_results.lambert_transfer_id                 \
                                    WHERE departure_object_id =" + a + "                          \
                                          and arrival_object_id =" + b + "                        \
                                          and departure_epoch BETWEEN " + str(c-0.00001) +"       \
                                                              AND "+str(c+0.00001), 
                                    database)
    # Between 0.864 seconds before and after the given departure epoch.

    # print atomdata
    atomdata.columns = ['time_of_flight','transfer_delta_v','atom_transfer_delta_v']

    cmap = plt.get_cmap('jet')
    fig=plt.figure()
    ax1 = plt.subplot(2,1,1)
    plt.scatter(atomdata['time_of_flight'],atomdata['transfer_delta_v'], 
                label = "Lambert", color='g')
    plt.scatter(atomdata['time_of_flight'],atomdata['atom_transfer_delta_v'],
                label = "Atom", color='b')

    formatter = matplotlib.ticker.ScalarFormatter(useOffset=False)
    ax1.xaxis.set_major_formatter(formatter)
    ax1.yaxis.set_major_formatter(formatter)
    
    if config['cutoff']!=0:
        plt.ylim(0,config['cutoff'])
    
    ax1.set_ylabel('Total transfer $\Delta V$ [km/s]', fontsize=13)
    plt.title("Porkchop plot of TLE elements " +str(a) + " to " + str(b) + 
              " at departure epoch " + str(c) + " [mjd]", fontsize=10, y=1.02)
    plt.legend(loc='best')
    plt.grid()

    ax2 = plt.subplot(2,1,2)
    ax2 = plt.scatter(atomdata['time_of_flight'],
                      atomdata['transfer_delta_v']-atomdata['atom_transfer_delta_v'], 
                      label = "Atom - Lambert", 
                      color = 'k')
    plt.ylabel('$\delta \ \Delta V$ [km/s] \n Lambert - Atom difference', fontsize=13)
    plt.xlabel('T$_{ToF}$ [s]', fontsize=13)
    plt.tight_layout()
    plt.axhline(y=0, color='k')
    plt.grid()

    plt.savefig(config["output_directory"] + "/" + config["scan_figure"] + "_comparison.png",
                dpi=config["figure_dpi"])
    print "Difference plots are generated."


print ""
print "Figures generated successfully!"

# Stop timer
end_time = time.time( )
print ""
print "------------------------------------------------------------------"
print ""
# Print elapsed time

print ""
print "------------------------------------------------------------------"
print "                         Exited successfully!                     "
print "                              "
print "                         Script time: " + str("{:,g}".format(end_time - start_time)) + "s"
print "------------------------------------------------------------------"
print ""
