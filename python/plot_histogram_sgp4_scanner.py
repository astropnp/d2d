'''
Copyright (c) 2014-2016 Kartik Kumar, Dinamica Srl (me@kartikkumar.com)
Copyright (c) 2014-2016 Abhishek Agrawal, Delft University of Technology (abhishek.agrawal@protonmail.com)
Distributed under the MIT License.
See accompanying file LICENSE.md or copy at http://opensource.org/licenses/MIT
'''

# Set up modules and packages
# Plotting
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib import rcParams
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import axes3d

# I/O
import commentjson
import json
from pprint import pprint

# SQL database
import sqlite3

# Numerical
import numpy as np
import pandas as pd

# System
import sys
import time

print ""
print "---------------------------------------------------------------------------------"
print "                               D2D                                               "
print "                              0.0.2                                              "
print "         Copyright (c) 2016, K. Kumar (me@kartikkumar.com)                       "
print "         Copyright (c) 2016, A. Agrawal (abhishek.agrawal@protonmail.com)        "
print "---------------------------------------------------------------------------------"
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

print "Input data files being read ..."

output_path_prefix = config["output_directory"] + '/'

print "Fetching scan data from database ..."

# Connect to SQLite database.
try:
    database = sqlite3.connect(config['database'])

except sqlite3.Error, e:
    print "Error %s:" % e.args[0]
    sys.exit(1)
                      
# Fetch scan data.
scan_data = pd.read_sql( "SELECT                                                                                                   \
                              " + config['error'] + "                                                                                    \
                              FROM sgp4_scanner_results                                                                                 \
                              WHERE success = 1;",                                                                                      \
                              database )
scan_data.columns = [ config['error'] ]

# The histogram of the data
x = scan_data[ config['error'] ]
# print x    
n, bins, patches = plt.hist( x, bins=50, normed=1, facecolor='green', alpha=0.75 )

# Figure properties
plt.xlabel('arrival position error magnitude [km]')
# plt.ylabel('Probability')
# plt.title(r'$\mathrm{Histogram\ of\ IQ:}\ \mu=100,\ \sigma=15$')
# plt.axis([40, 160, 0, 0.03])
plt.grid(True)
    
# Save figure to file.
plt.savefig(config["output_directory"] + "/" + config["histogram_figure"] + "_" +                                               \
                       ".png", dpi=config["figure_dpi"])
plt.close()

print "Figure generated successfully!"
print ""

# Close SQLite database if it's still open.
if database:
    database.close()

# Stop timer
end_time = time.time( )

# Print elapsed time
print "Script time: " + str("{:,g}".format(end_time - start_time)) + "s"

print ""
print "------------------------------------------------------------------"
print "                         Exited successfully!                     "
print "------------------------------------------------------------------"
print ""