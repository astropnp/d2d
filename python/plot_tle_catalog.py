'''
Copyright (c) 2014-2016, K. Kumar (me@kartikkumar.com)
Copyright (c) 2016, E.J. Hekma (ennehekma@gmail.com)
All rights reserved.
'''

###################################################################################################
# Set up input deck
###################################################################################################

# Set path to TLE catalog file.


###################################################################################################

'''
						DO NOT EDIT PARAMETERS BEYOND THIS POINT!!!
'''

###################################################################################################
# Set up modules and packages
###################################################################################################

from sgp4.earth_gravity import wgs72
from sgp4.io import twoline2rv
from sgp4.propagation import getgravconst

from matplotlib import rcParams
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from twoBodyMethods import convertMeanMotionToSemiMajorAxis
import sys

# I/O
import commentjson
import json
from pprint import pprint
import sqlite3


# Parse JSON configuration file
# Raise exception if wrong number of inputs are provided to script
if len(sys.argv) != 2:
    raise Exception("Only provide a JSON config file as input!")

json_data = open(sys.argv[1])
config = commentjson.load(json_data)
json_data.close()
pprint(config)

fontSize = config['fontSize']
figureDPI = config['figure_dpi']
# exit()

###################################################################################################

###################################################################################################
# Read and store TLE catalog
###################################################################################################

sma = []
ecc5 = []
incl5 = []

raan3 = []
ecc3 = []
aop3 = []
inclinations3 = []

# order = ['all','SSO','GEO','HEO']
for x in xrange(0,len(config['databases'])):
	# tleCatalogFilePathNew = eval(str('tleCatalogFilePath' + str(x)))
	tleCatalogFilePathNew = config['databases'][x]
	# Read in catalog and store lines in list.
	fileHandle = open(tleCatalogFilePathNew)
	catalogLines = fileHandle.readlines()
	fileHandle.close()

	# Strip newline and return carriage characters.
	for i in xrange(len(catalogLines)):
	    catalogLines[i] = catalogLines[i].strip('\r\n')

	# Parse TLE entries and store debris objects.
	debrisObjects = []
	for tleEntry in xrange(0,len(catalogLines),config['tleEntryNumberOfLines']):
		debrisObjects.append(twoline2rv(catalogLines[tleEntry+1], catalogLines[tleEntry+2], wgs72))

	# Sort list of debris objects based on inclination.
	inclinationSortedObjects = sorted(debrisObjects, key=lambda x: x.inclo, reverse=False)

	inclinations = []
	raan = []
	ecc = []
	aop = []
	for i in xrange(len(inclinationSortedObjects)):
		inclinations.append(inclinationSortedObjects[i].inclo)
		raan.append(inclinationSortedObjects[i].nodeo)
		ecc.append(inclinationSortedObjects[i].ecco)
		aop.append(inclinationSortedObjects[i].argpo)
	
	smatemp = []
	smatemp = [convertMeanMotionToSemiMajorAxis(debrisObject.no/60.0,							  \
			   getgravconst('wgs72')[1])-6373													  \
		  	   for debrisObject in debrisObjects]
	smatemp2 = []
	smatemp2 = pd.DataFrame(smatemp, columns=[str(x)])
	sma.append(smatemp2)
	
	ecctemp = []
	ecctemp = [debrisObject.ecco for debrisObject in debrisObjects]
	ecc4 = []
	ecc4 = pd.DataFrame(ecctemp, columns=[str(x)])
	ecc5.append(ecc4)
	
	incltemp = []
	incltemp = [np.rad2deg(debrisObject.inclo) for debrisObject in debrisObjects]
	incl4 = []
	incl4 = pd.DataFrame(incltemp, columns=[str(x)])
	incl5.append(incl4)

	raan2 = []
	raan2 = pd.DataFrame(raan, columns=[str(x)])
	raan3.append(raan2)
	
	ecc2 = []
	ecc2 = pd.DataFrame(ecc, columns=[str(x)])
	ecc3.append(ecc2)

	aop2 = []
	aop2 = pd.DataFrame(aop, columns=[str(x)])
	aop3.append(aop2)

	inclinations2 = []
	inclinations2 = pd.DataFrame(inclinations, columns=[str(x)])
	inclinations3.append(inclinations2)

sma = pd.concat(sma, axis=1)
ecc5 = pd.concat(ecc5, axis=1)
incl5 = pd.concat(incl5, axis=1)

raan3 = pd.concat(raan3, axis=1)
ecc3 = pd.concat(ecc3, axis=1)
aop3 = pd.concat(aop3, axis=1)
inclinations3 = pd.concat(inclinations3, axis=1)

###################################################################################################

###################################################################################################
# Generate plots
###################################################################################################

# Set font size for plot labels.
rcParams.update({'font.size': fontSize})

# Plot distribution of eccentricity [-] against semi-major axis [km].
figure = plt.figure()
axis = figure.add_subplot(111)
plt.xlabel("Semi-major axis altitude [km]")
plt.ylabel("Eccentricity [-]")
plt.ticklabel_format(style='sci', axis='x', scilimits=(0,0))
plt.plot(sma['0'],ecc3['0'], 																      \
		 marker='.', markersize=1, color='k', linestyle='none')
if len(config['databases']) > 1:
	plt.plot(sma['1'],ecc3['1'], 																  \
			 marker='s', markersize=10, color='c', linestyle='none')
	if len(config['databases']) > 2:
		plt.plot(sma['2'],ecc3['2'], 															  \
				 marker='^', markersize=10, color='g', linestyle='none')
		if len(config['databases']) > 3:
			plt.plot(sma['3'],ecc3['3'], 														  \
					 marker='D', markersize=6, color='r', linestyle='none')
# axis.set_xlim(xmax=0.4e5)
figure.set_tight_layout(True)
plt.savefig(config['output_directory'] + 
			"/figure1_debrisPopulation_eccentricityVsSemiMajorAxis.pdf",
			dpi = figureDPI)
plt.close()

# Plot components of eccentricity vector [-].
figure = plt.figure()
axis = figure.add_subplot(111)
plt.xlabel("$e \cos{\omega}$ [-]")
plt.ylabel("$e \sin{\omega}$ [-]")
plt.plot(ecc3['0']*np.cos(aop3['0']),ecc3['0']*np.sin(aop3['0']), 						  		  \
		 marker='.', markersize=1, color='k', linestyle='none')
if len(config['databases']) > 1:
	plt.plot(ecc3['1']*np.cos(aop3['1']),ecc3['1']*np.sin(aop3['1']), 						  	  \
			 marker='s', markersize=10, color='c', linestyle='none')
	if len(config['databases']) > 2:
		plt.plot(ecc3['2']*np.cos(aop3['2']),ecc3['2']*np.sin(aop3['2']), 						  \
				 marker='^', markersize=10, color='g', linestyle='none')
		if len(config['databases']) > 3:
			plt.plot(ecc3['3']*np.cos(aop3['3']),ecc3['3']*np.sin(aop3['3']), 					  \
					 marker='D', markersize=6, color='r', linestyle='none')
plt.axis('equal')
# axis.set_xlim(xmin=-.82, xmax=.82)
# axis.set_ylim(ymin=-.82, ymax=.82)
# axis.set(xticks=[-.8,-.4,0,.4,.8])
# axis.set(yticks=[-.8,-.4,0,.4,.8])
figure.set_tight_layout(True)
plt.savefig(config['output_directory'] + 
			"/figure2_debrisPopulation_eccentricityVector.pdf", 
			dpi = figureDPI)
plt.close()


# Plot distribution of inclination [deg] against semi-major axis [km].
figure = plt.figure()
axis = figure.add_subplot(111)
plt.xlabel("Semi-major axis altitude [km]")
plt.ylabel("Inclination [deg]")
plt.ticklabel_format(style='sci', axis='x', scilimits=(0,0))
plt.plot(sma['0'],incl5['0'], 																      \
		 marker='.', markersize=1, color='k', linestyle='none')
if len(config['databases']) > 1:
	plt.plot(sma['1'],incl5['1'], 																  \
			 marker='s', markersize=10, color='c', linestyle='none')
	if len(config['databases']) > 2:
		plt.plot(sma['2'],incl5['2'], 															  \
				 marker='^', markersize=10, color='g', linestyle='none')
		if len(config['databases']) > 3:
			plt.plot(sma['3'],incl5['3'], 														  \
					 marker='D', markersize=6, color='r', linestyle='none')
# axis.set_xlim(xmax=0.4e5)
figure.set_tight_layout(True)
plt.savefig(config['output_directory'] + 
			"/figure3_debrisPopulation_inclinationVsSemiMajorAxis.pdf",
			dpi = figureDPI)
plt.close()

# Plot components of inclination vector [deg].
figure = plt.figure()
axis = figure.add_subplot(111)
plt.xlabel("$i \cos{\Omega}$ [deg]")
plt.ylabel("$i \sin{\Omega}$ [deg]")
plt.plot(np.rad2deg(inclinations3['0'])*np.cos(raan3['0']),									  
		 np.rad2deg(inclinations3['0'])*np.sin(raan3['0']), 							  	  
		 marker='.', markersize=1, color='k', linestyle='none')
if len(config['databases']) > 1:
	plt.plot(np.rad2deg(inclinations3['1'])*np.cos(raan3['1']),									  \
			 np.rad2deg(inclinations3['1'])*np.sin(raan3['1']), 							  	  \
			 marker='s', markersize=10, color='c', linestyle='none')
	if len(config['databases']) > 2:
		plt.plot(np.rad2deg(inclinations3['2'])*np.cos(raan3['2']),					  		      \
				 np.rad2deg(inclinations3['2'])*np.sin(raan3['2']), 							  \
			 	 marker='^', markersize=10, color='g', linestyle='none')
		if len(config['databases']) > 3:
			plt.plot(np.rad2deg(inclinations3['3'])*np.cos(raan3['3']),							  \
		 			 np.rad2deg(inclinations3['3'])*np.sin(raan3['3']), 					  	  \
		 			 marker='D', markersize=6, color='r', linestyle='none')
plt.axis('equal')
# axis.set_xlim(xmin=-110.0, xmax=110.0)
# axis.set_ylim(ymin=-110.0, ymax=110.0)
figure.set_tight_layout(True)
plt.savefig(config['output_directory'] + 
			"/figure4_debrisPopulation_inclinationVector.pdf", 
			dpi = figureDPI)
plt.close()




###################################################################################################
###################################################################################################