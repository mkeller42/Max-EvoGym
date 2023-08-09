# first 7 or so spaces should be flat/normal
# then after that, create random variance in world generation

import random
import numpy as np
import json
from evogym import EvoWorld
import os


def randomizer(presetWorld, x, y, worldSeed):
	f = open(presetWorld)  #!!whatever world you want to randomize
	data = json.load(f)
	
	ground = data['objects']['ground']['indices']
	connections = data['objects']['ground']['neighbors']
	types = data['objects']['ground']['types']
	width = data['grid_width'] #should work with any worldsize? as long as the height is at least one above the highest ground block
	# print (ground)
	extraGround = []
	removeList = []

	
	randConstant = 0.4  #frequency of changes (50% at 0.5, 40% at 0.4, etc.)
	random.seed(worldSeed * 10000 + x*100 + y) #so one could replicate findings

	##add new blocks to envo
	for i in ground:
		# finds x-coordinates of all blocks
		ix = i%width
		# checks if blocks can be manipulated (wont if covered by another block OR if block is in starting era)
		if (i+width in ground or ix < 10):
			continue
		# randomly adds/removes blocks from first layer
		if random.random() < randConstant:
			if random.random() < .5:
				extraGround.append(i+width)
				types.append(5)
			else:
				ground.remove(i)
				removeList.append(i)
				types.pop()
	# this can't be in previous for loop due to errors with dicts
	for i in extraGround:
		ground.append(i)



	#remove connections for new blocks
	for item in removeList:
		connections.pop(str(item))
	for key in connections:
		for item in removeList:
			if (item in connections[key]):
				connections[key].remove(item)
				
	# add connections for new blocks
	for i in ground:
		if not(str(i) in connections.keys()):
			neighbors = [] #program needs this for some reason idk
			neighbors.append(i-width) #!!assumes there is always block underneath
			if (str(i+1) in ground):
				neighbors.append(i+1)
			elif (str(i-1) in ground):
				neighbors.append(i-1)
			connections[str(i)] = neighbors

	with open('random.json', 'w') as outfile:
		json.dump(data, outfile)
	world = EvoWorld.from_json(os.path.join('random.json'))
	return world, data

# unnecessary function but useful for visualizing in terminal
def visualize(presetWorld):
	with open('random.json', 'w') as outfile:
		json.dump(presetWorld, outfile)
	world = EvoWorld.from_json(os.path.join('random.json'))
	world.pretty_print()

