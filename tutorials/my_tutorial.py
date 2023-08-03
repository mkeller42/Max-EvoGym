from evogym import WorldObject, EvoWorld, EvoSim, EvoViewer, sample_robot
import os
import numpy as np
import random
import gym
import evogym.envs
import randomWorldGen

#finds score of given robot
def scoreChecker(e):  
	return e["previousScore"]

#finds open space for new offspring
def findOpenNeighbors(worldLocation, worldArray):
	possibleSpace = []
	removedSpaces = [] #program needs this to work? idk man
	ix = worldLocation[0]
	iy = worldLocation[1]
	possibleSpace.extend([[ix+1, iy], [ix-1, iy], [ix, iy+1], [ix, iy-1]])
	for j in possibleSpace:
		if not((0<=j[0]<worldWidth)and(0<=j[1]<worldHeight)):
			removedSpaces.append(j)
	for x in removedSpaces: #todo: make this function smaller
		possibleSpace.remove(x)
	for j in possibleSpace:
		if worldPops[str([j[0],j[1]])] > 5 or j in removedSpaces:
			possibleSpace.remove(j)
	return possibleSpace
	##todo: find adjacent available space


if __name__ == '__main__':

	print ("Hello World")
    
	worldWidth = 5 #seeds for generating each individual world randomly (used together) 
	worldHeight = 5
	worldSeed = 4 #seed for generating each entire collection of worlds randomly
	#!!!currently only properly seeds/randomzies up to 9x9 world size

	worldArray = []
	worldPops = {}

	for i in range(worldHeight):
		worldArray.append([])
		for j in range(worldWidth):
			world = randomWorldGen.randomizer(os.path.join('world_data', 'my_environment.json'), i+1, j+1, worldSeed)
			# print ("World: (" + str(i) + "," + str(j) + ") loaded")
			worldArray[i].append(world)
			worldPops[str([j, i])] = 0


	aliveRobots = []
	fossilizedRobots = []
	s1Robot = {}
	s1Robot["structure"], s1Robot["connections"] = sample_robot((5,5))
	s1Robot["location"] = [1,1]
	s1Robot["previousScore"] = 0
	# robot_structure, robot_connections = sample_robot((5, 5))
	# WorldObject.from_array(WorldObject, 'robot', robot_structure, robot_connections)
	aliveRobots.append(s1Robot)
	worldPops[str(s1Robot["location"])] = 1
	
	simRunTime = 100
		
	for t in range(simRunTime):

		#simulate every alive Robot
		for x in aliveRobots:
			xloc = x["location"][0]
			yloc = x["location"][1]

			worldArray[x["location"][0]][x["location"][1]].add_from_array(
				name='robot', 
				structure=x["structure"], 
				x=3, 
				y=1, 
				connections=x["connections"])
			
			# print ("robot added to world (1,1)")
			
			# worldArray[0][0].pretty_print()
			# worldArray[1][1].pretty_print()


			sim = EvoSim(worldArray[x["location"][0]][x["location"][1]])
			sim.reset()

			# viewer = EvoViewer(sim)
			# viewer.track_objects('robot')
			# viewer.show_debug_window() ##GLFW initilization fails here
			#goes to viewer.py, then into the function _init_viewer
			#_init_viewer calls on object Viewer from PythonBindings.cpp?
			#this is exactly where the error occurs i think

			for i in range(100):
				sim.set_action(
					'robot', 
					np.random.uniform(
						low = 0.6,
						high = 1.6,
						size=(sim.get_dim_action_space('robot'),))
					)
				sim.step()

			startScore = 0
			endScore = 0
			for i in sim.object_pos_at_time(0, 'robot')[0]:
				startScore += i
			for i in sim.object_pos_at_time(100, 'robot')[0]:
				endScore += i

			# print ("start:" + str(startScore))
			# print ("end: " + str(endScore))
			x["previousScore"] = endScore-startScore

			worldArray[x["location"][0]][x["location"][1]].remove_object('robot')

		
		aliveRobots.sort(key=scoreChecker, reverse = True)
		#fossilize bad robots
		if len(aliveRobots) > 100:
			# print("paring")
			curDead = aliveRobots[101:len(aliveRobots)+1]
			del aliveRobots[101:len(aliveRobots)+1]
			fossilizedRobots.append(curDead)
			for i in curDead:
				worldPops[str([i["location"][0], i["location"][1]])] -= 1
	

		print ("Top Score: " + str(aliveRobots[0]["previousScore"]))
		print ("Num Robots: " + str(len(aliveRobots)))

		#reproduce
		newRobots = []
		for x in aliveRobots:
			newRobot = x.copy()
			if worldPops[str(s1Robot["location"])] > 5:
				# print("overpop")
				newloc = findOpenNeighbors(newRobot["location"], worldArray)
				if newloc == None:
					continue
				newRobot["location"] = random.choice(newloc)
				# print(str(newRobot["location"]))
			newRobots.append(newRobot)
			worldPops[str(newRobot["location"])] += 1
		for i in newRobots:
			aliveRobots.append(i)
	


	##THROWS ERROR 
	# viewer.render('img')