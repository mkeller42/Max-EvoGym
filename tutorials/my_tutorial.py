from evogym import WorldObject, EvoWorld, EvoSim, EvoViewer, sample_robot, get_full_connectivity, is_in_bounds, is_connected
import os
import numpy as np
import random
import gym
import evogym.envs
import randomWorldGen

#finds score of given robot
def scoreChecker(e):  
	return e["previousScore"]

def valid(robot):
	return (is_connected(robot["structure"]) and
			(3 in robot["structure"] or 4 in robot["structure"]))

#finds open space for new offspring
def findOpenNeighbors(worldLocation):
	possibleSpace = []
	removedSpaces = [] #program needs this to work? idk man
	ix = worldLocation[0]
	iy = worldLocation[1]
	possibleSpace.extend([[ix+1, iy], [ix-1, iy], [ix, iy+1], [ix, iy-1]])
	for j in possibleSpace:
		if not(is_in_bounds(j[0], j[1], worldWidth, worldHeight)):
			removedSpaces.append(j)
	for x in removedSpaces: #todo: make this function smaller
		possibleSpace.remove(x)
	for j in possibleSpace:
		if worldPops[str([j[0],j[1]])] > 5 or j in removedSpaces:
			possibleSpace.remove(j)
	return possibleSpace
	##todo: use is_in_bounds from utils.py

def mutate(robot):
	old_shape = robot["structure"]
	count = 0
	while count <= 5000:
		pos = tuple(np.random.randint(0,4,2))
		robot["structure"][pos] = np.random.randint(0,4)
		robot["connections"] = get_full_connectivity(robot["structure"])

		if valid(robot) and is_connected(robot["structure"]):
			break

		robot["structure"] = old_shape
		count += 1
	if count > 5000:
		raise Exception("Can't find a valid mutation after 5000 tries!")
	return robot


if __name__ == '__main__':
    
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

	newRobot = WorldObject()
	newRobot.load_from_array(
		name = 'robot',
		structure = s1Robot["structure"],
		connections=s1Robot["connections"])

	# print(newRobot.__repr__())
	s1Robot["base"] = newRobot
	
	aliveRobots.append(s1Robot)
	worldPops[str(s1Robot["location"])] = 1
	
	simRunTime = 100
		
	for t in range(simRunTime):

		#simulate every alive Robot
		for x in aliveRobots:
			xloc = x["location"][0]
			yloc = x["location"][1]

			x["connections"] = get_full_connectivity(x["structure"])
			##NO IDEA why this has to be here. This is done in mutate function and should be fine. idk

			curRobot = x["base"]
			curRobot.set_pos(3, 1)

			worldArray[x["location"][0]][x["location"][1]].add_object(curRobot)

			worldArray[x["location"][0]][x["location"][1]].move_object('robot', 3, 1)


			#debugging
			# print(curRobot.__repr__())
			# worldArray[x["location"][0]][x["location"][1]].pretty_print()
			
			print(curRobot.get_name())

			#problem occurs here: 'no object named robot'
			sim = EvoSim(worldArray[x["location"][0]][x["location"][1]])
			sim.reset()

			# viewer = EvoViewer(sim)
			# viewer.track_objects('robot')
			# viewer.show_debug_window() ##GLFW initilization fails here
			#goes to viewer.py, then into the function _init_viewer
			#_init_viewer calls on object Viewer from PythonBindings.cpp?
			#this is exactly where the error occurs i think


			#actually simming the environmeny
			#how long is 100? i guess i don't know until i can actually see it
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
			curDead = aliveRobots[50:len(aliveRobots)+1]
			del aliveRobots[50:len(aliveRobots)+1]
			fossilizedRobots.append(curDead)
			for i in curDead:
				worldPops[str([i["location"][0], i["location"][1]])] -= 1
	

		print ("Top Score: " + str(aliveRobots[0]["previousScore"]))
		print ("Top scorer: " + str(aliveRobots[0]["structure"]))

		#reproduce
		newRobots = []
		for x in aliveRobots:
			#I would put this below findng a valid space, but it breaks the program
			#mutate the robot that is multiplying
			newRobot = x.copy()
			newRobot = mutate(newRobot)

			#find valid space
			if worldPops[str(x["location"])] > 5:
				newloc = findOpenNeighbors(x["location"], worldArray)
				if newloc == None:
					continue
				newRobot["location"] = random.choice(newloc)

			#add new worldObject of new robot
			base = WorldObject()
			base.load_from_array(
				name = 'robot',
				structure = newRobot["structure"],
				connections=newRobot["connections"])
			newRobot["base"] = base

			#add object to world
			newRobots.append(newRobot)
			worldPops[str(newRobot["location"])] += 1
		#add new robots to alive list
		for i in newRobots:
			aliveRobots.append(i)


	print ("sim over")


	##THROWS ERROR 
	# viewer.render('img')