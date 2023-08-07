from evogym import WorldObject, EvoWorld, EvoSim, EvoViewer, sample_robot, get_full_connectivity, is_connected
import os
import numpy as np
import random
import gym
import evogym.envs
import randomWorldGen
import copy

#finds score of given robot
def scoreChecker(e):  
	return e["previousScore"]

def valid(robot):
	return (is_connected(robot) and
			(3 in robot or 4 in robot))

#finds open space for new offspring
def findOpenNeighbors(worldLocation):
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
	##todo: use is_in_bounds from utils.py

def calcFitness(sim):
	startScore = 0
	endScore = 0
	size = 0
	for i in sim.object_pos_at_time(0, 'robot')[0]:
		startScore += i
	for i in sim.object_pos_at_time(sim.get_time(), 'robot')[0]:
		size += 1
		endScore += i
	return (endScore - startScore)/size

def mutate(robot):
	
	old_shape = robot
	count = 0
	while count <= 5000:
		pos = tuple(np.random.randint(0,4,2))
		robot[pos] = np.random.randint(0,4)

		if valid(robot):
			break

		robot = old_shape
		count += 1
	if count > 5000:
		raise Exception("Can't find a valid mutation after 5000 tries!")
	return robot

def action(robot, steps):
	action = []
	for _ in range(count_actuators(robot)):
		action.append(np.sin(steps/3 + (_*0.1))+1)
	return np.array(action)

def count_actuators(robot):
	count = 0
	for _x in robot.get_structure().flatten():
		if _x == 3 or _x == 4:
			count += 1
	return count

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
	s1Robot["previousScore"] = None

	newRobot = WorldObject()
	newRobot.load_from_array(
		name = 'robot',
		structure = s1Robot["structure"],
		connections=s1Robot["connections"])

	# print(newRobot.__repr__())
	s1Robot["object"] = newRobot
	
	aliveRobots.append(s1Robot)
	worldPops[str(s1Robot["location"])] = 1
	
	simRunTime = 100
		
	for t in range(simRunTime):

		#simulate every alive Robot
		for x in aliveRobots:
			if not(x["previousScore"] == None):
				continue

			xloc = x["location"][0]
			yloc = x["location"][1]

			
			curRobot = x["object"]
			curRobot.set_pos(3, 1)

			worldArray[x["location"][0]][x["location"][1]].add_object(curRobot)

			worldArray[x["location"][0]][x["location"][1]].move_object('robot', 3, 1)

			#can't add or change any functions from other files???
			#problem occurs here: 'no object named robot' (fixed with deepcopy())
			#there is 100% an object named robot, however, have double, triple checked
			
			sim = EvoSim(worldArray[x["location"][0]][x["location"][1]])
			sim.reset()

			# viewer = EvoViewer(sim)
			# viewer.track_objects('robot')
			# viewer.show_debug_window() ##GLFW initilization fails here
			#goes to viewer.py, then into the function _init_viewer
			#_init_viewer calls on object Viewer from PythonBindings.cpp?
			#this is exactly where the error occurs i think


			#actually simming the environment
			#how long is 100? i guess i don't know until i can actually see it
			for i in range(100):
				curAction = action(curRobot, sim.get_time())
				sim.set_action('robot', curAction)
				sim.step()
			

			#calculate fitness
			x["previousScore"] = calcFitness(sim)
			#remove robot
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
		print ("Top scorer: \n" + str(aliveRobots[0]["structure"]))

		#reproduce
		newRobots = []
		for x in aliveRobots:
			#I would put this below findng a valid space, but it breaks the program
			#mutate the robot that is multiplying
			newRobotShape = x["structure"].copy()
			## ^^slow asl, but only way mutate is able to work
			#regular copy() does not work 100% of the time
			newRobotShape = mutate(newRobotShape)

			#find valid space
			newloc = []
			if worldPops[str(x["location"])] > 5:
				newloc = findOpenNeighbors(x["location"])
				if newloc == None:
					continue
			else:
				newloc.append(x["location"])
			newRobot = {}
			newRobot["location"] = random.choice(newloc)
			newRobot["structure"] = newRobotShape
			newRobot["connections"] = get_full_connectivity(newRobotShape)
			newRobot["previousScore"] = None

			#add new worldObject of new robot
			object = WorldObject()
			object.load_from_array(
				name = 'robot',
				structure = newRobot["structure"],
				connections=newRobot["connections"])
			newRobot["object"] = object

			#add object to world
			newRobots.append(newRobot)
			worldPops[str(newRobot["location"])] += 1
		#add new robots to alive list
		for i in newRobots:
			aliveRobots.append(i)


	print ("sim over")


	##THROWS ERROR 
	# viewer.render('img')