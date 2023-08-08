from evogym import WorldObject, EvoWorld, EvoSim, EvoViewer, sample_robot, get_full_connectivity, is_connected
import os
import numpy as np
import random
import gym
import evogym.envs
import randomWorldGen
import copy
import plotly.express as px

#finds score of given robot
def scoreChecker(e):  
	return e["score"]

def valid(robot):
	return (is_connected(robot) and
			(3 in robot or 4 in robot))

#finds open space for new offspring
def findOpenNeighbors(worldLocation, competing, maxSpaces):
	possibleSpace = []
	removedSpaces = []
	ix = worldLocation[0]
	iy = worldLocation[1]
	possibleSpace.extend([[ix+1, iy], [ix-1, iy], [ix, iy+1], [ix, iy-1]])
	for j in possibleSpace:
		if not((0<=j[0]<worldWidth)and(0<=j[1]<worldHeight)):
			removedSpaces.append(j)
	for x in removedSpaces:
		possibleSpace.remove(x)
	removedSpaces = []
	for j in possibleSpace:
		if len(worldPops[j[0]][j[1]]) >= maxSpaces and competing == False:
			removedSpaces.append(j)
	for x in removedSpaces:
		possibleSpace.remove(x)
	if len(possibleSpace) == 0:
		return None
	return possibleSpace

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

def worldPrint(worldArray, scores):
	for row in range(len(scores)):
		for i in range(len(scores[0])):
			scores[row][i] = round(scores[row][i], 2)
		print(scores[row])

def getWorstScorer(list):
	worstScore = 10000
	worstScorer = None
	worstID = None
	for i in list:
		if i["score"] < worstScore:
			worstScore = i["score"]
			worstScorer = i
			worstID = i["id"]
	return worstScorer, worstID

def robotSim(robot, worldArray):
	xloc = robot["location"][0]
	yloc = robot["location"][1]
	robot["object"].set_pos(3, 2)
	worldArray[xloc][yloc].add_object(robot["object"])


	sim = EvoSim(worldArray[xloc][yloc])
	sim.reset()

	for i in range(100):
		curAction = action(robot["object"], sim.get_time())
		sim.set_action('robot', curAction)
		sim.step()

	worldArray[xloc][yloc].remove_object('robot')
	return sim

def createRobObj(robotDict):
	robObject = WorldObject()
	robObject.load_from_array(
		name = 'robot',
		structure = robotDict["structure"],
		connections = robotDict["connections"])
	return robObject

def getScore(robDict, worldArray, location):
	robDict["location"] = location
	curSim = robotSim(robDict, worldArray)
	robDict["score"] = calcFitness(curSim)
	return robDict

def delDeadRobs(curDead, aliveRobots):
	for i in curDead:
		for j in range(len(aliveRobots)):
			if i['id'] == aliveRobots[j]['id']:
				del aliveRobots[j]
				break
	
	curDead = []
	return curDead, aliveRobots
	

if __name__ == '__main__':


	globalID = 1 #ID variable for keeping track of robots
    
	worldWidth = 10 #seeds for generating each individual world randomly (used together) 
	worldHeight = 10 #maximum size of 99x99!
	worldSeed = 1 #seed for generating each entire collection of worlds randomly

	maxRobotsPerSpace = 3 #how many robots are allowed to occupy the same space
	
	simRunTime = 200 #number of rounds the sim will run

	worldArray = []
	bestScores = []
	worldPops = []
	curDead = []

	#generate random worlds and add them to array worldArray
	for i in range(worldHeight):
		worldArray.append([])
		bestScores.append([])
		for j in range(worldWidth):
			world = randomWorldGen.randomizer(os.path.join('world_data', 'flat_env.json'), i+1, j+1, worldSeed)
			worldArray[i].append(world)
			bestScores[i].append(-1.00)

	aliveRobots = []
	fossilizedRobots = []

	#creating worldPop lists within lists
	for i in range(worldHeight):
		worldPops.append([])
		for j in range(worldWidth):
			worldPops[i].append([])

	
	#initial robot
	s1Robot = {}
	
	s1Robot["structure"], s1Robot["connections"] = sample_robot((5,5))
	s1Robot["location"] = [1,1]
	s1Robot["score"] = None
	s1Robot["id"] = globalID

	s1Robot["object"] = createRobObj(s1Robot)

	curSim = robotSim(s1Robot, worldArray)
	s1Robot["score"] = calcFitness(curSim)
	
	aliveRobots.append(s1Robot)
	worldPops[1][1].append(s1Robot)
	
	
	for t in range(simRunTime):
		
		#TURNED OFF for now
		#fossilize unfit robots (keep population under 50 before reproduction)
		# if len(aliveRobots) > 200:
		# 	# print("paring")
		# 	curDead = aliveRobots[200:len(aliveRobots)+1]
		# 	del aliveRobots[200:len(aliveRobots)+1]
		# 	fossilizedRobots.append(curDead)
		# 	delDeadRobs(curDead, aliveRobots)
		# 	for i in worldPops:
		# 		for j in i:
		# 			if len(j) > 3:
		# 				print("TOO MANY: " + str(len(j)))
		

		#REPRODUCTION
		newRobots = []
		for x in aliveRobots:
			newRobot = {}
			newRobot["structure"] = mutate(x["structure"].copy())
			newRobot["connections"] = get_full_connectivity(newRobot["structure"])
			newRobot["object"] = createRobObj(newRobot)

			#find valid space
			newloc = []
			if len(worldPops[x["location"][0]][x["location"][1]]) >= maxRobotsPerSpace:
				#when findOpenNeighbors var==False, does not compete with others
				newloc = findOpenNeighbors(x["location"], False, maxRobotsPerSpace)
				if newloc == None:
					#when True, does compete
					newloc = findOpenNeighbors(x["location"], True, maxRobotsPerSpace)
					
					compZone = random.choice(newloc)
					newRobot = getScore(newRobot, worldArray, compZone)
					
					worst, worstID = getWorstScorer(worldPops[compZone[0]][compZone[1]])
					if newRobot["score"] > worst["score"]:
						
						#use id here to find and remove the worst scorer. can't just do remove(), throws error
						for j in range(len(worldPops[compZone[0]][compZone[1]])):
							if worldPops[compZone[0]][compZone[1]][j]["id"] == worstID:
								curDead.append(worldPops[compZone[0]][compZone[1]][j])
								del worldPops[compZone[0]][compZone[1]][j]
								break
					continue
				#goes here if: adjacent empty space found
				else:
					loc = random.choice(newloc)
					newRobot = getScore(newRobot, worldArray, loc)
			#goes here if: empty space in parent world found
			else:
				newRobot = getScore(newRobot, worldArray, x["location"])
			
			#add object to world
			xloc = newRobot["location"][0]
			yloc = newRobot["location"][1]

			globalID += 1
			newRobot["id"] = globalID
			newRobots.append(newRobot)
			worldPops[xloc][yloc].append(newRobot)


			if newRobot["score"] > bestScores[xloc][yloc]:
				bestScores[xloc][yloc] = newRobot["score"]

		#add new robots to alive list
		for i in newRobots:
			aliveRobots.append(i)
		
		curDead, aliveRobots = delDeadRobs(curDead, aliveRobots)

		aliveRobots.sort(key=scoreChecker, reverse = True)

		print ("Round: " + str(t))
		worldPrint(worldArray, bestScores)
		print ("Top Score: " + str(aliveRobots[0]["score"]))
		print ("Top scorer: \n" + str(aliveRobots[0]["structure"]))


		# fig = px.imshow(bestScores, range_color=[-1,20])

		# fig.show()


	print ("sim over")

	#current robot record: ~14.2