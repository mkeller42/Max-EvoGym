from evogym import WorldObject, EvoWorld, EvoSim, EvoViewer, sample_robot, get_full_connectivity, is_connected
import os
import numpy as np
import random
import gym
import evogym.envs
import randomWorldGen
import copy
import json
import plotly.express as px
import multiprocessing as mp

#finds score of given robot
def scoreChecker(e):  
	return e["score"]

#checks if robot shape is valid (no 'islands' of voxels)
def valid(robot):
	return (is_connected(robot) and
			(3 in robot or 4 in robot))

#finds open space for new offspring
def findOpenSpace(worldLocation, worldPops, maxSpaces):
	possibleSpaces = []
	finalSpaces = []
	ix = worldLocation[0]
	iy = worldLocation[1]
	possibleSpaces.extend([[ix+1, iy], [ix-1, iy], [ix, iy+1], [ix, iy-1]])
	for j in possibleSpaces:
		if (0<=j[0]<worldWidth)and(0<=j[1]<worldHeight) and (len(worldPops[j[0]][j[1]]) < maxSpaces):
			finalSpaces.append(j)
	return finalSpaces

def findOccupiedSpace(worldLocation, worldPops, maxSpaces):
	possibleSpaces = []
	finalSpaces = []
	ix = worldLocation[0]
	iy = worldLocation[1]
	possibleSpaces.extend([[ix+1, iy], [ix-1, iy], [ix, iy+1], [ix, iy-1]])
	for j in possibleSpaces:
		if (0<=j[0]<worldWidth)and(0<=j[1]<worldHeight) and (len(worldPops[j[0]][j[1]]) >= maxSpaces):
			finalSpaces.append(j)
	return finalSpaces

#calculate fitness (currently average voxel x-position)
def calcFitness(sim):
	score = 0
	size = 0
	for i in sim.object_pos_at_time(sim.get_time(), 'robot')[0]:
		size += 1
		score += i
	return (score)/size

#mutate robot
def mutate(robot, coParent):

	old_shape_0 = robot
	old_shape_1 = coParent
	count = 0
	while count <= 5000:

		new_shape = old_shape_0
		new_shape_blueprint = []
		for i in range(5):
			new_shape_blueprint.append(list(np.random.choice([0,1], size=5, replace=True)))

		for i in new_shape_blueprint:
			for j in i:
				if j == 1:
					new_shape[i][j] = old_shape_1[i][j]

		pos = tuple(np.random.randint(0,4,2))
		new_shape[pos] = np.random.randint(0,4)

		if valid(new_shape):
			robot = new_shape
			break

		count += 1
	if count > 5000:
		raise Exception("Can't find a valid mutation after 5000 tries!")
	return robot

def alt_mutate(robot):
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

#determines robot actions
def action(robot, steps):
	action = []
	for _ in range(count_actuators(robot)):
		action.append(np.sin(steps/3 + (_*0.1))+1)
	return np.array(action)

#returns # of actuators in robot
def count_actuators(robot):
	count = 0
	for _x in robot.get_structure().flatten():
		if _x == 3 or _x == 4:
			count += 1
	return count

#gets 'map' of world scores 
def worldData(scores, worldHeight):
	w = []
	for row in range(len(scores)):
		for i in range(len(scores[worldHeight-1])):
			scores[row][i] = round(scores[row][i], 2)
		w.append(scores[row])
	return w

#returns the least fit member of a certain space
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

#simulates robot in environment
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
		curScore = calcFitness(sim)
		if robot["score"] == None:
			robot["score"] = curScore
		if curScore > robot["score"]:
			robot["score"] = curScore

	worldArray[xloc][yloc].remove_object('robot')
	return sim

#creates a robot object to put in a world
def createRobObj(robotDict):
	robObject = WorldObject()
	robObject.load_from_array(
		name = 'robot',
		structure = robotDict["structure"],
		connections = robotDict["connections"])
	return robObject

#creates robot object, simulates it, and calculates score, returns the score
def updateRobot(robDict, worldArray, location):
	robDict["location"] = location
	curSim = robotSim(robDict, worldArray)
	robDict["score"] = calcFitness(curSim)
	return robDict

#removes the outcompeted robots from the program, stores in a list
def delDeadRobs(curDead, aliveRobots):
	for i in curDead:
		for j in range(len(aliveRobots)):
			if i['id'] == aliveRobots[j]['id']:
				fossilizedRobots.append(i)
				del aliveRobots[j]
				break
	
	curDead = []
	return curDead, aliveRobots
	
#coverts world data into a json file
def write_json(new_data, filename):
    desired_dir = "./saved_data"
    full_path = os.path.join(desired_dir, filename)
    with open(full_path, 'w') as f:
        json_string=json.dumps(new_data)
        f.write(json_string)

def correctCord(cords):
	tempCords = [100,100]
	tempCords[0] = cords[1]
	tempCords[1] = cords[0]	
	return tempCords

#returns best scorer/score given a certain area of the worldmap(to find species)
def getBestScorer(worldPops, columns, worldHeight):
	bestScore = -100
	bestScorer = None
	for i in columns:
		for j in range(worldHeight):
			if not worldPops[j][i]:
				continue
			if worldPops[j][i][0]["score"] > bestScore:
				bestScore = worldPops[j][i][0]["score"]
				bestScorer = worldPops[j][i][0]
	return bestScore, bestScorer

def getParent(robot, worldPops, maxSpaces):
	parentRobot = None
	parentSpaces = findOccupiedSpace(robot["location"], worldPops, maxSpaces)

	if not parentSpaces:
		return None

	parentLoc = random.choice(parentSpaces)
	parentRobot = worldPops[parentLoc[0]][parentLoc[1]][0]

	return parentRobot

if __name__ == '__main__':


	globalID = 1 #ID variable for keeping track of robots
    
	worldWidth = 10 #seeds for generating each individual world randomly (used together) 
	worldHeight = 10 #maximum size of 99x99!
	worldSeed = 1 #seed for generating each entire collection of worlds randomly
	robotSeed = 1

	maxRobotsPerSpace = 1 #how many robots are allowed to occupy the same space
	mutationRate = 0.5

	simRunTime = 200 #number of rounds the sim will run

	worldArray = []
	bestScores = []
	worldPops = []
	curDead = []
	aliveRobots = []
	fossilizedRobots = []

	#lists for what COLUMNS each environment (1 or 2) should appear in
	#(will later expand to be more than columns)
	env_1_list = [0,1,2,3,4]
	env_2_list = [5,6,7,8,9]
	total_env_list = [[env_1_list], [env_2_list]]

	#generate random worlds and add them to array worldArray
	worldFile = {}
	for i in range(worldHeight):
		worldArray.append([])
		bestScores.append([])
		for j in range(worldWidth):
			if j in env_1_list:
				world, dataFile = randomWorldGen.randomizer(os.path.join('world_data', 'flat_env.json'), j+1, i+1, worldSeed)
			elif j in env_2_list:
				world, dataFile = randomWorldGen.randomizer(os.path.join('world_data', 'hill_env.json'), j+1, i+1, worldSeed)
			worldArray[i].append(world)
			bestScores[i].append(-1.00)
			worldFile["World [" + str(j) + "," + str(i) + "]"] = dataFile
			world.pretty_print()
	write_json(worldFile, "_worlds.json")

	#creating worldPop lists within lists
	for i in range(worldHeight):
		worldPops.append([])
		for j in range(worldWidth):
			worldPops[i].append([])

	
	#initial robot
	s1Robot = {}
	#I know I am using a preset seed here, but this actually fully randomizes it.
	#It isn't fully randomized otherwise? idk, but its currently NOT SEEDED
	random.seed(robotSeed) 
	
	s1Robot["structure"], s1Robot["connections"] = sample_robot((5,5))
	s1Robot["location"] = [1,1]
	s1Robot["trueLocation"] = [1,1]
	s1Robot["score"] = None
	s1Robot["id"] = globalID
	s1Robot["object"] = createRobObj(s1Robot)
	s1Robot = updateRobot(s1Robot, worldArray, s1Robot["location"])
	aliveRobots.append(s1Robot)
	worldPops[1][1].append(s1Robot)
	
	
	
	for t in range(simRunTime):
		
		# #TURNED OFF for now
		# popCap = 100
		#  #fossilize unfit robots (keep population under x before reproduction)
		# if len(aliveRobots) > popCap:
		# 	# print("paring")
		# 	curDead = aliveRobots[popCap:len(aliveRobots)+1]
		# 	del aliveRobots[popCap:len(aliveRobots)+1]
		# 	fossilizedRobots.append(curDead)
		# 	delDeadRobs(curDead, aliveRobots)
		# for i in worldPops:
		# 	for j in i:
		# 		if len(j) > maxRobotsPerSpace:
		# 			print("TOO MANY: " + str(len(j)))
		

		#REPRODUCTION
		newRobots = []
		for x in aliveRobots:

			#create offspring
			newRobot = {}
			#determine if mutation occurs
			if random.random() < mutationRate:
				coParent = getParent(x, worldPops, maxRobotsPerSpace)
				if coParent == None:
					newRobot["structure"] = alt_mutate(x["structure"].copy())
				else:
					newRobot["structure"] = mutate(x["structure"].copy(), coParent["structure"].copy())
			else:
				newRobot["structure"] = x["structure"].copy()
			newRobot["connections"] = get_full_connectivity(newRobot["structure"])
			newRobot["object"] = createRobObj(newRobot)
			newRobot["score"] = None
			#adds the location as its parent's location, but doesn't put into world yet
			newRobot["location"] = x["location"]
			newRobots.append(newRobot)



		#SIMULATION
		for x in newRobots:
			#find valid space
			locList = []
			loc = [-1, -1]
			#this statement only necessary when # of bots per space > 1
			if len(worldPops[x["location"][0]][x["location"][1]]) >= maxRobotsPerSpace:
				#when findOpenNeighbors var==False, does not compete with others
				locList = findOpenSpace(x["location"], worldPops, maxRobotsPerSpace)
				if not locList:
					#when True, does compete
					locList = findOccupiedSpace(x["location"], worldPops, maxRobotsPerSpace)
					loc = random.choice(locList)
					newRobot = updateRobot(x, worldArray, loc)
					
					#if newRobot's score is better than the curRobot score
					if newRobot["score"] > worldPops[loc[0]][loc[1]][-1]["score"]:
						#remove unfit robot
						curDead.append(worldPops[loc[0]][loc[1]][-1])
						worldPops[loc[0]][loc[1]].pop()
					else:
						continue
					
				#goes here if: adjacent empty space found
				else:
					loc = random.choice(locList)
					newRobot = updateRobot(x, worldArray, loc)
			#goes here if: empty space in parent world found
			else:
				loc = x["location"]
				newRobot = updateRobot(x, worldArray, loc)
			
			
			#add object to world
			xloc = newRobot["location"][0]
			yloc = newRobot["location"][1]
			globalID += 1
			newRobot["id"] = globalID
			newRobot["trueLocation"] = correctCord(newRobot["location"])
			worldPops[xloc][yloc].append(newRobot)
			aliveRobots.append(newRobot)
			#if only 1 robot per space, don't need this
			# worldPops[xloc][yloc].sort(key=scoreChecker, reverse = True)

			#tracks best scores yet
			if newRobot["score"] > bestScores[xloc][yloc]:
				bestScores[xloc][yloc] = newRobot["score"]

		#update robot lists
		curDead, aliveRobots = delDeadRobs(curDead, aliveRobots)
		aliveRobots.sort(key=scoreChecker, reverse = True)



		#create save file of world state
		infoDict = {
			"round": str(t),
			"totalRobots": str(len(aliveRobots)),
			"totalDeadRobots": str(len(fossilizedRobots)),
			"bestScoreWorld": str(worldData(bestScores, worldHeight)),
			"topScore": str(aliveRobots[0]["score"]),
			"topRobot": str(aliveRobots[0]["structure"]),
			"topRobotLocation": str(aliveRobots[0]["trueLocation"]),
			"env_1_BestScorer": str(getBestScorer(worldPops, env_1_list, worldHeight)),
			"env_2_BestScorer": str(getBestScorer(worldPops, env_2_list, worldHeight))
		}
		write_json(infoDict, "dataRound" + str(t) + ".json")


		#print select data from each round to terminal
		print ("Round: " + str(t))
		print ("Total Robots: " + str(len(aliveRobots)))
		for i in worldData(bestScores, worldHeight):
			x = i.copy()
			x.insert(env_2_list[0], '|')
			print (x)
		print ("Top Score: " + str(aliveRobots[0]["score"]))
		print ("Top Scorer Location: " + str(aliveRobots[0]["trueLocation"]))
		print ("Top Scorer: \n" + str(aliveRobots[0]["structure"]))


		#code for creating heatmap
		# fig = px.imshow(bestScores, range_color=[-1,20])
		# fig.show()


	print ("sim over")

	#current robot record: ~14.2