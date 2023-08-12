from evogym import WorldObject, EvoWorld, EvoSim, EvoViewer, sample_robot, get_full_connectivity, is_connected
import os
import numpy as np
import random
import gym
import evogym.envs
import randomWorldGen
import robot
import environment
import json
import plotly.express as px
import multiprocessing as mp

#finds score of given robot
def scoreChecker(e):  
	return e.get_score()

#finds open space for new offspring
def findOpenSpace(worldLocation, worldArray, maxSpaces):
	possibleSpaces = []
	finalSpaces = []
	ix = worldLocation[0]
	iy = worldLocation[1]
	possibleSpaces.extend([[ix+1, iy], [ix-1, iy], [ix, iy+1], [ix, iy-1]])
	for j in possibleSpaces:
		if (0<=j[0]<worldWidth)and(0<=j[1]<worldHeight) and (worldArray[j[0]][j[1]].getNumRobots() < maxSpaces):
			finalSpaces.append(j)
	return finalSpaces

def findOccupiedSpace(worldLocation, worldArray, maxSpaces):
	possibleSpaces = []
	finalSpaces = []
	ix = worldLocation[0]
	iy = worldLocation[1]
	possibleSpaces.extend([[ix+1, iy], [ix-1, iy], [ix, iy+1], [ix, iy-1]])
	for j in possibleSpaces:
		if (0<=j[0]<worldWidth)and(0<=j[1]<worldHeight) and (worldArray[j[0]][j[1]].getNumRobots() >= maxSpaces):
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

#gets 'map' of world scores 
def worldData(worldArray, worldHeight):
	w = []
	for row in range(len(worldArray)):
		w.append([])
		for i in range(len(worldArray[worldHeight-1])):
			w[row].append(round(worldArray[row][i].get_score(), 2))
	return w

#removes the outcompeted robots from the program, stores in a list
def delDeadRobs(curDead, aliveRobots):
	for i in curDead:
		for j in range(len(aliveRobots)):
			if i.get_id() == aliveRobots[j].get_id():
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
def getBestScorer(worldArray, columns, worldHeight):
	bestScorer = None
	for i in columns:
		for j in range(worldHeight):
			if not worldArray[j][i].getRobot():
				continue
			elif not bestScorer:
				bestScorer = worldArray[j][i].getRobot()
			elif worldArray[j][i].get_score() > bestScorer.get_score():
				bestScorer = worldArray[j][i].getRobot()
	if bestScorer == None:
		return None
	return bestScorer.get_structure(), bestScorer.get_genes()

def getParent(robot, worldArray, maxSpaces):
	parentRobot = None
	parentSpaces = findOccupiedSpace(robot.get_location(), worldArray, maxSpaces)

	if not parentSpaces:
		return None

	parentLoc = random.choice(parentSpaces)
	parentRobot = worldArray[parentLoc[0]][parentLoc[1]].getRobot()

	return parentRobot

def robotSim(robot, world):
	robot.getWorldObj().set_pos(3, 2)
	world.add_object(robot.getWorldObj())

	sim = EvoSim(world)
	sim.reset()

	robot.set_score(-100)

	for i in range(100):
		#if want to change how actions are decided, do it here
		curAction = robot.evolvedAction(sim.get_time(), robot.get_genes(), robot.get_structure())
		sim.set_action('robot', curAction)
		sim.step()
		curScore = calcFitness(sim)
		if curScore > robot.get_score():
			robot.set_score(curScore)
		elif robot.get_score() == None:
			robot.set_score(curScore)	

	world.remove_object('robot')
	return sim

if __name__ == '__main__':


	globalID = 1 #ID variable for keeping track of robots
    
	worldWidth = 16 #seeds for generating each individual world randomly (used together) 
	worldHeight = 16 #maximum size of 99x99!
	worldSeed = 3 #seed for generating each entire collection of worlds randomly
	robotSeed = 3

	maxRobotsPerSpace = 1 #how many robots are allowed to occupy the same space
	mutationRate = 1

	simRunTime = 500 #number of rounds the sim will run

	worldArray = []
	curDead = []
	aliveRobots = []
	fossilizedRobots = []

	#lists for what COLUMNS each environment (1 or 2) should appear in
	#(will later expand to be more than columns)
	env_1_list = [0,1,2,3,4,5,6,7]
	env_2_list = [8,9,10,11,12,13,14,15]
	total_env_list = [[env_1_list], [env_2_list]]

	#generate random worlds and add them to array worldArray
	worldFile = {}
	for i in range(worldHeight):
		worldArray.append([])
		for j in range(worldWidth):
			if j in env_1_list:
				evo, dataFile = randomWorldGen.randomizer(os.path.join('world_data', 'flat_env.json'), j+1, i+1, worldSeed)
			elif j in env_2_list:
				evo, dataFile = randomWorldGen.randomizer(os.path.join('world_data', 'hill_env.json'), j+1, i+1, worldSeed)
			world = environment.World(evo, j, i)
			worldArray[i].append(world)
			worldFile["World [" + str(j) + "," + str(i) + "]"] = dataFile
	write_json(worldFile, "_worlds.json")
	
	#I know I am using a preset seed here, but this actually fully randomizes it.
	#It isn't fully randomized otherwise? idk, but its currently NOT SEEDED
	random.seed(robotSeed) 

	s1Robot = robot.Robot(sample_robot((5,5)), globalID)
	s1Robot.set_location([1,1])
	s1Robot.set_true_location([1,1])
	curSim = robotSim(s1Robot, worldArray[s1Robot.get_location()[0]][s1Robot.get_location()[1]].getEvo())
	worldArray[1][1].setRobot(s1Robot)

	aliveRobots.append(s1Robot)
	
	
	
	for t in range(simRunTime):


		#REPRODUCTION
		newRobots = []
		for x in aliveRobots:
			#create offspring
			globalID += 1
			newRobot = robot.Robot((x.get_structure(), x.get_connections()), globalID)
			#determine if mutation occurs
			if random.random() < mutationRate:
				coParent = getParent(x, worldArray, maxRobotsPerSpace)
				newRobot.set_structure(newRobot.mutate(x.get_structure().copy(), x.get_genes().copy(), coParent))
			else:
				newRobot.set_structure(x.get_structure().copy())
			#adds the location as its parent's location, but doesn't put into world yet
			newRobot.set_location(x.get_location())
			newRobots.append(newRobot)


		#SIMULATION
		for x in newRobots:
			#find valid space
			locList = []
			loc = [-1, -1]
			
			locList = findOpenSpace(x.get_location(), worldArray, maxRobotsPerSpace)
			if not locList:
				locList = findOccupiedSpace(x.get_location(), worldArray, maxRobotsPerSpace)
				loc = random.choice(locList)
				x.set_location(loc)
				curSim = robotSim(x, worldArray[x.get_location()[0]][x.get_location()[1]].getEvo())

				#if newRobot's score is better than the curRobot score
				if x.get_score() > worldArray[loc[0]][loc[1]].get_score():
					#remove unfit robot
					curDead.append(worldArray[loc[0]][loc[1]].getRobot())
				else:
					continue
				
			#goes here if: adjacent empty space found
			else:
				loc = random.choice(locList)
				x.set_location(loc)
				curSim = robotSim(x, worldArray[x.get_location()[0]][x.get_location()[1]].getEvo())
			
			#add object to world
			xloc, yloc = x.get_location()
			x.set_true_location(correctCord([xloc, yloc]))
			worldArray[xloc][yloc].setRobot(x)
			aliveRobots.append(x)

		#update robot lists
		curDead, aliveRobots = delDeadRobs(curDead, aliveRobots)
		aliveRobots.sort(key=scoreChecker, reverse = True)


		#create save file of world state
		infoDict = {
			"round": str(t),
			"totalRobots": str(len(aliveRobots)),
			"totalDeadRobots": str(len(fossilizedRobots)),
			"bestScoreWorld": str(worldData(worldArray, worldHeight)),
			"topScore": str(aliveRobots[0].get_score()),
			"topRobot": str(aliveRobots[0].get_structure()),
			"topGenes": str(aliveRobots[0].get_genes()),
			"topRobotLocation": str(aliveRobots[0].get_true_location()),
			"env_1_BestScorer": str(getBestScorer(worldArray, env_1_list, worldHeight)),
			"env_2_BestScorer": str(getBestScorer(worldArray, env_2_list, worldHeight))
		}
		write_json(infoDict, "dataRound" + str(t) + ".json")


		#print select data from each round to terminal
		print ("Round: " + str(t))
		print ("Total Robots: " + str(len(aliveRobots)))
		for i in worldData(worldArray, worldHeight):
			x = i.copy()
			x.insert(env_2_list[0], '|')
			print (x)
		print ("Top Score: " + str(aliveRobots[0].get_score()))
		print ("Top Scorer Location: " + str(aliveRobots[0].get_true_location()))
		print ("Top Scorer: \n" + str(aliveRobots[0].get_structure()))


		#code for creating heatmap
		# fig = px.imshow(bestScores, range_color=[-1,20])
		# fig.show()


	print ("sim over")

	#current robot record: ~14.2