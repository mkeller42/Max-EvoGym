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

#finds score of given robot
def scoreChecker(e):  
	return e["score"]

def valid(robot):
	return (is_connected(robot) and
			(3 in robot or 4 in robot))

#finds open space for new offspring
def findOpenNeighbors(worldLocation, competing, maxSpaces, worldPops):
	possibleSpaces = []
	competingSpaces = []
	passiveSpaces = []
	ix = worldLocation[0]
	iy = worldLocation[1]
	possibleSpaces.extend([[ix+1, iy], [ix-1, iy], [ix, iy+1], [ix, iy-1]])
	for j in possibleSpaces:
		if (0<=j[0]<worldWidth)and(0<=j[1]<worldHeight):
			competingSpaces.append(j)
	if (competing == True):
		return competingSpaces
	for k in competingSpaces:
		if (len(worldPops[k[0]][k[1]]) < maxSpaces):
			passiveSpaces.append(k)
	return passiveSpaces

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

def worldData(scores):
	w = []
	for row in range(len(scores)):
		for i in range(len(scores[0])):
			scores[row][i] = round(scores[row][i], 2)
		w.append(str(scores[row]))
	return w

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
				fossilizedRobots.append(i)
				del aliveRobots[j]
				break
	
	curDead = []
	return curDead, aliveRobots
	
def write_json(new_data, filename):
    desired_dir = "./saved_data"
    full_path = os.path.join(desired_dir, filename)
    with open(full_path, 'w') as f:
        json_string=json.dumps(new_data)
        f.write(json_string)

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
			world = randomWorldGen.randomizer(os.path.join('world_data', 'hill_env.json'), i+1, j+1, worldSeed)
			worldArray[i].append(world)
			bestScores[i].append(-1.00)
			world.pretty_print()

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
		
		# TURNED OFF for now
		# fossilize unfit robots (keep population under 50 before reproduction)
		if len(aliveRobots) > 100:
			# print("paring")
			curDead = aliveRobots[100:len(aliveRobots)+1]
			del aliveRobots[100:len(aliveRobots)+1]
			fossilizedRobots.append(curDead)
			delDeadRobs(curDead, aliveRobots)
		for i in worldPops:
			for j in i:
				if len(j) > maxRobotsPerSpace:
					print("TOO MANY: " + str(len(j)))
		

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
				newloc = findOpenNeighbors(x["location"], False, maxRobotsPerSpace, worldPops)
				if newloc == []:
					#when True, does compete
					newloc = findOpenNeighbors(x["location"], True, maxRobotsPerSpace, worldPops)
					
					compZone = random.choice(newloc)
					newRobot = getScore(newRobot, worldArray, compZone)
					
					if newRobot["score"] > worldPops[compZone[0]][compZone[1]][-1]["score"]:
						curDead.append(worldPops[compZone[0]][compZone[1]][-1])
						worldPops[compZone[0]][compZone[1]].pop()
					else:
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
			worldPops[xloc][yloc].sort(key=scoreChecker, reverse = True)

			
			#tracks best scores yet
			if newRobot["score"] > bestScores[xloc][yloc]:
				bestScores[xloc][yloc] = newRobot["score"]

		#add new robots to alive list
		for i in newRobots:
			aliveRobots.append(i)
		
		curDead, aliveRobots = delDeadRobs(curDead, aliveRobots)

		aliveRobots.sort(key=scoreChecker, reverse = True)

		infoDict = {
			"round": str(t),
			"totalRobots": str(len(aliveRobots)),
			"totalDeadRobots": str(len(fossilizedRobots)),
			"bestScoreWorld": str(worldData(bestScores)),
			"topScore": str(aliveRobots[0]["score"]),
			"topRobot": str(aliveRobots[0]["structure"]),
			"topRobotLocation": str(aliveRobots[0]["location"])
		}
		write_json(infoDict, "dataRound" + str(t) + ".json")


		print ("Round: " + str(t))
		print ("Total Robots: " + str(len(aliveRobots)))
		for i in worldData(bestScores):
			print (i)
		print ("Top Score: " + str(aliveRobots[0]["score"]))
		print ("Top scorer: \n" + str(aliveRobots[0]["structure"]))


		# fig = px.imshow(bestScores, range_color=[-1,20])

		# fig.show()


	print ("sim over")

	#current robot record: ~14.2