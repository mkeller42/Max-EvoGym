

class World():
    
	def __init__(self, evo, xloc, yloc) -> None:
		self.terrain = evo
		self.curRobot = None
		self.trueLocation = [xloc, yloc]
		self.numRobots = 0

	def getRobot(self, ):
		return self.curRobot

	def setRobot(self, robot):
		self.curRobot = robot
		self.numRobots += 1

	def getEvo(self, ):
		return self.terrain

	def getNumRobots(self, ):
		return self.numRobots
	
	def get_score(self, ):
		if self.curRobot == None:
			return -100
		return self.curRobot.get_score()
	

