#/*PARAMETERS*/ 
import re
import random as rd
from decimal import Decimal
import Drone

#!!!!Attention verifications are currently desactivated to save computation time
class Param:
	def __init__(self, model, A, priorities, K, nbPt, k1, k2, t1, t2, sep12, sep21, fixedIntentions = [], fixedLevels = []):
		self.nbFL = model.nb_FL				#number of flight levels
		self.vVert = Drone.vertical_speed	#vertical seed in m/s
		self.dFL = model.FL_sep				#vertical separation between two flight levels in m = 30ft
		self.tSeedUp = Drone.vertical_acceleration_time(0, self.vVert) #time needed to acceleate from 0 to vVert with acceleration aVert and vv
		
		self.nbflights = len(A)					#number of flight instances
		self.nbTrajs = len(K)					#total number of alternative trajectories
		
		self.maxDelay = model.delay_max			#maximum delay

		self.A = A								#set of flight intentions
		#if any(a not in priorities for a in self.A):
		#	raise ValueError("Priority must be defined for every flight intention!")
		self.priorities = priorities			#flight intention priorities
		#if any(d < 0 for k, d, a in K):
		#	raise ValueError("Trajectory duration may not be negative!")
		#if any(a not in self.A for k, d, a in K):
		#	raise ValueError("Flight of a trajectory must be in the list of flights!")
		self.K = []								#set of all horizontal trajectories
		self.d = {}								#duration of horizontal trajectory
		self.mon_vol = {}						#flight intention to which belongs given horizontal trajectory 
		for k, d, a in K:
			self.K.append(k)
			self.d[k] = d
			self.mon_vol[k] = a

		#if any(n < 0 for n in nbPt):
		#	raise ValueError("Number of intersecting points may not be negative!")
		self.nbPtHor = nbPt[0]					#number of horizontal intersecting points
		self.nbPtClmb = nbPt[1]					#number of climbing-horizontal intersecting points
		self.nbPtDesc = nbPt[2]					#number of descending-horizontal intersecting points
		self.nbPtDep = nbPt[3]					#number of deprture intersecting points
		self.nbPtArr = nbPt[4]					#number of arrival intersecting points
		self.nbPtInter = self.nbPtHor + self.nbPtClmb + self.nbPtDesc + self.nbPtDep + self.nbPtArr		#total number of intersecting points
		
		self.P = list(range(self.nbPtInter)) if self.nbPtInter > 0 else []	#set of all horizontal intersecting points
		self.Phor = list(range(self.nbPtHor)) if self.nbPtHor > 0 else []
		self.Pclmb = list(range(self.nbPtHor, self.nbPtHor+self.nbPtClmb)) if self.nbPtClmb > 0 else []
		self.Pdesc = list(range(self.nbPtHor+self.nbPtClmb, self.nbPtHor+self.nbPtClmb+self.nbPtDesc)) if self.nbPtDesc > 0 else []
		self.Pdep = list(range(self.nbPtHor+self.nbPtClmb+self.nbPtDesc, self.nbPtHor+self.nbPtClmb+self.nbPtDesc+self.nbPtDep)) if self.nbPtDep > 0 else []
		self.Parr = list(range(self.nbPtHor+self.nbPtClmb+self.nbPtDesc+self.nbPtDep, self.nbPtInter)) if self.nbPtArr > 0 else []
		
		
		#if len(k1) != self.nbPtInter or len(k2) != self.nbPtInter or \
		#   len(t1) != self.nbPtInter or len(t2) != self.nbPtInter or \
		#   len(sep12) != self.nbPtInter or len(sep21) != self.nbPtInter:
		#	raise ValueError("Parameters k1, k2, t1, t2, sep12 and sep21 must be defined exactly for every interesecting point !")
		#if any(i not in self.K for i in k1) or any(i not in self.K for i in k2):
		#	raise ValueError("Trajectory of an intersecting point must be in the list of trajectories!")
		#if any(i < 0 for i in t1) or any(i < 0 for i in t2):
		#	raise ValueError("Arrival time at intersecting point may not be negative!")
		#if any(i < 0 for i in sep12) or any(i < 0 for i in sep21):
		#	print("SEP12 : ")
		#	for j in sep12:
		#		if j < 0:
		#			print(j)
		#	print("SEP21 : ")
		#	for j in sep21:
		#		if j < 0:
		#			print(j)
		#	raise ValueError("Separation at intersecting point may not be negative!")
		self.k1 = k1 						#first trajectory of intersecting point
		self.k2 = k2						#second trajectory of intersecting point
		self.t1 = t1						#first trajectory planned time over intersecting point
		self.t2 = t2						#second trajectory planned time over intersecting point
		self.sep12 = sep12					#required time separation at intersecting point if first then second passes
		self.sep21 = sep21					#required time separation at intersecting point if second then first passes
											
											#set of pairs of "intersecting"" flight intnetions
		self.AInter = list(set((self.mon_vol[k1[p]], self.mon_vol[k2[p]]) for p in self.P).union(set((self.mon_vol[k2[p]], self.mon_vol[k1[p]]) for p in self.P)))
											
											#set of pairs of "intersecting"" trajectories
		self.KInterHor = list(set((k1[p], k2[p]) for p in self.Phor).union(set((k2[p], k1[p]) for p in self.Phor)))
		KInterClmb = set((k1[p], k2[p]) for p in self.Pclmb).union(set((k2[p], k1[p]) for p in self.Pclmb))
		#self.KInterClmb = list(KInterClmb)
		KInterDesc = set((k1[p], k2[p]) for p in self.Pdesc).union(set((k2[p], k1[p]) for p in self.Pdesc))
		#self.KInterDesc = list(KInterDesc)
		self.KInterEvol = list(KInterClmb.union(KInterDesc))
		#self.KInterDep = list(set((k1[p], k2[p]) for p in self.Pdep).union(set((k2[p], k1[p]) for p in self.Pdep)))
		#self.KInterArr = list(set((k1[p], k2[p]) for p in self.Parr).union(set((k2[p], k1[p]) for p in self.Parr)))
		
		#big M parameters
		self.FLmax = self.nbFL					#big M is set to max difference between two flight levels
		self.delta = 0.5					#small number that is less than difference between any different flight levels
		self.deltaFLmax = (self.nbFL - 1)*self.dFL	#maximum difference in meters between two flight levels
		self.deltaFLmin = -self.deltaFLmax	#minimum difference in meters between two flight levels
	
		
		#fixed flight intentions parameters
		#if any(a not in self.A for a, k, y, delay in fixedIntentions):
		#	raise ValueError("Fixed flight is not presented in the list of flights!")
		#if any(k not in self.K for a, k, y, delay in fixedIntentions):
		#	raise ValueError("Fixed ftrajector doesn't exist!")
		#if any(y <= 0 or y > self.nbFL for a, k, y, delay in fixedIntentions):
		#	raise ValueError("Inexisting fixed flight level!")
		#if any(delay < 0 or delay > self.maxStep for a, k, y, delay in fixedIntentions):
		#	raise ValueError("Wrong fixed delay!")
		self.Kfix = []
		self.Afix = {}
		for a, k, y, delay in fixedIntentions:
			self.Kfix.append(k)
			self.Afix[a] = (y, delay)
		
		self.FLfix = []
		if fixedLevels:
			#if any(a not in self.A or l <= 0 or l > self.nbFL for a, l in fixedLevels):
			#	raise ValueError("Either flight or level is out of the range for fixed level flights!")
			self.FLfix = fixedLevels
		
class ParamLevelChoice:
	def __init__(self, model, A, priorities, conflicts, interactions):
		self.nbflights = len(A)					#number of flight instances
		self.FL = list(range(1, model.nb_FL+1))	#set of flight levels
		self.A = A								#set of flight intentions
		
		#if any(a not in priorities for a in self.A):
		#	raise ValueError("Priority must be defined for every flight intention!")
		self.priorities = priorities			#flight intention priorities
		
		#conflicts is a list of pairs of flight intentions that must not be assigned to the same level
		#if any(i not in self.A or j not in self.A for i,j in conflicts):
		#	raise ValueError("Non existing fights in conflict list!")
		self.I = [(i, j) for (i, j) in conflicts if i < j] #conflicts must be symetric if (i,j) in interactions => (j,i) is too
		
		#interactions is dict of pairs of flight intentions with magnitude of their interaction that we penalise
		#if any(i not in self.A or j not in self.A for i,j in interactions):
		#	raise ValueError("Non existing fights in interaction list!")
		self.P = [(i, j) for (i, j) in interactions if i < j] #interactions must be symetric if (i,j) in interactions => (j,i) is too
		self.interactions = interactions
		
	def __str__(self):
		rez = "FL assignement parameter\n"
		rez += "Number of flights: " + str(self.nbflights) + "\n"
		rez += "Flights: " + str(self.A) + "\n"
		rez += "Incompatible pairs: " + str(self.I) + "\n"
		rez += "Penalised pairs: " + str(self.interactions)
		return rez
'''
def readDat(filename):
	print("Loading parameters from ", filename)
	d = []
	mon_vol = []
	k1 = []
	k2 = []
	t1 = []
	t2 = []
	sep12 = []
	sep21 = []
	p = [None] * 6
	params = {}
	with open(filename) as f:
		for line in f:
			line = line.strip()
			if line: 
				line = [x for x in re.split(";| |:=", line) if x]

				#single parameters
				if line[0] == "param":
					params[line[1]] = int(line[2])
				
				#list parameters
				elif line[0] == "param:":
					for i in range(1,len(line)):
						if line[i] == "d": p[i-1] = d
						if line[i] == "mon_vol": p[i-1] = mon_vol
						if line[i] == "k1": p[i-1] = k1
						if line[i] == "k2": p[i-1] = k2
						if line[i] == "t1": p[i-1] = t1
						if line[i] == "t2": p[i-1] = t2
						if line[i] == "sep12": p[i-1] = sep12
						if line[i] == "sep21": p[i-1] = sep21
				#values of the list parameters
				else:
					for i in range(1,len(line)):
						try:
							p[i-1].append(int(line[i]))
						except:
							p[i-1].append(float(line[i]))
	
	return Param(list(range(params["nbflights"])), [(k, dur, mon_vol[k]) for k, dur in enumerate(d)], params["maxDelay"], [params["nbPtHor"], params["nbPtClmb"], params["nbPtDesc"], params["nbPtDep"], params["nbPtArr"]], k1, k2, t1, t2, sep12, sep21)

def addRandomFixFligth(filename, nbAfix=0):
	param = readDat(filename)
	if nbAfix > param.nbflights:
		nbAfix = param.nbflights
		
	#randomly chose selected # of flight and set their traj, fl and delay
	for i in range(nbAfix):
		a = rd.choice(param.A)
		while a in param.Afix:
			a = rd.choice(param.A)
		ks = []
		for k in param.K:
			if param.mon_vol[k] == a: ks.append(k)
		k = rd.choice(ks)
		y = rd.randint(1, param.nbFL)
		delay = rd.randint(0, param.maxStep)
		
		param.Kfix.append(k)
		param.Afix[a] = (y, delay)
	print("param.Afix = ", param.Afix)
	print("param.Kfix = ", param.Kfix)
	
	return param

def createRandomParamLevelChoice(nbflights):
	A = list(range(nbflights))
	interactions = []
	for i in range(rd.randint(0, 2*nbflights)):
		a1 = rd.choice(A)
		a2 = rd.choice(A)
		if a1 != a2 and (a1,a2) not in interactions and (a2,a1) not in interactions:
			interactions.append((a1,a2))
	priorities = {a: rd.randint(1, 4) for a in A}
	return ParamLevelChoice(A, priorities, interactions)


def levelChoiceParamGenerator(filename):
	param = readDat(filename)
	
	#let's find the shortest traj for every flight intention
	shortest = {}
	for k in param.K:
		a = param.mon_vol[k]
		
		if not a in shortest:
			shortest[a] = (-1, Decimal('Inf'))
		
		if shortest[a][1] > param.d[k]:
			shortest[a] = (k, param.d[k])
		
	interactions = set([])
	for i, k1 in enumerate(param.k1):
		k2 = param.k2[i]
		a1 = param.mon_vol[k1]
		a2 = param.mon_vol[k2]
		if shortest[a1][0] == k1 and shortest[a2][0] == k2:
			interactions.add((a1, a2))
		
	return param, ParamLevelChoice(param.A, interactions)
'''
