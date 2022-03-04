#/*PARAMETERS*/ 
import re
import random as rd
from decimal import Decimal

nbFL = 16;        					#number of flight levels
aVert = 3.5;						#vertical acceleration in m/s^2
vVert = 5;							#vertical seed in m/s
dFL = 9.14;							#vertical separation between two flight levels in m = 30ft
dSeedUp = vVert*vVert/(2*aVert);	#distance needed to acceleate from 0 to vVert with acceleration aVert and vv
tSeedUp = vVert/aVert;				#time needed to acceleate from 0 to vVert with acceleration aVert and vv

delayStep = 10;						#delay step duration; total delay = number od delay steps (dec. var) * delay step


class Param:
	def __init__(self, model, A, K, maxDelay, nbPt, k1, k2, t1, t2, sep12, sep21, fixedIntentions = [], fixedLevels = []):
		self.nbFL = model.nb_FL
		self.aVert = aVert
		self.vVert = vVert
		self.dFL = dFL
		self.dSeedUp = dSeedUp
		self.tSeedUp = tSeedUp
		self.delayStep = delayStep
		
		self.nbflights = len(A)					#number of flight instances
		self.nbTrajs = len(K)					#total number of alternative trajectories
		
		self.maxDelay = maxDelay				#maximum delay
		self.maxStep = maxDelay // delayStep	#maximum number of steps

		self.A = A								#set of flight intentions
		if any(d < 0 for k, d, a in K):
			raise ValueError("Trajectory duration may not be negative!")
		if any(a not in self.A for k, d, a in K):
			raise ValueError("Flight of a trajectory must be in the list of flights!")
		self.K = []								#set of all horizontal trajectories
		self.d = {}								#duration of horizontal trajectory i
		self.mon_vol = {}						#flight intention to which belongs given horizontal trajectory 
		for k, d, a in K:
			self.K.append(k)
			self.d[k] = d
			self.mon_vol[k] = a

		if any(n < 0 for n in nbPt):
			raise ValueError("Number of intersecting points may not be negative!")
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
		
		
		if len(k1) != self.nbPtInter or len(k2) != self.nbPtInter or \
		   len(t1) != self.nbPtInter or len(t2) != self.nbPtInter or \
		   len(sep12) != self.nbPtInter or len(sep21) != self.nbPtInter:
			raise ValueError("Parameters k1, k2, t1, t2, sep12 and sep21 must be defined exactly for every interesecting point !")
		if any(i not in self.K for i in k1) or any(i not in self.K for i in k2):
			raise ValueError("Trajectory of an intersecting point must be in the list of trajectories!")
		if any(i < 0 for i in t1) or any(i < 0 for i in t2):
			raise ValueError("Arrival time at intersecting point may not be negative!")
		if any(i < 0 for i in sep12) or any(i < 0 for i in sep21):
			raise ValueError("Separation at intersecting point may not be negative!")
		self.k1 = k1 						#first trajectory of intersecting point
		self.k2 = k2						#second trajectory of intersecting point
		self.t1 = t1						#first trajectory planned time over intersecting point
		self.t2 = t2						#second trajectory planned time over intersecting point
		self.sep12 = sep12					#required time separation at intersecting point if first then second passes
		self.sep21 = sep21					#required time separation at intersecting point if second then first passes
											
											#set of pairs of "intersecting"" flight intnetions
		#self.AhorsDiag = [(i,j) for i in self.A for j in self.A if i != j]
		self.AInter = list(set((self.mon_vol[k1[p]], self.mon_vol[k2[p]]) for p in self.P).union(set((self.mon_vol[k2[p]], self.mon_vol[k1[p]]) for p in self.P)))
											
											#set of pairs of "intersecting"" trajectories
		#self.KhorsDiag = [(i,j) for i in self.K for j in self.K if mon_vol[i] != mon_vol[j]]
		self.KInterHor = list(set((k1[p], k2[p]) for p in self.Phor).union(set((k2[p], k1[p]) for p in self.Phor)))
		KInterClmb = set((k1[p], k2[p]) for p in self.Pclmb).union(set((k2[p], k1[p]) for p in self.Pclmb))
		self.KInterClmb = list(KInterClmb)
		KInterDesc = set((k1[p], k2[p]) for p in self.Pdesc).union(set((k2[p], k1[p]) for p in self.Pdesc))
		self.KInterDesc = list(KInterDesc)
		self.KInterEvol = list(KInterClmb.union(KInterDesc))
		self.KInterDep = list(set((k1[p], k2[p]) for p in self.Pdep).union(set((k2[p], k1[p]) for p in self.Pdep)))
		self.KInterArr = list(set((k1[p], k2[p]) for p in self.Parr).union(set((k2[p], k1[p]) for p in self.Parr)))
		
		#big M parameters
		self.FLmax = nbFL					#big M is set to max difference between two flight levels
		self.delta = 0.5					#small number that is less than difference between any different flight levels
		self.deltaFLmax = (nbFL - 1)*dFL	#maximum difference in meters between two flight levels
		self.deltaFLmin = -self.deltaFLmax	#minimum difference in meters between two flight levels
	
		
		#fixed flight intentions parameters
		if any(a not in self.A for a, k, y, delay in fixedIntentions):
			raise ValueError("Fixed flight is not presented in the list of flights!")
		if any(k not in self.K for a, k, y, delay in fixedIntentions):
			raise ValueError("Fixed ftrajector doesn't exist!")
		if any(y <= 0 or y > self.nbFL for a, k, y, delay in fixedIntentions):
			raise ValueError("Inexisting fixed flight level!")
		if any(delay < 0 or delay > self.maxStep for a, k, y, delay in fixedIntentions):
			raise ValueError("Wrong fixed delay!")
		self.Kfix = []
		self.Afix = {}
		for a, k, y, delay in fixedIntentions:
			self.Kfix.append(k)
			self.Afix[a] = (y, delay)
		
		self.FLfix = []
		if fixedLevels:
			if len(fixedIntentions) + len(fixedLevels) != self.nbflights:
				raise ValueError("All flights must have fixed levels!")
			if any(a not in self.A or l <= 0 or l > self.nbFLy for a, l in fixedLevels):
				raise ValueError("Either flight or level is out of the range for fixed level flights!")
			self.FLfix = fixedLevels
		
class ParamLevelChoice:
	def __init__(self, A, interactions):
		self.nbFL = nbFL
		self.nbflights = len(A)				#number of flight instances
		self.A = A							#set of flight intentions labeled from 1 to nbflights
		
		if any(i not in self.A or j not in self.A for i,j in interactions):
			raise ValueError("Non existing fights in interaction list!")
		self.AInter = list(set((i,j) for i,j in interactions).union(set((j,i) for i,j in interactions)))
											
		#big M parameters
		self.FLmax = nbFL					#big M is set to max difference between two flight levels
		self.delta = 0.5					#small number that is less than difference between any different flight levels
		self.deltaFLmax = (nbFL - 1)*dFL	#maximum difference in meters between two flight levels
		self.deltaFLmin = -self.deltaFLmax	#minimum difference in meters between two flight levels
	

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
	
def fixLevelParam():
	param = readDat("output.dat")
	
	param.Kfix = [10, 56, 72, 83, 126, 148, 151, 155, 164, 210, 220, 230, 235, 270, 309, 342, 346, 350, 387, 390, 397, 420, 486, 405]
	Afix = [2, 11, 14, 16, 25, 29, 30, 31, 32, 42, 44, 46, 47, 54, 61, 68, 69, 70, 77, 78, 79, 84, 97, 81]
	yfix = [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3]
	delayfix = [0, 0, 3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
	for i, a in enumerate(Afix):
		param.Afix[a] = (yfix[i], delayfix[i])
	
	param.FLfix = [(0, 1), (1, 1), (3, 1), (4, 1), (5, 1), (6, 1), (7, 1), (8, 1), (9, 1), (10, 1), (12, 1), (13, 1), (15, 1), (17, 1), (18, 1), (19, 1), (20, 1), (21, 1), (22, 1), (23, 1), (24, 1), (26, 1), (27, 1), (28, 1), (33, 1), (34, 1), (35, 1), (36, 1), (37, 1), (38, 1), (39, 1), (40, 1), (41, 1), (43, 1), (45, 1), (48, 1), (49, 1), (50, 1), (51, 1), (52, 1), (53, 1), (55, 1), (56, 1), (57, 1), (58, 1), (59, 1), (60, 1), (62, 1), (63, 1), (64, 1), (65, 1), (66, 1), (67, 1), (71, 1), (72, 1), (73, 1), (74, 1), (75, 1), (76, 1), (80, 1), (82, 1), (83, 1), (85, 1), (86, 1), (87, 1), (88, 1), (89, 1), (90, 1), (91, 1), (92, 1), (93, 1), (94, 1), (95, 1), (96, 1), (98, 1), (99, 1)]

	return param

def createRandomParamLevelChoice(nbflights):
	A = list(range(nbflights))
	interactions = []
	for i in range(rd.randint(0, 2*nbflights)):
		a1 = rd.choice(A)
		a2 = rd.choice(A)
		if a1 != a2 and (a1,a2) not in interactions and (a2,a1) not in interactions:
			interactions.append((a1,a2))
	print(interactions)
	return ParamLevelChoice(A, interactions)


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
