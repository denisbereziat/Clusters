import gurobipy as gb
import Param
import sys
import itertools as it

W = {1: 1, 2: 1.5, 3: 2, 4: 3} #weights defining importance of the flight instancies based on their priority
C = 0.01 #coeficient (scalling factor) of the flight level importance in objective function

class ProblemGlobal:
	def __init__(self, param, name = "MyProblem"):
		self.param = param
		
		print("Creating model for ", name)	
		self.model = gb.Model(name)
		self.createVars()
		self.model.update 					# Integrate new variables
		self.createObjectiveFunction()
		self.createConstraints()
		self.model.update
		
	def solve(self):
		self.model.optimize()
		return self.model.Status == gb.GRB.OPTIMAL
	
	def createVars(self):
		self.x = self.model.addVars(self.param.K, vtype=gb.GRB.BINARY, name="x")							#assignment variables =1 whether horizontal trajectory i is used
		self.y = self.model.addVars(self.param.A, vtype=gb.GRB.INTEGER, lb=1, ub=self.param.nbFL, name="y") #flight level choice for flight intention i
		self.delay = self.model.addVars(self.param.A, lb=0, ub=self.param.maxDelay, name="delay")			#(real) delay of flight intention i

		self.same_fl = self.model.addVars(self.param.AInter, vtype=gb.GRB.BINARY, name="same_fl")			#auxiliary =1 if two flight intentions i and j are assigned to same flight level
		self.lower_fl = self.model.addVars(self.param.AInter, vtype=gb.GRB.BINARY, name="lower_fl")			#auxiliary =1 if flight level of j flight intention is lower than flihgt level of flight intention i
		self.higher_fl = self.model.addVars(self.param.AInter, vtype=gb.GRB.BINARY, name="higher_fl")		#auxiliary =1 if flight level of j flight intention is higher than flihgt level of flight intention i

		self.z = self.model.addVars(self.param.P, vtype=gb.GRB.BINARY, name="z")							#auxiliary =1 if for given intersecting point order is k1 then k2


	def createObjectiveFunction(self):
		self.model.setObjective(sum(W[self.param.priorities[self.param.mon_vol[k]]]*self.param.d[k]*self.x[k] for k in self.param.K) + sum(W[self.param.priorities[a]]*(self.delay[a] + 2*(self.param.tSeedUp + self.y[a]*self.param.dFL/self.param.vVert)) for a in self.param.A), gb.GRB.MINIMIZE) 

	
	def createConstraints(self):
		self.model.addConstrs((sum(self.x[k] for k in self.param.K if self.param.mon_vol[k]==i) >= 1 for i in self.param.A), "covering")

		self.model.addConstrs((-self.param.FLmax*self.lower_fl[i,j] + self.param.delta*self.higher_fl[i,j] <= self.y[i] - self.y[j] for i, j in self.param.AInter if i < j), "assuring_lower_fl")
		self.model.addConstrs((self.y[i] - self.y[j] <= -self.param.delta*self.lower_fl[i,j] + self.param.FLmax*self.higher_fl[i,j] for i, j in self.param.AInter if i < j), "assuring_higher_fl")
		self.model.addConstrs((self.lower_fl[i,j] + self.higher_fl[i,j] + self.same_fl[i,j] == 1 for i, j in self.param.AInter if i < j), "assuring_same_fl")
		self.model.addConstrs((self.lower_fl[i,j] == self.higher_fl[j,i] for i, j in self.param.AInter if i < j), "assuring_lower_fl_appendix")
		self.model.addConstrs((self.higher_fl[i,j] == self.lower_fl[j,i] for i, j in self.param.AInter if i < j), "assuring_higher_fl_appendix")
		self.model.addConstrs((self.same_fl[i,j] == self.same_fl[j,i] for i, j in self.param.AInter if i < j), "assuring_same_fl_appendix")

		#conflicts between to trajectories at same level
		self.model.addConstrs(((self.param.t2[p] + self.delay[self.param.mon_vol[self.param.k2[p]]]) - (self.param.t1[p] + self.delay[self.param.mon_vol[self.param.k1[p]]]) >=  self.param.sep12[p] - (self.param.sep12[p] - self.param.t2[p] + self.param.t1[p] + self.param.maxDelay)*(4 - self.z[p] - self.x[self.param.k1[p]] - self.x[self.param.k2[p]] - self.same_fl[self.param.mon_vol[self.param.k1[p]],self.param.mon_vol[self.param.k2[p]]]) for p in self.param.Phor), "hor_conflict1")
		self.model.addConstrs(((self.param.t2[p] + self.delay[self.param.mon_vol[self.param.k2[p]]]) - (self.param.t1[p] + self.delay[self.param.mon_vol[self.param.k1[p]]]) <= -self.param.sep21[p] + (self.param.sep21[p] + self.param.t2[p] - self.param.t1[p] + self.param.maxDelay)*(3 + self.z[p] - self.x[self.param.k1[p]] - self.x[self.param.k2[p]] - self.same_fl[self.param.mon_vol[self.param.k1[p]],self.param.mon_vol[self.param.k2[p]]]) for p in self.param.Phor), "hor_conflict2")

		#it is assumed that k1 is climbing trajectory
		self.model.addConstrs(((self.param.t2[p] + self.delay[self.param.mon_vol[self.param.k2[p]]] + self.param.tSeedUp/2) - (self.param.t1[p] + self.delay[self.param.mon_vol[self.param.k1[p]]]) >=  self.param.sep12[p] - (self.param.sep12[p] - self.param.t2[p] - self.param.tSeedUp/2 + self.param.t1[p] + self.param.maxDelay)*(4 - self.z[p] - self.x[self.param.k1[p]] - self.x[self.param.k2[p]] - self.higher_fl[self.param.mon_vol[self.param.k1[p]],self.param.mon_vol[self.param.k2[p]]]) for p in self.param.Pclmb), "clmb_conflict1")
		self.model.addConstrs(((self.param.t2[p] + self.delay[self.param.mon_vol[self.param.k2[p]]] + self.param.tSeedUp/2) - (self.param.t1[p] + self.delay[self.param.mon_vol[self.param.k1[p]]]) <= -self.param.sep21[p] + (self.param.sep21[p] + self.param.t2[p] + self.param.tSeedUp/2 + self.param.maxDelay - self.param.t1[p])*(3 + self.z[p] - self.x[self.param.k1[p]] - self.x[self.param.k2[p]] - self.higher_fl[self.param.mon_vol[self.param.k1[p]],self.param.mon_vol[self.param.k2[p]]]) for p in self.param.Pclmb), "clmb_conflict2")

		#it is assumed that k1 is descending trajectory
		self.model.addConstrs(((self.param.t2[p] + self.delay[self.param.mon_vol[self.param.k2[p]]]) - (self.param.t1[p] + self.delay[self.param.mon_vol[self.param.k1[p]]] + self.param.tSeedUp/2 + 2*(self.y[self.param.mon_vol[self.param.k1[p]]] - self.y[self.param.mon_vol[self.param.k2[p]]])*self.param.dFL/self.param.vVert) >=  self.param.sep12[p] - (self.param.sep12[p] - self.param.t2[p] + self.param.t1[p] + self.param.maxDelay + self.param.tSeedUp/2 + 2*self.param.deltaFLmax/self.param.vVert)*(4 - self.z[p] - self.x[self.param.k1[p]] - self.x[self.param.k2[p]] - self.higher_fl[self.param.mon_vol[self.param.k1[p]],self.param.mon_vol[self.param.k2[p]]]) for p in self.param.Pdesc), "desc_conflict1")
		self.model.addConstrs(((self.param.t2[p] + self.delay[self.param.mon_vol[self.param.k2[p]]]) - (self.param.t1[p] + self.delay[self.param.mon_vol[self.param.k1[p]]] + self.param.tSeedUp/2 + 2*(self.y[self.param.mon_vol[self.param.k1[p]]] - self.y[self.param.mon_vol[self.param.k2[p]]])*self.param.dFL/self.param.vVert) <= -self.param.sep21[p] + (self.param.sep21[p] + self.param.t2[p] + self.param.maxDelay - self.param.t1[p] - self.param.tSeedUp/2 - 2*self.param.deltaFLmin/self.param.vVert)*(3 + self.z[p] - self.x[self.param.k1[p]] - self.x[self.param.k2[p]] - self.higher_fl[self.param.mon_vol[self.param.k1[p]],self.param.mon_vol[self.param.k2[p]]]) for p in self.param.Pdesc), "desc_conflict2")

		#conflicts between two departures
		self.model.addConstrs(((self.param.t2[p] + self.delay[self.param.mon_vol[self.param.k2[p]]]) - (self.param.t1[p] + self.delay[self.param.mon_vol[self.param.k1[p]]]) >=  self.param.sep12[p] - (self.param.sep12[p] - self.param.t2[p] + self.param.t1[p] + self.param.maxDelay)*(3 - self.z[p] - self.x[self.param.k1[p]] - self.x[self.param.k2[p]]) for p in self.param.Pdep), "dep_conflict1")
		self.model.addConstrs(((self.param.t2[p] + self.delay[self.param.mon_vol[self.param.k2[p]]]) - (self.param.t1[p] + self.delay[self.param.mon_vol[self.param.k1[p]]]) <= -self.param.sep21[p] + (self.param.sep21[p] + self.param.t2[p] + self.param.maxDelay - self.param.t1[p])*(2 + self.z[p] - self.x[self.param.k1[p]] - self.x[self.param.k2[p]]) for p in self.param.Pdep), "dep_conflict2")

		#conflicts between two arrivals
		self.model.addConstrs(((self.param.t2[p] + self.delay[self.param.mon_vol[self.param.k2[p]]]) - (self.param.t1[p] + self.delay[self.param.mon_vol[self.param.k1[p]]] + 2*(self.y[self.param.mon_vol[self.param.k1[p]]] - self.y[self.param.mon_vol[self.param.k2[p]]])*self.param.dFL/self.param.vVert) >=  self.param.sep12[p] - (self.param.sep12[p] - self.param.t2[p] + self.param.t1[p] + self.param.maxDelay + 2*self.param.deltaFLmax/self.param.vVert)*(3 - self.z[p] - self.x[self.param.k1[p]] - self.x[self.param.k2[p]]) for p in self.param.Parr), "arr_conflict1")
		self.model.addConstrs(((self.param.t2[p] + self.delay[self.param.mon_vol[self.param.k2[p]]]) - (self.param.t1[p] + self.delay[self.param.mon_vol[self.param.k1[p]]] + 2*(self.y[self.param.mon_vol[self.param.k1[p]]] - self.y[self.param.mon_vol[self.param.k2[p]]])*self.param.dFL/self.param.vVert) <= -self.param.sep21[p] + (self.param.sep21[p] + self.param.t2[p] + self.param.maxDelay - self.param.t1[p] - 2*self.param.deltaFLmin/self.param.vVert)*(2 + self.z[p] - self.x[self.param.k1[p]] - self.x[self.param.k2[p]]) for p in self.param.Parr), "arr_conflict2")

		#fixed flights constraints
		for k in self.param.Kfix:
			self.x[k].lb = 1
			self.x[k].ub = 1
		for a, val in self.param.Afix.items():
			self.y[a].lb = val[0]
			self.y[a].ub = val[0]
			self.delay[a].lb = val[1]
			self.delay[a].ub = val[1]
		#fixed level constraints
		for a, l in self.param.FLfix:
			self.y[a].lb = l
			self.y[a].ub = l

	def setPartialSolution(self, x_val, y_val, delay_val, same_fl_val, lower_fl_val, higher_fl_val):
		for k in self.param.K:
			if k in x_val:
				self.x[k].Start = x_val[k]
		for a in self.param.A:
			if a in y_val: # and a in delay_val that is always the case
				self.y[a].Start = y_val[a]
				self.delay[a].Start = delay_val[a]
		for a_pair in self.param.AInter:
			if a_pair in same_fl_val: # other should have it then
				self.same_fl[a_pair].Start = same_fl_val[a_pair]
				self.lower_fl[a_pair].Start = lower_fl_val[a_pair]
				self.higher_fl[a_pair].Start = higher_fl_val[a_pair]
	
	def printSolution(self):
		print("Obj: ", self.model.objVal)
		#for every flight print trajectory selected, flight level and delay
		t = {}
		for k in self.param.K:
			if self.x[k].x == 1:
				t[self.param.mon_vol[k]] = k

		print(" FN,   traj_id,   fl,   delay")
		for a in self.param.A:
			print(a, "   ", t[a], "   ", int(self.y[a].x), "   ", int(self.delay[a].x))

class ProblemGlobalRelaxed:
	def __init__(self, param, name = "MyProblemRelaxed"):
		self.param = param
		
		print("Creating model for ", name)	
		self.model = gb.Model(name)
		self.createVars()
		self.model.update 					# Integrate new variables
		self.createObjectiveFunction()
		self.createConstraints()
		self.model.update
		
	def solve(self):
		self.model.optimize()
		return self.model.Status == gb.GRB.SUBOPTIMAL or self.model.Status == gb.GRB.OPTIMAL
	
	def createVars(self):
		self.x = self.model.addVars(self.param.K, vtype=gb.GRB.BINARY, name="x")							#assignment variables =1 whether horizontal trajectory i is used
		self.y = self.model.addVars(self.param.A, vtype=gb.GRB.INTEGER, lb=1, ub=self.param.nbFL, name="y") #flight level choice for flight intention i
		self.delay = self.model.addVars(self.param.A, lb=0, ub=self.param.maxDelay, name="delay")			#(real) delay of flight intention i

		self.same_fl = self.model.addVars(self.param.AInter, vtype=gb.GRB.BINARY, name="same_fl")			#auxiliary =1 if two flight intentions i and j are assigned to same flight level
		self.lower_fl = self.model.addVars(self.param.AInter, vtype=gb.GRB.BINARY, name="lower_fl")			#auxiliary =1 if flight level of j flight intention is lower than flihgt level of flight intention i
		self.higher_fl = self.model.addVars(self.param.AInter, vtype=gb.GRB.BINARY, name="higher_fl")		#auxiliary =1 if flight level of j flight intention is higher than flihgt level of flight intention i

		self.z = self.model.addVars(self.param.P, vtype=gb.GRB.BINARY, name="z")							#auxiliary =1 if for given intersecting point order is k1 then k2
		self.v = self.model.addVars(self.param.P, lb=0, name="v")											#intersection constraints violation variable used for relaxation; =0 when separation are respected


	def createObjectiveFunction(self):
		self.model.setObjective(10*sum(self.v[p] for p in self.param.P) + sum(W[self.param.priorities[self.param.mon_vol[k]]]*self.param.d[k]*self.x[k] for k in self.param.K) + sum(W[self.param.priorities[a]]*(self.delay[a] + 2*(self.param.tSeedUp + self.y[a]*self.param.dFL/self.param.vVert)) for a in self.param.A), gb.GRB.MINIMIZE) 

	
	def createConstraints(self):
		self.model.addConstrs((sum(self.x[k] for k in self.param.K if self.param.mon_vol[k]==i) >= 1 for i in self.param.A), "covering")

		self.model.addConstrs((-self.param.FLmax*self.lower_fl[i,j] + self.param.delta*self.higher_fl[i,j] <= self.y[i] - self.y[j] for i, j in self.param.AInter if i < j), "assuring_lower_fl")
		self.model.addConstrs((self.y[i] - self.y[j] <= -self.param.delta*self.lower_fl[i,j] + self.param.FLmax*self.higher_fl[i,j] for i, j in self.param.AInter if i < j), "assuring_higher_fl")
		self.model.addConstrs((self.lower_fl[i,j] + self.higher_fl[i,j] + self.same_fl[i,j] == 1 for i, j in self.param.AInter if i < j), "assuring_same_fl")
		self.model.addConstrs((self.lower_fl[i,j] == self.higher_fl[j,i] for i, j in self.param.AInter if i < j), "assuring_lower_fl_appendix")
		self.model.addConstrs((self.higher_fl[i,j] == self.lower_fl[j,i] for i, j in self.param.AInter if i < j), "assuring_higher_fl_appendix")
		self.model.addConstrs((self.same_fl[i,j] == self.same_fl[j,i] for i, j in self.param.AInter if i < j), "assuring_same_fl_appendix")

		#conflicts between to trajectories at same level
		self.model.addConstrs((self.v[p] >= self.param.sep12[p] - (self.param.t2[p] + self.delay[self.param.mon_vol[self.param.k2[p]]]) + (self.param.t1[p] + self.delay[self.param.mon_vol[self.param.k1[p]]]) - (self.param.sep12[p] - self.param.t2[p] + self.param.t1[p] + self.param.maxDelay)*(4 - self.z[p] - self.x[self.param.k1[p]] - self.x[self.param.k2[p]] - self.same_fl[self.param.mon_vol[self.param.k1[p]],self.param.mon_vol[self.param.k2[p]]]) for p in self.param.Phor), "hor_conflict1")
		self.model.addConstrs((self.param.sep21[p] + (self.param.t2[p] + self.delay[self.param.mon_vol[self.param.k2[p]]]) - (self.param.t1[p] + self.delay[self.param.mon_vol[self.param.k1[p]]]) - (self.param.sep21[p] + self.param.t2[p] - self.param.t1[p] + self.param.maxDelay)*(3 + self.z[p] - self.x[self.param.k1[p]] - self.x[self.param.k2[p]] - self.same_fl[self.param.mon_vol[self.param.k1[p]],self.param.mon_vol[self.param.k2[p]]]) <= self.v[p] for p in self.param.Phor), "hor_conflict2")

		#it is assumed that k1 is climbing trajectory
		self.model.addConstrs((self.v[p] >= self.param.sep12[p] - (self.param.t2[p] + self.delay[self.param.mon_vol[self.param.k2[p]]] + self.param.tSeedUp/2) + (self.param.t1[p] + self.delay[self.param.mon_vol[self.param.k1[p]]]) - (self.param.sep12[p] - self.param.t2[p] - self.param.tSeedUp/2 + self.param.t1[p] + self.param.maxDelay)*(4 - self.z[p] - self.x[self.param.k1[p]] - self.x[self.param.k2[p]] - self.higher_fl[self.param.mon_vol[self.param.k1[p]],self.param.mon_vol[self.param.k2[p]]]) for p in self.param.Pclmb), "clmb_conflict1")
		self.model.addConstrs((self.param.sep21[p] + (self.param.t2[p] + self.delay[self.param.mon_vol[self.param.k2[p]]] + self.param.tSeedUp/2) - (self.param.t1[p] + self.delay[self.param.mon_vol[self.param.k1[p]]]) - (self.param.sep21[p] + self.param.t2[p] + self.param.tSeedUp/2 + self.param.maxDelay - self.param.t1[p])*(3 + self.z[p] - self.x[self.param.k1[p]] - self.x[self.param.k2[p]] - self.higher_fl[self.param.mon_vol[self.param.k1[p]],self.param.mon_vol[self.param.k2[p]]]) <= self.v[p] for p in self.param.Pclmb), "clmb_conflict2")

		#it is assumed that k1 is descending trajectory
		self.model.addConstrs((self.v[p] >= self.param.sep12[p] - (self.param.t2[p] + self.delay[self.param.mon_vol[self.param.k2[p]]]) + (self.param.t1[p] + self.delay[self.param.mon_vol[self.param.k1[p]]] + self.param.tSeedUp/2 + 2*(self.y[self.param.mon_vol[self.param.k1[p]]] - self.y[self.param.mon_vol[self.param.k2[p]]])*self.param.dFL/self.param.vVert) - (self.param.sep12[p] - self.param.t2[p] + self.param.t1[p] + self.param.maxDelay + self.param.tSeedUp/2 + 2*self.param.deltaFLmax/self.param.vVert)*(4 - self.z[p] - self.x[self.param.k1[p]] - self.x[self.param.k2[p]] - self.higher_fl[self.param.mon_vol[self.param.k1[p]],self.param.mon_vol[self.param.k2[p]]]) for p in self.param.Pdesc), "desc_conflict1")
		self.model.addConstrs((self.param.sep21[p] + (self.param.t2[p] + self.delay[self.param.mon_vol[self.param.k2[p]]]) - (self.param.t1[p] + self.delay[self.param.mon_vol[self.param.k1[p]]] + self.param.tSeedUp/2 + 2*(self.y[self.param.mon_vol[self.param.k1[p]]] - self.y[self.param.mon_vol[self.param.k2[p]]])*self.param.dFL/self.param.vVert) - (self.param.sep21[p] + self.param.t2[p] + self.param.maxDelay - self.param.t1[p] - self.param.tSeedUp/2 - 2*self.param.deltaFLmin/self.param.vVert)*(3 + self.z[p] - self.x[self.param.k1[p]] - self.x[self.param.k2[p]] - self.higher_fl[self.param.mon_vol[self.param.k1[p]],self.param.mon_vol[self.param.k2[p]]]) <= self.v[p] for p in self.param.Pdesc), "desc_conflict2")

		#conflicts between two departures
		self.model.addConstrs((self.v[p] >= self.param.sep12[p] - (self.param.t2[p] + self.delay[self.param.mon_vol[self.param.k2[p]]]) + (self.param.t1[p] + self.delay[self.param.mon_vol[self.param.k1[p]]]) - (self.param.sep12[p] - self.param.t2[p] + self.param.t1[p] + self.param.maxDelay)*(3 - self.z[p] - self.x[self.param.k1[p]] - self.x[self.param.k2[p]]) for p in self.param.Pdep), "dep_conflict1")
		self.model.addConstrs((self.param.sep21[p] + (self.param.t2[p] + self.delay[self.param.mon_vol[self.param.k2[p]]]) - (self.param.t1[p] + self.delay[self.param.mon_vol[self.param.k1[p]]]) - (self.param.sep21[p] + self.param.t2[p] + self.param.maxDelay - self.param.t1[p])*(2 + self.z[p] - self.x[self.param.k1[p]] - self.x[self.param.k2[p]]) <= self.v[p] for p in self.param.Pdep), "dep_conflict2")

		#conflicts between two arrivals
		self.model.addConstrs((self.v[p] >= self.param.sep12[p] - (self.param.t2[p] + self.delay[self.param.mon_vol[self.param.k2[p]]]) + (self.param.t1[p] + self.delay[self.param.mon_vol[self.param.k1[p]]] + 2*(self.y[self.param.mon_vol[self.param.k1[p]]] - self.y[self.param.mon_vol[self.param.k2[p]]])*self.param.dFL/self.param.vVert) - (self.param.sep12[p] - self.param.t2[p] + self.param.t1[p] + self.param.maxDelay + 2*self.param.deltaFLmax/self.param.vVert)*(3 - self.z[p] - self.x[self.param.k1[p]] - self.x[self.param.k2[p]]) for p in self.param.Parr), "arr_conflict1")
		self.model.addConstrs((self.param.sep21[p] + (self.param.t2[p] + self.delay[self.param.mon_vol[self.param.k2[p]]]) - (self.param.t1[p] + self.delay[self.param.mon_vol[self.param.k1[p]]] + 2*(self.y[self.param.mon_vol[self.param.k1[p]]] - self.y[self.param.mon_vol[self.param.k2[p]]])*self.param.dFL/self.param.vVert) - (self.param.sep21[p] + self.param.t2[p] + self.param.maxDelay - self.param.t1[p] - 2*self.param.deltaFLmin/self.param.vVert)*(2 + self.z[p] - self.x[self.param.k1[p]] - self.x[self.param.k2[p]]) <= self.v[p] for p in self.param.Parr), "arr_conflict2")

		#fixed flights constraints
		for k in self.param.Kfix:
			self.x[k].lb = 1
			self.x[k].ub = 1
		for a, val in self.param.Afix.items():
			self.y[a].lb = val[0]
			self.y[a].ub = val[0]
			self.delay[a].lb = val[1]
			self.delay[a].ub = val[1]
		#fixed level constraints
		for a, l in self.param.FLfix:
			self.y[a].lb = l
			self.y[a].ub = l

	def setPartialSolution(self, x_val, y_val, delay_val, same_fl_val, lower_fl_val, higher_fl_val):
		for k in self.param.K:
			if k in x_val:
				self.x[k].Start = x_val[k]
		for a in self.param.A:
			if a in y_val: # and a in delay_val that is always the case
				self.y[a].Start = y_val[a]
				self.delay[a].Start = delay_val[a]
		for a_pair in self.param.AInter:
			if a_pair in same_fl_val: # other should have it then
				self.same_fl[a_pair].Start = same_fl_val[a_pair]
				self.lower_fl[a_pair].Start = lower_fl_val[a_pair]
				self.higher_fl[a_pair].Start = higher_fl_val[a_pair]
	
	def printSolution(self):
		violation = sum(self.v[p] for p in self.param.P)
		print("Trajectory duration: ", self.model.objVal - violation)
		print("Violation: ", violation)
		#for every flight print trajectory selected, flight level and delay
		t = {}
		for k in self.param.K:
			if self.x[k].x == 1:
				t[self.param.mon_vol[k]] = k

		print(" FN,   traj_id,   fl,   delay")
		for a in self.param.A:
			print(a, "   ", t[a], "   ", int(self.y[a].x), "   ", int(self.delay[a].x))


class ProblemLevelChoice:
	def __init__(self, param, withcost = True, name = "MyProblem"):
		self.param = param
		
		print("Creating model for ", name)
		#print(param)	
		self.model = gb.Model(name)
		self.createVars()
		self.model.update 					# Integrate new variables
		self.createObjectiveFunction(withcost)
		self.createConstraints()
		self.model.update
		
		# Objects to use inside callbacks
		self.model._param = param
		self.model._x = self.x
		self.model._z = self.z
		self.model._x_vals = None
		self.model._z_vals = None
		
	def solve(self):
		#self.model.optimize(ProblemLevelChoice.mycallback)
		self.model.optimize()
	
	def createVars(self):
		self.x = self.model.addVars(self.param.A, self.param.FL, vtype=gb.GRB.BINARY, name="x")	#assignment variables =1 when flight is assignet to fl
		self.z = self.model.addVars(self.param.P, vtype=gb.GRB.BINARY, name="z")				#auxiliary =1 if two flight intentions i and j are assigned to same flight level (assured only when penalized)

	def createObjectiveFunction(self, withcost):
		if withcost:
			self.model.setObjective(sum(self.param.interactions[(i,j)]*self.z[i,j] for i,j in self.param.P) + C*sum(fl*sum(W[self.param.priorities[a]]*self.x[a,fl] for a in self.param.A) for fl in self.param.FL), gb.GRB.MINIMIZE)
		else:
			self.model.setObjective(sum(self.param.interactions[(i,j)]*self.z[i,j] for i,j in self.param.P), gb.GRB.MINIMIZE)

	
	def createConstraints(self):
		self.model.addConstrs((sum(self.x[a,fl] for fl in self.param.FL) == 1 for a in self.param.A), "assignement_constraint")
		self.model.addConstrs((self.x[a1,fl] <= 1 - self.x[a2,fl] for ((a1, a2), fl) in it.product(self.param.I, self.param.FL)), "incompatible_flight")
		self.model.addConstrs((self.x[a1,fl] + self.x[a2,fl] - 1 <= self.z[a1,a2] for ((a1, a2), fl) in it.product(self.param.P, self.param.FL)), "linking_constraint")

	def printSolution(self):
		print("Obj: ", self.model.objVal)
		print("Remaining interactions number: ", sum(self.z[i,j].x for i,j in self.param.P))
		print("Remaining interactions magnitude: ", sum(self.param.interactions[(i,j)]*self.z[i,j].x for i,j in self.param.P))
		#for every flight print chosen flight level
		FLs = {}
		level_use = {fl: 0 for fl in self.param.FL}
		for a in self.param.A:
			for fl in self.param.FL:
				if round(self.x[a,fl].x) == 1:
					level_use[fl] += 1
					FLs[a] = fl
					print("Flight ", a, "\t", fl)
		print("Level usage: ", level_use)
		for (a1, a2) in self.param.I:
			if FLs[a1] == FLs[a2]:
				print("Incompatible flights ", a1, a2," assigned at same level")
		for (a1, a2) in self.param.P:
			if round(self.z[a1, a2].x) == 1:
				print("z=1 for Interacting flights ", a1, a2, " with ", self.param.interactions[a1, a2], "overlap")
			if FLs[a1] == FLs[a2]:
				print("Interacting flights ", a1, a2, " with ", self.param.interactions[a1, a2], "overlap assigned at same level ", self.z[a1, a2].x)

	def mycallback(model, where):
		'''if where == gb.GRB.Callback.PRESOLVE:
			interactions = {a: 0 for a in model._param.A}
			for pair in model._param.AInter:
				interactions[pair[0]] += 1
			interactions = sorted(interactions.items(), key=lambda item: -item[1])
			print(interactions)
			sys.exit(0)'''
		
		if where == gb.GRB.Callback.MIPSOL:
			#print("improving ", model.cbGet(gb.GRB.Callback.MIPSOL_OBJ))
			# computed number of flight instances per level
			x = model.cbGetSolution(model._x)
			#print("Old assignement: ", x)
			available_FL = sorted(list(model._param.FL))
			level_use = {}
			for fl in available_FL:
				level_use[fl] = sum(round(x[a,fl]) for a in model._param.A)
			#print("current level usage ", level_use)
			
			er = any(level_use[available_FL[index]]<level_use[available_FL[index+1]] for index in range(len(available_FL)-1))
			if er:
				level_use = sorted(level_use.keys(), key=lambda k: -level_use[k])
				#print("Sorted levels per usage ", level_use)
				level_use = {level: index for index, level in enumerate(level_use)}
				#print("Old level: index of a new level ", level_use)
				model._x_vals = {}
				for a in model._param.A:
					a_level = False
					for fl in available_FL:
						model._x_vals[a,fl] = 0 					#default var value
						if round(x[a,fl]) == 1: 					#it is his ancien level
							a_level = available_FL[level_use[fl]] 	#note his new level
					model._x_vals[a,a_level] = 1					#set his new level
				#print("New assignement: ", model._x_vals)
				model._z_vals = model.cbGetSolution(model._z) #level assignment (hence z var) doesn't change
				
		if where == gb.GRB.Callback.MIPNODE and model._x_vals is not None:
			## inject heuristic solution
			model.cbSetSolution(model._z, model._z_vals)
			model.cbSetSolution(model._x, model._x_vals)
			model._x_vals = None
			
			model.cbUseSolution()
