import math
import time
from debug import Debugger
from zss_debug_pb2 import Debug_Msgs

class bubble:
	x = 0
	y = 0
	new_label = 0 # indicate whether the bubble is newly created
	stop_label = 0
	rho = 0
	def __init__(self,x,y,label):
		self.x = x
		self.y = y
		self.new_label = label

	def SetRho(self,in_rho):
		self.rho = in_rho

	def SetStop(self):
		self.stop_label = 1

	def SetNew(self):
		self.new_label = 1


class ElasticBand:
	obs = []
	bubbles = []
	N = 0
	Kc = 10
	Kr = 0.5
	rho0 = 750
	fmin = 10
	min_dist = 400

	def __init__(self,vision,debug):
		self.vision = vision
		self.obs = vision.yellow_robot
		self.debugger = debug

	def __UpdateBubble(self,bubble_x,bubble_y):
		# read in bubbles
		self.bubbles = []
		for (x,y) in zip(bubble_x,bubble_y):
			self.bubbles.append(bubble(x,y,0))
		self.N = len(bubble_x)
		#print(bubble_x,bubble_y)
		#print("------\n")

	def MoveBubble(self,path_x,path_y):
		self.__UpdateBubble(path_x,path_y)
		#self.__UpdateObs()
		count = 0
		#start elatic process
		self.__UpdateRho()
		# while there is node that not stopped and the total iteration less than 40 times
		while self.__CheckNotStop() and count < 40:
			#print([r.rho for r in self.bubbles])
			for i in range(1,self.N-1):
				if self.bubbles[i].stop_label == True:
					continue
				else:
					ifx,ify = self.__CalInForce(i)
					rfx,rfy = self.__CalRelForce(i)
					fx = ifx+rfx
					fy = ify+rfy
					if fx < self.fmin and fx > -self.fmin:
						fx = 0
					if fy < self.fmin and fy > -self.fmin:
						fy = 0
					if fx == 0 and fy == 0:
						self.bubbles[i].SetStop()
						continue
					self.bubbles[i].x += self.bubbles[i].rho/1000*fx
					self.bubbles[i].y += self.bubbles[i].rho/1000*fy
			# go into the check process
			self.__UpdateRho()
			if count % 5 == 0:
				if self.__DeleteandCreate() == False:
					return [],[]
			count += 1
		print(count)
		package = Debug_Msgs()
		self.debugger.draw_points(package, [obj.x for obj in self.bubbles], [obj.y for obj in self.bubbles])
		for r in self.bubbles:
			self.debugger.draw_circle(package,r.x,r.y,r.rho)
		self.debugger.send(package)
		return [r.x for r in self.bubbles],[r.y for r in self.bubbles]

	def __DeleteandCreate(self):
		#first delete reluctant points
		i = 0
		while i < self.N-2:
			b = self.bubbles[i]
			bb = self.bubbles[i+1]
			tempd = math.sqrt((b.x-bb.x)**2+(b.y-bb.y)**2)
			if tempd < self.min_dist:
				del self.bubbles[i+1]
				self.N -= 1
			else:
				i = i+1
		#then create point if needed
		i = 0
		while i < self.N-1:
			b = self.bubbles[i]
			bb = self.bubbles[i+1]
			tempd = math.sqrt((b.x-bb.x)**2+(b.y-bb.y)**2)
			# check the collapse of the band
			if b.rho+bb.rho <= tempd+400:
				if b.new_label == 1 or bb.new_label == 1:
					return False
				else:
					newone = bubble((b.x+bb.x)/2,(b.y+bb.y)/2,1)
					newone.SetRho(self.__FindNearestObs(newone.x,newone.y))
					self.bubbles.insert(i+1,newone)
					b.SetNew()
					self.N += 1
					i = i+2
			else:
				i += 1
		return True


	# ori_idx should bigger than 0 and less than len(bubble)
	def __CalInForce(self,ori_idx):
		dx0 = self.bubbles[ori_idx-1].x-self.bubbles[ori_idx].x
		dy0 = self.bubbles[ori_idx-1].y-self.bubbles[ori_idx].y
		dx1 = self.bubbles[ori_idx+1].x-self.bubbles[ori_idx].x
		dy1 = self.bubbles[ori_idx+1].y-self.bubbles[ori_idx].y
		l0 = math.sqrt(dx0**2+dy0**2)
		l1 = math.sqrt(dx1**2+dy1**2)
		fx = self.Kc*(dx0/l0+dx1/l1)
		fy = self.Kc*(dy0/l0+dy1/l1)
		return fx,fy

	def __CalRelForce(self,ori_idx):
		if self.bubbles[ori_idx].rho > self.rho0:
			return 0,0
		else:
			#calculate the repulse force
			rho_now = self.bubbles[ori_idx].rho
			h = 25
			t1 = self.__FindNearestObs(self.bubbles[ori_idx].x+h,self.bubbles[ori_idx].y)
			t2 = self.__FindNearestObs(self.bubbles[ori_idx].x-h,self.bubbles[ori_idx].y)
			fx = self.Kr*(self.rho0-rho_now)*(t1-t2)/(2*h)
			t1 = self.__FindNearestObs(self.bubbles[ori_idx].x,self.bubbles[ori_idx].y+h)
			t2 = self.__FindNearestObs(self.bubbles[ori_idx].x,self.bubbles[ori_idx].y-h)
			fy = self.Kr*(self.rho0-rho_now)*(t1-t2)/(2*h)
			return fx,fy


	def __FindNearestObs(self,ori_x,ori_y):
		min_dist = 0xffffff
		min_index = 0
		for i in range(len(self.obs)):
			dist = (self.obs[i].raw_x-ori_x)**2 + (self.obs[i].raw_y-ori_y)**2
			if dist < min_dist:
				min_dist = dist
				min_index = i
		return math.sqrt(min_dist)


	# def __UpdateObs(self):
	# 	self.obs_x =[]
	# 	self.obs_y =[]
	# 	self.rho = []
	# 	for r in self.vision.yellow_robot:
	# 		self.obs_x.append(r.raw_x)
	# 		self.obs_y.append(r.raw_y)
	# 	for i in range(self.N):
	# 		t1 = self.__FindNearestObs(self.bubble_x[i],self.bubble_y[i])
	# 		self.rho.append(t1)

	def __UpdateRho(self):
		for i in range(self.N):
			self.bubbles[i].rho = self.__FindNearestObs(self.bubbles[i].x,self.bubbles[i].y)

	# if all nodes stopped return 0
	# else return 1
	def __CheckNotStop(self):
		for b in self.bubbles:
			if b.stop_label == 0:
				return True
		return False
