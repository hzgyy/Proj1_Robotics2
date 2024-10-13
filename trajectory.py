import math
import time
from debug import Debugger
from zss_debug_pb2 import Debug_Msgs

#the control and feedback method for trajectory generation
class Trajectory:
	ReachRange = 400
	StopRange = 50
	K1 = 1
	K2 = 3
	Vmax = 750
	beta = 0.3
	lamda = 2
	tao = 5
	Kp = 1.5
	v = 0
	w = 0
	def __init__(self,vision,action):
		self.vision = vision
		self.my_robot = vision.my_robot
		self.action = action

	#use this function to follow the trajectory
	def FollowPath(self,path_x,path_y):
		for i in range(1,len(path_x)):
			des_x = path_x[i]
			des_y = path_y[i]
			des_o = math.atan2(des_y-path_y[i-1],des_x-path_x[i-1])
			#des_o = 0
			# the control loop for direct move
			while self.__Reach(des_x,des_y,self.ReachRange) == False:
				#change to the polar coordinate
				r,delta,theta = self.__TurnPolar(des_x,des_y,des_o)
				v,w = self.__PlanSpeed(r,delta,theta)
				self.action.sendCommand(vx = v,vy = 0, vw = w)
				print("r:%.2f,delta:%.2f,theta:%.2f\n v:%.2f,w:%.2f,odes%.2f\n realv:%.2f,orientation:%.2f\n"\
					%(r,delta,theta,v,w,des_o,self.my_robot.raw_vel_x,self.my_robot.raw_orientation),flush = True)
				time.sleep(0.05)

			if i+1 == len(path_x):
				# goto stop function
				self.__Stop(des_x,des_y,des_o)	
			else:
				# goto transition function
				next_x = path_x[i+1]
				next_y = path_y[i+1]
				next_o = math.atan2(next_y-path_y[i],next_x-path_x[i])
				#counter for trans
				tc = 0
				while tc < self.tao:
					tc = tc+1
					r1,delta1,theta1 = self.__TurnPolar(des_x,des_y,des_o)
					r2,delta2,theta2 = self.__TurnPolar(next_x,next_y,next_o)
					tv,tw = self.__PlanTrans(tc,[r1,r2],[delta1,delta2],[theta1,theta2])
					self.action.sendCommand(vx = tv,vw = tw)
					time.sleep(0.01)

	def Planning(self,path_x,path_y):
		i = 1
		des_x = path_x[i]
		des_y = path_y[i]
		des_o = math.atan2(des_y-path_y[i-1],des_x-path_x[i-1])		
		if self.__Reach(des_x,des_y,self.ReachRange) == False:
			#change to the polar coordinate
			r,delta,theta = self.__TurnPolar(des_x,des_y,des_o)
			v,w = self.__PlanSpeed(r,delta,theta)
			self.action.sendCommand(vx = v,vy = 0, vw = w)
			self.v = v
			self.w = w
			#print("r:%.2f,delta:%.2f,theta:%.2f\n v:%.2f,w:%.2f,odes%.2f\n realv:%.2f,orientation:%.2f\n"\
			#		%(r,delta,theta,v,w,des_o,self.my_robot.raw_vel_x,self.my_robot.raw_orientation),flush = True)
			#time.sleep(0.05)
		#counter for trans
		else:
			if len(path_x) == 2:
				self.__Stop(des_x,des_y,des_o)
				return 0
			tc = 0
			next_x = path_x[i+1]
			next_y = path_y[i+1]
			next_o = math.atan2(next_y-path_y[i],next_x-path_x[i])
			while tc < self.tao:
				tc = tc+1
				r1,delta1,theta1 = self.__TurnPolar(des_x,des_y,des_o)
				r2,delta2,theta2 = self.__TurnPolar(next_x,next_y,next_o)
				tv,tw = self.__PlanTrans(tc,[r1,r2],[delta1,delta2],[theta1,theta2])
				self.action.sendCommand(vx = tv,vw = tw)
				time.sleep(0.01)
			del path_x[0]
			del path_y[0]
		return 1


	def __Stop(self,des_x,des_y,des_o):
		# goto stop function
		while self.__Reach(des_x,des_y,self.StopRange) == False:
			r,delta,theta = self.__TurnPolar(des_x,des_y,des_o)
			if delta >= -1.57 and delta <= 1.57:
				kai = 1/r*(self.K2*(delta-math.atan(-self.K1*theta))+(1+self.K1/(1+(self.K1*theta)**2))*math.sin(delta))
			else:
				kai = 1/r*(self.K2*(delta-math.atan(-self.K1*theta))-(1+self.K1/(1+(self.K1*theta)**2))*math.sin(delta))
			v = self.Kp * r
			w = v * kai
			self.action.sendCommand(vx = v, vw = w)
			time.sleep(0.01)
		# total stop
		self.action.sendCommand(vx = 0, vw = 0)

	def __PlanSpeed(self,r,delta,theta):
		if delta >= -1.57 and delta <= 1.57:
			kai = 1/r*(self.K2*(delta-math.atan(-self.K1*theta))+(1+self.K1/(1+(self.K1*theta)**2))*math.sin(delta))
		else:
			kai = 1/r*(self.K2*(delta-math.atan(-self.K1*theta))-(1+self.K1/(1+(self.K1*theta)**2))*math.sin(delta))
		#print('\r'+str(kai)+'\n',end = '',flush = True)
		#print("\n")
		v = self.Vmax/(1+self.beta*abs(kai)**self.lamda)
		w = v*kai
		return v,w


	def __Reach(self,des_x,des_y,Range):
		distance = (self.my_robot.x-des_x)**2 + (self.my_robot.y-des_y)**2
		if distance <= Range**2:
			return True
		else:
			return False

	def __TurnPolar(self,des_x,des_y,des_o):
		now_x = self.my_robot.x
		now_y = self.my_robot.y
		now_o = self.my_robot.orientation
		r = math.sqrt((now_x-des_x)**2+(now_y-des_y)**2)
		delta = math.atan2(des_y-now_y,des_x-now_x)-now_o
		theta = -des_o+math.atan2(des_y-now_y,des_x-now_x)
		delta = (delta+math.pi) % (2*math.pi) - math.pi
		theta = (theta+math.pi) % (2*math.pi) - math.pi
		return r,delta,theta

	def __PlanTrans(self,t,rs,deltas,thetas):
		#calculate sigmoid
		scaler = 1/0.98*(1/(1+math.exp(-9.2*(t/self.tao-0.5)))-0.01)
		v1,w1 = self.__PlanSpeed(rs[0],deltas[0],thetas[0])
		v2,w2 = self.__PlanSpeed(rs[1],deltas[1],thetas[1])
		return (1-scaler)*v1+scaler*v2,(1-scaler)*w1+scaler*w2

	def SetVmax(self,vmax):
		self.Vmax =vmax
