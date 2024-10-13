from vision import Vision
from action import Action
from debug import Debugger
import math
from zss_debug_pb2 import Debug_Msgs
import time
from planning import BiRRT,RRT
import numpy as np
import threading
import random
from trajectory import Trajectory
from elasticband import ElasticBand
from rrtstar import RRTStar
from threading import Thread

def plan(goal_x,goal_y,vision):
	while True:
		start_x, start_y = vision.my_robot.x, vision.my_robot.y
		path_x, path_y, draw_x1, draw_y1, draw_x2, draw_y2 = planner.planning(start_x=start_x, start_y=start_y,
	                                                                                  goal_x=goal_x, goal_y=goal_y,
	                                                                                  vision=vision)
		# package = Debug_Msgs()
		# debugger.draw_points(package, path_x, path_y)
		# debugger.draw_circle(package, vision.my_robot.x, vision.my_robot.y)
		# debugger.draw_lines(package, draw_x1, draw_y1, draw_x2, draw_y2)
		# debugger.send(package)
		if len(path_x) > 0 and len(path_y) > 0:
			break
	return path_x,path_y

# Dynamic
if __name__ == '__main__':
	vision = Vision()
	time.sleep(0.01)
	action = Action()
	debugger = Debugger()
	time.sleep(0.01)
	planner = RRTStar(N_SAMPLE = 400,Stepsize = 125)
	trajectory = Trajectory(vision,action)
	eb = ElasticBand(vision,debugger)
	desti = [[-2400,-1500],[2400,1500]]
	dirc = 0

	path_x,path_y = plan(desti[dirc%2][0],desti[dirc%2][1],vision)
	print(path_x,path_y)
	while True:
		path_x,path_y = eb.MoveBubble(path_x,path_y)
		if len(path_x) == 0:
			path_x,path_y = plan(desti[dirc%2][0],desti[dirc%2][1],vision)
			continue
		break
	#planner.SetStep(200,350)
	# while True:
	# 	path_x,path_y = plan(desti[dirc%2][0],desti[dirc%2][1],vision)
	# 	path_x,path_y = eb.MoveBubble(path_x,path_y)
	# 	time.sleep(0.5)
	while True:
		start_time = time.perf_counter()
		# if the path is long enough
		if len(path_x) > 5:
			static_x = path_x[5:].copy()
			static_y = path_y[5:].copy()
			goal_x = path_x[5]
			goal_y = path_y[5]
			resx,resy = eb.MoveBubble(path_x[0:5],path_y[0:5])
			if len(resx) == 0:
				#action.sendCommand(vx = trajectory.v*0.5,vw = trajectory.w*0.5)
				planner.SetStep(200,350)
				path_x,path_y = plan(goal_x,goal_y,vision)
				path_x = path_x + static_x
				path_y = path_y + static_y
				continue
			path_x = resx + static_x
			path_y = resy + static_y
		else:
			resx,resy = eb.MoveBubble(path_x,path_y)
			if len(resx) == 0:
				planner.SetStep(200,350)
				path_x,path_y = plan(desti[dirc%2][0],desti[dirc%2][1],vision)
				continue
			path_x = resx
			path_y = resy
		print(path_x,path_y)
		# nowv = eb.bubbles[0].rho*1.5
		# if nowv > 1000:
		# 	nowv = 1000
		# if nowv < 700:
		# 	nowv = 700
		# trajectory.SetVmax(nowv)
		res = trajectory.Planning(path_x,path_y)
		if res == 0:
			dirc += 1
			planner.SetStep(125,400)
			path_x,path_y = plan(desti[dirc%2][0],desti[dirc%2][1],vision)
		end_time = time.perf_counter()
		print("time"+str((end_time-start_time)*1000))