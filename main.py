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

def plan(goal_x,goal_y,vision):
	while True:
		start_x, start_y = vision.my_robot.x, vision.my_robot.y
		path_x, path_y, draw_x1, draw_y1, draw_x2, draw_y2 = planner.planning(start_x=start_x, start_y=start_y,
	                                                                                  goal_x=goal_x, goal_y=goal_y,
	                                                                                  vision=vision)
		package = Debug_Msgs()
		debugger.draw_points(package, path_x, path_y)
		debugger.draw_circle(package, vision.my_robot.x, vision.my_robot.y)
		debugger.draw_lines(package, draw_x1, draw_y1, draw_x2, draw_y2)
		debugger.send(package)
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
	planner = RRTStar(N_SAMPLE = 350,Stepsize = 150)
	trajectory = Trajectory(vision,action)
	eb = ElasticBand(vision,debugger)
	# stop
	path_x,path_y = plan(-2400,-1500,vision)
	time.sleep(3)
	path_x,path_y = eb.MoveBubble(path_x,path_y)
	while True:
		start_time = time.perf_counter()
		if len(path_x) > 5:
			static_x = path_x[5:].copy()
			static_y = path_y[5:].copy()
			resx,resy = eb.MoveBubble(path_x[0:5],path_y[0:5])
			if len(resx) == 0:
				path_x,path_y = plan(path_x[5],path_y[5],vision)
				path_x = path_x + static_x
				path_y = path_y + static_y
				continue
			path_x = resx + static_x
			path_y = resy + static_y
		else:
			resx,resy = eb.MoveBubble(path_x,path_y)
			if len(resx) == 0:
				path_x,path_y = plan(-2400,-1500,vision)
				path_x = path_x
				path_y = path_y
				continue
			path_x = resx
			path_y = resy
		trajectory.Planning(path_x,path_y)
		end_time = time.perf_counter()
		print("time"+str((end_time-start_time)*1000))

        	

