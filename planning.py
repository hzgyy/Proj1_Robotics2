import math
import random
import numpy as np

class Node(object):
    def __init__(self, x, y, parent=None):
        self.x = x
        self.y = y
        self.parent = parent

class BiRRT(object):
    def __init__(self, MAX_EDGE_LEN=500, N_SAMPLE=200):
        self.minx = -4500
        self.maxx = 4500
        self.miny = -3000
        self.maxy = 3000
        self.robot_size = 250
        self.avoid_dist = 250
        self.MAX_EDGE_LEN = MAX_EDGE_LEN
        self.N_SAMPLE = N_SAMPLE
        self.nodeListStart = []
        self.nodeListGoal = []

    def distanceCost(self, a, b):
        return math.sqrt((a - b) ** 2)

    def feasiblePoint(self, vision, point_x, point_y):
        for robot_blue in vision.blue_robot:
            if robot_blue.visible and robot_blue.id > 0:
                if self.distanceCost(robot_blue.x, point_x)**2 + self.distanceCost(robot_blue.y, point_y)**2 < (self.robot_size + self.avoid_dist)**2:
                    return False
        for robot_yellow in vision.yellow_robot:
            if robot_yellow.visible:
                if self.distanceCost(robot_yellow.x, point_x)**2 + self.distanceCost(robot_yellow.y, point_y)**2 < (self.robot_size + self.avoid_dist)**2:
                    return False
        return True

    def randompoint(self, vision):
        while True:
            tx = random.uniform(self.minx, self.maxx)
            ty = random.uniform(self.miny, self.maxy)
            if self.feasiblePoint(vision, tx, ty):
                return tx, ty

    def sample(self, vision):
        return self.randompoint(vision)

    def nearest_node(self, node_list, rnd):
        min_dist = float('inf')
        nearest_node = None
        for node in node_list:
            dist = self.distanceCost(node.x, rnd[0]) + self.distanceCost(node.y, rnd[1])
            if dist < min_dist:
                min_dist = dist
                nearest_node = node
        return nearest_node

    def steer(self, nearest_node, rnd):
        theta = math.atan2(rnd[1] - nearest_node.y, rnd[0] - nearest_node.x)
        new_node_x = nearest_node.x + math.cos(theta) * min(self.MAX_EDGE_LEN, self.distanceCost(nearest_node.x, rnd[0]))
        new_node_y = nearest_node.y + math.sin(theta) * min(self.MAX_EDGE_LEN, self.distanceCost(nearest_node.y, rnd[1]))
        return new_node_x, new_node_y

    def planning(self, goal_x, goal_y, start_x, start_y, vision):
        self.nodeListStart = [Node(start_x, start_y)]
        self.nodeListGoal = [Node(goal_x, goal_y)]

        for _ in range(self.N_SAMPLE):
            rnd = self.sample(vision)

            # Expand tree from start
            nearest_node_start = self.nearest_node(self.nodeListStart, rnd)
            new_node_start_x, new_node_start_y = self.steer(nearest_node_start, rnd)

            # Check feasibility of the new node before adding it
            if self.feasiblePoint(vision, new_node_start_x, new_node_start_y):
                new_node_start = Node(new_node_start_x, new_node_start_y)
                new_node_start.parent = nearest_node_start
                self.nodeListStart.append(new_node_start)

                # Check for connection to goal tree
                if (self.distanceCost(new_node_start.x, goal_x) < 100 and 
                    self.distanceCost(new_node_start.y, goal_y) < 100):
                    return self.construct_path(new_node_start)

            # Expand tree from goal
            nearest_node_goal = self.nearest_node(self.nodeListGoal, rnd)
            new_node_goal_x, new_node_goal_y = self.steer(nearest_node_goal, rnd)

            # Check feasibility of the new node before adding it
            if self.feasiblePoint(vision, new_node_goal_x, new_node_goal_y):
                new_node_goal = Node(new_node_goal_x, new_node_goal_y)
                new_node_goal.parent = nearest_node_goal
                self.nodeListGoal.append(new_node_goal)

                # Check for connection to start tree
                if (self.distanceCost(new_node_goal.x, start_x) < 100 and 
                    self.distanceCost(new_node_goal.y, start_y) < 100):
                    return self.construct_path(new_node_goal)

        return [], [], [], [], [], []

    def construct_path(self, meeting_point):
        path_x_start_to_meeting = []
        path_y_start_to_meeting = []

        # Trace back from the meeting point to the start
        node = meeting_point
        while node is not None:
            path_x_start_to_meeting.append(node.x)
            path_y_start_to_meeting.append(node.y)
            node = node.parent

        path_x_start_to_meeting.reverse()  # Reverse for correct order
        path_y_start_to_meeting.reverse()  # Reverse for correct order

        path_x_goal_to_meeting = []
        path_y_goal_to_meeting = []

        # Now trace back from the meeting point to the goal
        node = meeting_point.parent  # Start from the parent of the meeting point
        while node is not None:
            path_x_goal_to_meeting.append(node.x)
            path_y_goal_to_meeting.append(node.y)
            node = node.parent

        # Combine both paths: Start to Meeting Point + Meeting Point to Goal
        
        draw_x1 = []
        draw_y1 = []
        draw_x2 = []
        draw_y2 = []

        # Collect drawing data only from start tree and goal tree without connecting them directly.
        for node in self.nodeListStart:
            if node.parent is not None:
                draw_x1.append(node.parent.x)
                draw_y1.append(node.parent.y)
                draw_x2.append(node.x)
                draw_y2.append(node.y)

        for node in self.nodeListGoal:
            if node.parent is not None:
                draw_x1.append(node.parent.x)
                draw_y1.append(node.parent.y)
                draw_x2.append(node.x)
                draw_y2.append(node.y)

        return (path_x_start_to_meeting + path_x_goal_to_meeting,
                path_y_start_to_meeting + path_y_goal_to_meeting,
                draw_x1,
                draw_y1,
                draw_x2,
                draw_y2)  # Return combined paths and drawing data without extra lines between start and goal.



class RRT(object):
    def __init__(self, MAX_EDGE_LEN=500, N_SAMPLE=200):
        self.minx = -4500
        self.maxx = 4500
        self.miny = -3000
        self.maxy = 3000
        self.robot_size = 250
        self.avoid_dist = 250

        self.MAX_EDGE_LEN = MAX_EDGE_LEN
        self.N_SAMPLE = N_SAMPLE
        self.nodeList = []

    def distanceCost(self, a, b):
        return math.sqrt((a - b) ** 2)

    def feasiblePoint(self, vision, point_x, point_y):
        for robot_blue in vision.blue_robot:
            if robot_blue.visible and robot_blue.id > 0:
                if self.distanceCost(robot_blue.x, point_x)**2 + self.distanceCost(robot_blue.y, point_y)**2 < (self.robot_size + self.avoid_dist)**2:
                    return False
        for robot_yellow in vision.yellow_robot:
            if robot_yellow.visible:
                if self.distanceCost(robot_yellow.x, point_x)**2 + self.distanceCost(robot_yellow.y, point_y)**2 < (self.robot_size + self.avoid_dist)**2:
                    return False
        return True

    def randompoint(self, vision):
        while True:
            tx = random.uniform(self.minx, self.maxx)
            ty = random.uniform(self.miny, self.maxy)
            if self.feasiblePoint(vision, tx, ty):
                return tx, ty

    def sample(self, vision):
        return self.randompoint(vision)

    def nearest_node(self, node_list, rnd):
        min_dist = float('inf')
        nearest_node = None
        for node in node_list:
            dist = self.distanceCost(node.x, rnd[0]) + self.distanceCost(node.y, rnd[1])
            if dist < min_dist:
                min_dist = dist
                nearest_node = node
        return nearest_node

    def steer(self, nearest_node, rnd):
        theta = math.atan2(rnd[1] - nearest_node.y, rnd[0] - nearest_node.x)
        new_node_x = nearest_node.x + math.cos(theta) * min(self.MAX_EDGE_LEN, self.distanceCost(nearest_node.x, rnd[0]))
        new_node_y = nearest_node.y + math.sin(theta) * min(self.MAX_EDGE_LEN, self.distanceCost(nearest_node.y, rnd[1]))
        return new_node_x, new_node_y

    def planning(self, goal_x, goal_y, start_x, start_y, vision):
        self.nodeList = []
        start_node = Node(start_x, start_y)
        self.nodeList.append(start_node)

        for _ in range(self.N_SAMPLE):
            rnd = self.sample(vision)
            nearest_node = self.nearest_node(self.nodeList, rnd)
            new_node_x, new_node_y = self.steer(nearest_node, rnd)

            if self.feasiblePoint(vision, new_node_x, new_node_y):
                new_node = Node(new_node_x, new_node_y)
                new_node.parent = nearest_node
                self.nodeList.append(new_node)

            last_node = self.nearest_node(self.nodeList, (goal_x, goal_y))
            if (self.distanceCost(last_node.x, goal_x) < 100 and 
                self.distanceCost(last_node.y, goal_y) < 100):
                break

        path_x, path_y = [], []
        path_x.append(goal_x)
        path_y.append(goal_y)

        last_node = last_node if last_node else start_node
        
        while last_node is not None:
            path_x.append(last_node.x)
            path_y.append(last_node.y)
            last_node = last_node.parent

        draw_x1, draw_y1, draw_x2, draw_y2 = [], [], [], []
        
        for node in self.nodeList:
            if node.parent is not None:
                draw_x1.append(node.parent.x)
                draw_y1.append(node.parent.y)
                draw_x2.append(node.x)
                draw_y2.append(node.y)

        return path_x[::-1], path_y[::-1], draw_x1, draw_y1, draw_x2, draw_y2