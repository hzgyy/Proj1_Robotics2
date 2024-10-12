import math
import random
import numpy as np

class Node:
    def __init__(self, x, y, cost, parent):
        self.x = x
        self.y = y
        self.cost = cost
        self.parent = parent

class RRTStar:
    def __init__(self, MAX_EDGE_LEN=10000, N_SAMPLE=300, KNN=5, Prob=0.3, Stepsize=125):
        self.minx = -4500
        self.maxx = 4500
        self.miny = -3000
        self.maxy = 3000
        self.robot_size = 250
        self.avoid_dist = 250

        self.MAX_EDGE_LEN = MAX_EDGE_LEN
        self.N_SAMPLE = N_SAMPLE
        self.KNN = KNN
        self.Prob = Prob
        self.Stepsize = Stepsize
        
        self.nodeList = []

    def distance(self, a, b):
        return math.sqrt((a - b) ** 2)

    def is_feasible(self, vision, point_x, point_y):
        for robot_blue in vision.blue_robot:
            if robot_blue.visible and robot_blue.id > 0:
                if self.distance(robot_blue.x, point_x)**2+ self.distance(robot_blue.y, point_y)**2 < (self.robot_size/1.8 + self.avoid_dist/1.8)**2:
                    return False
        for robot_yellow in vision.yellow_robot:
            if robot_yellow.visible:
                if self.distance(robot_yellow.x, point_x)**2+self.distance(robot_yellow.y, point_y)**2 < (self.robot_size/1.8 + self.avoid_dist/1.8)**2:
                    return False
        return True

    def random_point(self, vision):
        while True:
            tx = random.uniform(self.minx, self.maxx)
            ty = random.uniform(self.miny, self.maxy)
            if self.is_feasible(vision, tx, ty):
                return tx, ty

    def sample(self, vision, goal_x, goal_y):
        return (goal_x, goal_y) if random.random() < self.Prob else self.random_point(vision)

    def nearest_node(self, rnd):
        return min(self.nodeList, key=lambda node: self.distance(node.x, rnd[0]) + self.distance(node.y, rnd[1]))

    def nearest_nodes(self, new_node):
        distances = [self.distance(new_node.x, node.x) + self.distance(new_node.y, node.y) for node in self.nodeList]
        indices = np.argsort(distances)
        return [self.nodeList[i] for i in indices[:self.KNN]]

    def steer(self, nearest_node, rnd):
        theta = math.atan2(rnd[1] - nearest_node.y, rnd[0] - nearest_node.x)
        new_node_x = nearest_node.x + math.cos(theta) * self.Stepsize
        new_node_y = nearest_node.y + math.sin(theta) * self.Stepsize
        return new_node_x, new_node_y

    def rewire(self, new_node, near_nodes):
        for node in near_nodes:
            new_cost = new_node.cost + self.distance(new_node.x, node.x) + self.distance(new_node.y, node.y)
            if new_cost < node.cost:
                node.parent = new_node
                node.cost = new_cost

    def planning(self, goal_x, goal_y, start_x, start_y, vision):
        self.nodeList.clear()
        
        start_node = Node(start_x, start_y, 0.0, None)
        self.nodeList.append(start_node)

        for i in range(self.N_SAMPLE):
            rnd = self.sample(vision, goal_x, goal_y)
            nearest_node = self.nearest_node(rnd)
            new_node_x, new_node_y = self.steer(nearest_node, rnd)
            new_node = Node(new_node_x, new_node_y, float('inf'), None)

            if not self.is_feasible(vision, new_node_x, new_node_y):
                continue
            
            near_nodes = self.nearest_nodes(new_node)
            min_cost_node = nearest_node
            min_cost_value = nearest_node.cost + self.distance(nearest_node.x, new_node_x) + \
                             self.distance(nearest_node.y, new_node_y)

            for near_node in near_nodes:
                cost_value = near_node.cost + \
                             self.distance(near_node.x, new_node_x) + \
                             self.distance(near_node.y, new_node_y)
                if cost_value < min_cost_value:
                    min_cost_value = cost_value
                    min_cost_node = near_node
            
            new_node.cost = min_cost_value
            new_node.parent = min_cost_node
            
            # Add the new node to the list and rewire nearby nodes.
            self.nodeList.append(new_node)
            self.rewire(new_node, near_nodes)

            # Check if we are close enough to the goal.
            last_node = self.nearest_node((goal_x, goal_y))
            if (self.distance(last_node.x, goal_x)**2 + 
                self.distance(last_node.y, goal_y)**2 < 1000):
                break

        # Backtrack to get the path.
        path_x, path_y = [], []
        
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

        return path_x[::-1], path_y[::-1], draw_x1[::-1], draw_y1[::-1], draw_x2[::-1], draw_y2[::-1]