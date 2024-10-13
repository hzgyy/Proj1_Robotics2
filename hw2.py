import math
import numpy as np

class EightNum:
	def __init__(self,num_list,last,target,father_cost):
		self.Num = num_list.copy()
		self.LastMove = last
		self.estimate = np.sum(self.Num != target)
		self.cost = father_cost+1
		self.target = target
		self.f = self.estimate+self.cost

	def FindChild(self):
		childs = []
		zero_index = np.where(self.Num == 0)
		z_row = zero_index[0][0]
		z_col = zero_index[1][0]
		for i in (-1,1):
			if z_row+i >=0 and z_row+i <= 2 and self.Num[z_row+i][z_col] != self.LastMove:
				newlist = self.Num.copy()
				newlist[z_row][z_col] = newlist[z_row+i][z_col]
				newlist[z_row+i][z_col] = 0
				childs.append(EightNum(newlist,self.Num[z_row+i][z_col],self.target,self.cost))
			if z_col+i >=0 and z_col+i <= 2 and self.Num[z_row][z_col+i] != self.LastMove:
				newlist = self.Num.copy()
				newlist[z_row][z_col] = newlist[z_row][z_col+i]
				newlist[z_row][z_col+i] = 0
				childs.append(EightNum(newlist,self.Num[z_row][z_col+i],self.target,self.cost))
		return childs

	def ReachGoal(self):
		return np.array_equal(self.Num,self.target)


class RGBF:
	def __init__(self,start):
		res,_ = self.rgbf(start,0xfffff)
		print("RGBF:"+str(res))

	def rgbf(self,node,flimit):
		# whether reach the goal
		if node.ReachGoal():
			return node.cost,node.f
		successors = node.FindChild()
		if len(successors) == 0:
			return -1,0xfffff
		while True:
			successors = sorted(successors,key = get_estimate)
			best = successors[0]
			if best.f > flimit:
				node.f = best.f
				return -1, best.f
			if len(successors) > 1:
				second_best_f = successors[1].f
			else:
				second_best_f = 0xfffff
			res,best.f = self.rgbf(best,min(second_best_f,flimit))
			if res != -1:
				return res,best.f


class AStar:
	def __init__(self,start):
		pool = [start]
		inlier = start
		while True:
			pool += inlier.FindChild()
			pool.remove(inlier)
			#find next
			inlier = min(pool,key = get_estimate)
			if np.array_equal(inlier.Num,target):
				break
		print("AStar:"+str(inlier.cost))

def get_estimate(obj):  
    return obj.f

if __name__ == '__main__':
	# define the target and the start_status
	target = np.array([[1,2,3],[8,0,4],[7,6,5]])
	initial = np.array([[1,4,3],[8,0,6],[7,8,5]])
	# define the problem object
	start = EightNum(initial,0,target,-1)
	# solve the problem
	a = AStar(start)
	b = RGBF(start)
	




	













