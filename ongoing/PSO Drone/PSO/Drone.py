import numpy as np
import math

c1 = 0.7
c2 = 0.7
w = 0.7

def obj_func(func_num, pos):
	c = 0.0
	if func_num == 0:
		for i in range(len(pos)-1):
			c = c + 100*(pos[i+1] - pos[i]*pos[i])**2 + (1-pos[i])**2 
		c = 1.0/c
	elif func_num == 1:
		Q = 10
		u = 3
		h = 10
		l_y = -2.555
		j_y = 1.0423
		k_y = -0.0087
		l_z = -3.186
		j_z = 1.1137
		k_z = -0.0316
		(x,y,z) = pos
		if x<0:
			pass
		else:
			s_y = math.exp(l_y+j_y*math.log(x+0.000001)+k_y*math.log(x+0.000001)**2)	
			s_z = math.exp(l_z+j_z*math.log(x+0.000001)+k_z*math.log(x+0.000001)**2)
			c = Q / (2*math.pi*s_y*s_z) * math.exp(-0.5*(y/s_y)**2) * \
					(math.exp(-0.5*((z-h)/s_z)**2) + math.exp(-0.5*((z+h)/s_z)**2))	
	return c

class Drone:
	def __init__(self, func_num=0, num_dims=3, lower=(-50.0, -50.0, 0.0), upper=(50.0, 50.0, 50.0)):
		self.func_num = func_num
		self.num_dims = num_dims
		self.lower = np.array(lower)
		self.upper = np.array(upper)
		rand = np.random.random_sample(num_dims)
		self.pos = (self.upper-self.lower) * rand + self.lower
		self.vel = np.zeros(num_dims,int)
		self.obj_val = obj_func(self.func_num, self.pos)
		self.prev_pos = self.pos.copy()
		self.pb_pos = self.pos.copy()
		self.pb_val = self.obj_val
		self.trajectory = [self.pos.copy()]

	def check_bound(self):
		for i in range(0,self.num_dims):
			if self.pos[i]>self.upper[i] or self.pos[i]<self.lower[i]:
				rand = np.random.random_sample()
				self.pos[i] = (self.upper[i]-self.lower[i]) * rand + self.lower[i]

	def update_pb(self):
		if self.obj_val > self.pb_val:
			self.pb_pos = self.pos.copy()
			self.pb_val = self.obj_val

	def update_pb_online(self, pos_):
		pos = np.array(pos_)*10.0
		value = obj_func(self.func_num, pos)
		if value > self.pb_val:
			self.pb_pos = pos.copy()
			self.pb_val = value

	def add_trajectory(self):
		self.trajectory.append(self.pos.copy())

class Swarm:
	def __init__(self, func_num=0, num_drones=10, num_dims=3, lower=(-50.0, -50.0, 0.0), upper=(50.0, 50.0, 50.0)):
		self.func_num = func_num
		self.lower = np.array(lower)
		self.upper = np.array(upper)
		self.num_drones = num_drones
		self.num_dims = num_dims
		self.drones = []
		for i in range(0, num_drones):
			self.drones.append(Drone(self.func_num, num_dims, lower, upper))
		self.gb_pos = self.drones[0].pos.copy()
		self.gb_val = self.drones[0].obj_val
		self.cur_iter = 0

	def check_collision(self, idx):
		is_collision = 0
		collision_thres = 0.5 #meters
		for i in range(0,idx):
			p1 = self.drones[idx].pos
			p2 = self.drones[i].pos
			u1 = self.drones[idx].pos - self.drones[idx].prev_pos
			u2 = self.drones[i].pos - self.drones[i].prev_pos
			p1p2 = p2-p1
			a = np.cross(u1,u2)
			b = np.linalg.norm(a)
			if b==0:
				return 0
			dist = np.linalg.norm(np.dot(p1p2,a)) / b
			#print(dist)
			if dist<collision_thres:
				is_collision+=1
		if is_collision>0:
			return 1
		else:
			return 0

	def update_drones(self):
		self.cur_iter += 1
		for drone in self.drones:
			drone.vel = w * drone.vel
			rand = np.random.random_sample((2, drone.num_dims))
			drone.vel = drone.vel + c1 * rand[0] * (drone.pb_pos - drone.pos)
			drone.vel = drone.vel + c2 * rand[1] * (self.gb_pos - drone.pos)
			tmp_pos = drone.prev_pos.copy()
			drone.prev_pos = drone.pos.copy()
			drone.pos = drone.pos + drone.vel
			drone.check_bound()
			if self.check_collision(self.drones.index(drone)):
				#drone.pos = drone.prev_pos.copy()
				#drone.prev_pos = tmp_pos.copy()
				pass
			#drone.obj_val = obj_func(drone.pos)
			#drone.update_pb()
			if drone.pb_val > self.gb_val:
				self.gb_pos = drone.pb_pos.copy()
				self.gb_val = drone.pb_val

print(obj_func(1, (0, 0, 10.0)))
# s = Swarm(10)
# for i in range(0,100):
# 	s.update_drones()
# print(s.gb_pos)
# print(s.gb_val)