import numpy as np
import RobotUtil as rt
import math
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D


class FrankArm:
	
	def __init__(self):
		# Robot descriptor taken from URDF file (rpy xyz for each rigid link transform) - NOTE: don't change
		self.Rdesc=[
			[0, 0, 0, 0., 0, 0.333], # From robot base to joint1
			[-np.pi/2, 0, 0, 0, 0, 0],
			[np.pi/2, 0, 0, 0, -0.316, 0],
			[np.pi/2, 0, 0, 0.0825, 0, 0],
			[-np.pi/2, 0, 0, -0.0825, 0.384, 0],
			[np.pi/2, 0, 0, 0, 0, 0],
			[np.pi/2, 0, 0, 0.088, 0, 0],
			[0, 0, 0, 0, 0, 0.107] # From joint5 to end-effector center
			]
		
		#Define the axis of rotation for each joint 
		self.axis=[
				[0, 0, 1],
				[0, 0, 1],
				[0, 0, 1],
				[0, 0, 1],
				[0, 0, 1],
				[0, 0, 1],
				[0, 0, 1],
				[0, 0, 1]
				]

		#Set base coordinate frame as identity - NOTE: don't change
		self.Tbase= [[1,0,0,0],
			[0,1,0,0],
			[0,0,1,0],
			[0,0,0,1]]
		
		#Initialize matrices - NOTE: don't change this part
		self.Tlink=[] #Transforms for each link (const)
		self.Tjoint=[] #Transforms for each joint (init eye)
		self.Tcurr=[] #Coordinate frame of current (init eye)
		for i in range(len(self.Rdesc)):
			self.Tlink.append(rt.rpyxyz2H(self.Rdesc[i][0:3],self.Rdesc[i][3:6]))
			self.Tcurr.append([[1,0,0,0],[0,1,0,0],[0,0,1,0.],[0,0,0,1]])
			self.Tjoint.append([[1,0,0,0],[0,1,0,0],[0,0,1,0.],[0,0,0,1]])

		self.Tlinkzero=rt.rpyxyz2H(self.Rdesc[0][0:3],self.Rdesc[0][3:6])  

		self.Tlink[0]=np.matmul(self.Tbase,self.Tlink[0])					

		# initialize Jacobian matrix
		self.J=np.zeros((6,7))
		
		self.q=[0.,0.,0.,0.,0.,0.,0.]
		self.ForwardKin([0.,0.,0.,0.,0.,0.,0.])
		
	def ForwardKin(self,ang):
		'''
		inputs: joint angles
		outputs: joint transforms for each joint, Jacobian matrix
		'''
		self.q[0:-1]=ang

		# Calculate the Joint transforms
		for i in range(len(self.q)):

			self.Tjoint[i] = [[math.cos(self.q[i]), -math.sin(self.q[i]), 0, 0], [math.sin(self.q[i]),  math.cos(self.q[i]), 0, 0], [0,0,1,0], [0,0,0,1]]

			if i is 0:
				self.Tcurr[i] = np.matmul(self.Tlink[i], self.Tjoint[i])
			else:
				self.Tcurr[i] = np.matmul(np.matmul(self.Tcurr[i-1], self.Tlink[i]), self.Tjoint[i])

		for i in range(len(self.Tcurr) - 1):
			p = self.Tcurr[-1][0:3, 3] - self.Tcurr[i][0:3, 3]
			a = self.Tcurr[i][0:3,2]
			self.J[0:3,i] = np.cross(a,p)
			self.J[3:7, i] = a


		return self.Tcurr, self.J
				

	def IterInvKin(self,ang,TGoal,x_eps=1e-3, r_eps=1e-3):
		'''
		inputs: starting joint angles (ang), target end effector pose (TGoal)

		outputs: computed joint angles to achieve desired end effector pose, 
		Error in your IK solution compared to the desired target
		'''	

		W = np.diag([1, 1, 100, 100, 1, 1, 100])
		C = np.diag([10000000, 10000000, 10000000, 1000, 1000, 1000])

		Err = 0

		f_q = x_0 + J @ (q - q_0)

		
		

		return self.q[0:-1], Err