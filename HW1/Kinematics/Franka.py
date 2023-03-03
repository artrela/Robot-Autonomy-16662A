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

		# 1. Calculate the Joint transforms: all rotations about z
		# 2. No rot between base frame and first link
		# 3. After that there is a transformation of a translation along a link and rot about a joint
		# 4. Create the jacobian w/ column by column method
			- all revolute joints (first row is omega x r & then the second is omega)
		'''

		self.q[0:-1]=ang

		
		for i in range(len(self.q)):
			# 1
			self.Tjoint[i] = [[math.cos(self.q[i]), -math.sin(self.q[i]), 0, 0], [math.sin(self.q[i]),  math.cos(self.q[i]), 0, 0], [0,0,1,0], [0,0,0,1]]
			# 2 
			if i == 0:
				self.Tcurr[i] = np.matmul(self.Tlink[i], self.Tjoint[i])
			# 3
			else:
				self.Tcurr[i] = np.matmul(np.matmul(self.Tcurr[i-1], self.Tlink[i]), self.Tjoint[i])

		# 4
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

		1. Create weight and cost
			- also init fk real quick
		2. init error vals
		3. check if either error is too high
		4. find the rotation error
		5. Clip the angle if its too big
		6. find the error in axis angle terms
		7. find the displacement error
		8. clip the displacement if its too big
		9. construct J_hash according to damped least squares eq
		10. update err
		11. update joint positions
		12. update the jacobian & htms
		'''	
		# 1 
		W = np.diag([1, 1, 100, 100, 1, 1, 100])
		W[-1, 0] = 1
		C = np.diag([1000000, 1000000, 1000000, 1000, 1000, 1000])

		#! Corrected from submission
		#! Add one iteration of FK before IK 
		#! Sets joints at proper starting points
		self.q[0:-1] = ang
		self.ForwardKin(self.q[0:-1])


		# 2
		Err = np.ones(6) * np.inf
		xErr, rERR_A = Err[0:3], Err[3:6]
		
		# 3
		while np.linalg.norm(xErr) > x_eps or np.linalg.norm(rERR_A) > r_eps:

			# 4 
			rERR_R = TGoal[0:3, 0:3] @ self.Tcurr[-1][0:3, 0:3].T # Rerr = Rgoal*Rerrcurr'
			rERRAxis, rErrAng = rt.R2axisang(rERR_R)

			# 5 
			if rErrAng > 0.1:
				rErrAng = 0.1

			if rErrAng < -0.1:
				rErrAng = -0.1
			
			# 6 
			rERR_A = [rERRAxis[0]*rErrAng, rERRAxis[1]*rErrAng, rERRAxis[2]*rErrAng]

			# 7 
			xErr = TGoal[0:3, 3] - self.Tcurr[-1][0:3, 3] # Xerr = Xgoal - Xcurr

			# 8 
			if np.linalg.norm(xErr) > 0.01:
				xErr = xErr * 0.01 / np.linalg.norm(xErr)

			# 9 
			J_Hash = np.linalg.inv(W) @ self.J.T @ np.linalg.inv(self.J @ np.linalg.inv(W) @ self.J.T + np.linalg.inv(C))

			# 10 
			Err[0:3] = xErr
			Err[3:6] = rERR_A

			# 11 
			self.q[0:-1] += J_Hash @ Err

			# 12 
			self.ForwardKin(self.q[0:-1])

		print(f"Jacobian: {self.J} | ")

		return self.q[0:-1], Err