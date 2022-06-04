import math

import rospy
from hector_uav_msgs.srv import *
from std_srvs.srv import Empty
import random
import time
from gazebo_msgs.srv import *
from hector_uav_msgs.msg import *
from geometry_msgs.msg import *
import actionlib

ALTITUDE = 2

worldReferenceFrame = 'ground_plane'
rospy.init_node('quadrotor', anonymous=True)
PoseClient = PoseAction("/action/Pose", PoseActionGoal, True)
pose_client = actionlib.SimpleActionClient('/action/pose', PoseAction)
landing_client = actionlib.SimpleActionClient('/action/landing', LandingAction)

enable_motors = rospy.ServiceProxy("/enable_motors", EnableMotors)


model_coordinates = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
world_properties = rospy.ServiceProxy("/gazebo/get_world_properties", GetWorldProperties)



pose_client.wait_for_server()
landing_client.wait_for_server()


class GazeboOperations:
	@staticmethod
	def resetSim():
		reset = rospy.ServiceProxy("/gazebo/reset_world", Empty)
		reset()
	
	@staticmethod
	def getTargetsPositions(target_name):
		target_positions = []
		
		gazebo_all_model_names = world_properties()
		target_models = [name for name in gazebo_all_model_names.model_names if name.startswith(target_name)]
		
		for model in target_models:
			target_positions.append(GazeboOperations.getRelativePosition(model, worldReferenceFrame))
		
		return target_positions
	
	@staticmethod
	def getRelativePosition(source, relative_target):
		response = model_coordinates(source, relative_target)
		return response.pose.position


class FlightPlanning:
	target_positions = []
	trajectory = []
	
	def createTrajectory(self, randomState=False):
		self.target_positions = GazeboOperations.getTargetsPositions("coke_can")
		
		if randomState:
			random.shuffle(self.target_positions)
		
		startPos = Pose()
		startPos.position.x = 0
		startPos.position.y = 0
		startPos.position.z = ALTITUDE
		startPos.orientation.w = 1.0
		self.trajectory.append(startPos)
		
		for pos in self.target_positions:
			wayPoint = Pose()
			wayPoint.position.x = pos.x
			wayPoint.position.y = pos.y
			wayPoint.position.z = ALTITUDE
			wayPoint.orientation.w = 1.0
			self.trajectory.append(wayPoint)


class Quadrotor:
	motor_state = None
	pose_x = 0
	pose_y = 0
	pose_z = 0
	vel_linear_x = 0
	vel_linear_y = 0
	vel_linear_z = 0
	rot_z = 0
	
	planner = FlightPlanning()
	
	def flyTrajectory(self):
		self.planner.createTrajectory()
		trajectory_accelerations = self.trajectoryAccelerations()
		self.fly(trajectory_accelerations)
		
	@staticmethod
	def takeOffQuadrotor(x=0, y=0):
		pose_goal = PoseGoal()
		p = Pose()
		p.position.x = x
		p.position.y = y
		p.position.z = 0.8
		p.orientation.w = 1
		pose_goal.target_pose.pose = p
		pose_goal.target_pose.header.frame_id = "world"
		pose_client.send_goal(pose_goal)
		pose_client.wait_for_result()
	
	def landQuadrotor(self):
		landing_goal = LandingGoal()
		p = Pose()
		p.position.x = self.pose_x
		p.position.y = self.pose_y
		p.position.z = 0.3
		landing_goal.landing_zone.header.frame_id = "world"
		landing_goal.landing_zone.pose = p
		landing_client.send_goal(landing_goal)
		rospy.loginfo("QUADROTOR İNİYOR")
		landing_client.wait_for_result()
	
	@staticmethod
	def publishVelocity(twist):
		publisher = rospy.Publisher('/uav1/cmd_vel', Twist, queue_size=10)
		publisher.publish(twist)
	
	def changeMotorState(self, state=True):
		self.motor_state = enable_motors(state)
	
	def fly(self, trajectory_accelerations):
		velocity = Twist()
		
		for acc in trajectory_accelerations:
			start = time.time()
			counter = -1.0
			
			while time.time() - start < acc[1] - abs((math.sqrt(pow(acc[0][0], 2) + pow(acc[0][1], 2)) / acc[1])):
				if round((time.time() - start), 1) != counter:
					counter = round((time.time() - start), 1)
					velocity.linear.x += acc[0][0] / (math.floor(acc[1] * 10 / 2) + 0.5)
					velocity.linear.y += acc[0][1] / (math.floor(acc[1] * 10 / 2) + 0.5)
					self.publishVelocity(velocity)
			start = time.time()
			while time.time() - start < acc[1] - abs((math.sqrt(pow(acc[0][0], 2) + pow(acc[0][1], 2)) / acc[1])):
				if round((time.time() - start), 1) != counter:
					counter = round((time.time() - start), 1)
					velocity.linear.x += -acc[0][0] / (math.floor(acc[1] * 10 / 2) + 0.5)
					velocity.linear.y += -acc[0][1] / (math.floor(acc[1] * 10 / 2) + 0.5)
					self.publishVelocity(velocity)
			rospy.sleep(2)
	
	def trajectoryAccelerations(self):
		trajectory_acceleration = []
		
		for i in range(len(self.planner.trajectory) - 1):
			length = math.sqrt(pow(self.planner.trajectory[i].position.x - self.planner.trajectory[i+1].position.x, 2)
			                   + pow(self.planner.trajectory[i].position.y - self.planner.trajectory[i+1].position.y, 2))
			current_x = self.planner.trajectory[i].position.x
			current_y = self.planner.trajectory[i].position.y
			next_x = self.planner.trajectory[i+1].position.x
			next_y = self.planner.trajectory[i+1].position.y
			
			a_x = (next_x - current_x) / length
			a_y = (next_y - current_y) / length
			
			t = math.sqrt((2 * length) / math.sqrt(pow(a_x, 2) + pow(a_y, 2)))

			trajectory_acceleration.append(([a_x, a_y], t))
		return trajectory_acceleration
	
	