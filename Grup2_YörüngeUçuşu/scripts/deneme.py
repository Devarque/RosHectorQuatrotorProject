#!/usr/bin/python3
import rospy
import UAV
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

velocity_publisher = rospy.Publisher('/uav1/cmd_vel', Twist, queue_size=10)


def posecallback(msg):
    quadrotor.pose_x = msg.pose.position.x
    quadrotor.pose_y = msg.pose.position.y
    quadrotor.pose_z = msg.pose.position.z
    
    
def velocityCallback(msg):
    quadrotor.vel_linear_x = msg.linear.x
    quadrotor.vel_linear_y = msg.linear.y

    
def main():
    rospy.init_node('quadrotor', anonymous=True)
    UAV.GazeboOperations.resetSim()

    rospy.Subscriber("/uav1/ground_truth_to_tf/pose", PoseStamped, posecallback)
    rospy.Subscriber("/uav1/cmd_vel", Twist, velocityCallback)

    rospy.loginfo("QUADROTOR HAVALANIYOR")
    quadrotor.takeOffQuadrotor()
    quadrotor.flyTrajectory()
    # quadrotor.randomPointsFlight(3)
    # quadrotor.randomDirectionFlight(3)
    quadrotor.landQuadrotor()


if __name__ == '__main__':
    quadrotor = UAV.Quadrotor()
    main()