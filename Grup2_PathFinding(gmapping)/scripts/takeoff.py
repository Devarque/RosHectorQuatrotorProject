#!/usr/bin/python3
import rospy
import UAV
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped

velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


def posecallback(msg):
    quadrotor.pose_x = msg.pose.position.x
    quadrotor.pose_y = msg.pose.position.y
    quadrotor.pose_z = msg.pose.position.z
    
    
def velocityCallback(msg):
    quadrotor.vel_linear_x = msg.linear.x
    quadrotor.vel_linear_y = msg.linear.y

    
def main():
    rospy.init_node('quadrotor', anonymous=True)

    quadrotor.takeOffQuadrotor()


if __name__ == '__main__':
    quadrotor = UAV.Quadrotor()
    main()
