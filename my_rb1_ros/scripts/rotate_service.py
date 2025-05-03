#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from my_rb1_ros.srv import Rotate, RotateResponse
import math

class RotateRobotService:
    def __init__(self):
        rospy.init_node('rotate_robot_service_server')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.current_yaw = 0.0

        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.service = rospy.Service('/rotate_robot', Rotate, self.handle_rotate_robot)

        rospy.loginfo("Service Ready")
        rospy.spin()

    def odom_callback(self, msg):
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        self.current_yaw = yaw

    def handle_rotate_robot(self, req):
        target_degrees = req.degrees
        target_radians = math.radians(target_degrees)

        if target_degrees != 90 and target_degrees != -90:
            rospy.logwarn("Invalid Input")
            return RotateResponse(result="Invalid Input")

        rospy.loginfo(f"Service Requested")

        start_yaw = self.current_yaw
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.3 if target_radians > 0 else -0.3

        rate = rospy.Rate(10)
        turned_angle = 0.0

        while not rospy.is_shutdown():
            self.pub.publish(twist)
            rate.sleep()

            # Calculate: difference between current yaw and start yaw
            delta_yaw = self.current_yaw - start_yaw

            # Normalize to range -pi ~ pi
            delta_yaw = math.atan2(math.sin(delta_yaw), math.cos(delta_yaw))
            turned_angle = delta_yaw

            # Stop when the target rotation angle is reached
            if abs(turned_angle) >= abs(target_radians):
                break

        # Stop rotation
        twist.angular.z = 0.0
        self.pub.publish(twist)

        rospy.loginfo("Service Completed")
        return RotateResponse(result="Rotation completed successfully!")

if __name__ == '__main__':
    RotateRobotService()
