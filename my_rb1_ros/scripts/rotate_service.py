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

        if target_degrees == 0:
            rospy.logwarn("Zero rotation requested.")
            return RotateResponse(result="No rotation needed.")

        rospy.loginfo(f"Service Requested: rotate {target_degrees} degrees")

        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 1.0 if target_radians > 0 else -1.0

        rate = rospy.Rate(10)

        last_yaw = self.current_yaw
        turned_angle = 0.0

        success = False
        while not rospy.is_shutdown():
            self.pub.publish(twist)
            rate.sleep()

            # Calculate incremental rotation
            delta_yaw = self.current_yaw - last_yaw
            delta_yaw = math.atan2(math.sin(delta_yaw), math.cos(delta_yaw))
            turned_angle += delta_yaw
            last_yaw = self.current_yaw

            # Stop when the target rotation angle is reached
            if abs(turned_angle) >= abs(target_radians):
                success = True
                break

        # Stop rotation
        twist.angular.z = 0.0
        self.pub.publish(twist)

        if success:
            rospy.loginfo("Service Completed")
            return RotateResponse(result="Rotation completed successfully!")
        else:
            rospy.logwarn("Rotation interrupted before completion")
            return RotateResponse(result="Rotation interrupted or failed.")

if __name__ == '__main__':
    RotateRobotService()
