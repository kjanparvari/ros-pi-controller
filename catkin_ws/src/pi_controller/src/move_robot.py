#! /usr/bin/env python3
from math import pi, sin, cos, atan2, sqrt
import numpy as np
from geometry_msgs.msg import Twist
import rospy
from tf.transformations import euler_from_quaternion
from gazebo_msgs.srv import GetModelState
import matplotlib.pyplot as plt


class Robot:
    def __init__(self):
        self.kp = .3
        self.ki = .02
        self.kt = 0.85
        self.d_star = 0.25
        self.dt = .1
        self.plan = []
        self.path = []
        self.theta = 0.

        rospy.init_node('MoveRobot', anonymous=False)
        rospy.loginfo("CTRL + C to stop the turtlebot")
        rospy.on_shutdown(self.shutdown)

        rospy.loginfo("Wait for service ....")
        rospy.wait_for_service("gazebo/get_model_state")
        self.get_ground_truth = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.vel = Twist()

    def path_plan(self, _type="spiral"):
        if _type == "spiral":
            self.d_star = .25
            number_of_points = 250
            growth_factor = 0.2
            for i in range(number_of_points):
                w = (i / (number_of_points / 10)) * pi
                x = (w * growth_factor) * cos(w)
                y = (w * growth_factor) * sin(w)
                self.plan.append((x, y))

        elif _type == "ellipse":
            self.d_star = .1
            number_of_points = 100
            number_of_rounds = 5
            center_x, center_y, radius_x, radius_y = 0, 0, 1, 3
            t = np.linspace(0, 2 * pi, number_of_points)
            for _ in range(number_of_rounds):
                for i in range(len(t)):
                    self.plan.append((center_x + radius_x * np.cos(t[i]), center_y + radius_y * np.sin(t[i])))

    def shutdown(self):
        rospy.loginfo("Shutdown!")
        rospy.loginfo("Stop TurtleBot")
        self.vel.linear.x = 0.
        self.vel.angular.z = 0.
        self.vel_pub.publish(self.vel)
        rospy.sleep(1)

    def run(self):
        try:
            self.path_plan("spiral")
            _sum = last_theta_star = last_yaw = 0
            reached_first_goal = False
            path_error = []
            total_error = 0.
            for goal_x, goal_y in self.plan:

                while True:
                    if rospy.is_shutdown():
                        exit(-1)
                    ground_truth = self.get_ground_truth("turtlebot3_waffle_pi", "world")
                    pose_x = ground_truth.pose.position.x
                    pose_y = ground_truth.pose.position.y
                    ori_x = ground_truth.pose.orientation.x
                    ori_y = ground_truth.pose.orientation.y
                    ori_z = ground_truth.pose.orientation.z
                    ori_w = ground_truth.pose.orientation.w
                    roll, pitch, yaw = euler_from_quaternion((ori_x, ori_y, ori_z, ori_w))
                    self.path.append((pose_x, pose_y))
                    rospy.loginfo(f"pose_x: {pose_x}, pose_y: {pose_y}, yaw: {yaw}")

                    self.theta = last_yaw = yaw = self.adjust_angle(yaw, last_yaw)
                    distance = self.distance(goal_x, goal_y, pose_x, pose_y)
                    if distance < self.d_star:
                        reached_first_goal = True
                        rospy.loginfo("\n\n Reached to the Goal \n")
                        break

                    theta_star = atan2(goal_y - pose_y, goal_x - pose_x)
                    last_theta_star = theta_star = self.adjust_angle(theta_star, last_theta_star)
                    gamma = self.kt * (theta_star - yaw)
                    error = distance - self.d_star

                    if reached_first_goal:
                        path_error.append(error)
                        total_error += error * self.dt

                    _sum += error * self.dt
                    self.vel.linear.x = error * self.kp + _sum * self.ki
                    self.vel.angular.z = gamma
                    self.vel_pub.publish(self.vel)
                    rospy.sleep(self.dt)
            plt.plot(path_error)
            plt.title(f"total error: {total_error}")
            plt.show()

        except Exception as e:
            print(e)
            rospy.loginfo("node shutdown")

    @staticmethod
    def adjust_angle(angle, last):
        while angle < last - pi:
            angle += 2 * pi
        while angle > last + pi:
            angle -= 2 * pi
        return angle

    @staticmethod
    def distance(x1, y1, x2, y2):
        return sqrt(((x1 - x2) ** 2) + ((y1 - y2) ** 2))


if __name__ == '__main__':
    robot = Robot()
    robot.run()
