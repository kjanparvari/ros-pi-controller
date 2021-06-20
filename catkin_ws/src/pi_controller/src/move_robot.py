#! /usr/bin/env python3
from math import pi, sin, cos, atan2, sqrt
import numpy as np
from geometry_msgs.msg import Twist
import rospy
from tf.transformations import euler_from_quaternion
from gazebo_msgs.srv import GetModelState


class Robot:
    def __init__(self):
        self.kp = .77
        self.ki = .01
        self.kt = .9
        self.d_star = 0.25
        self.dt = .1
        self.l = 0.288
        self.plan = []
        self.path = [(0., 0.)]
        self.theta = 0.

        rospy.init_node('MoveRobot', anonymous=False)

        rospy.loginfo("CTRL + C to stop the turtlebot")

        rospy.on_shutdown(self.shutdown)

        print("Wait for service ....")
        rospy.wait_for_service("gazebo/get_model_state")
        self.get_ground_truth = rospy.ServiceProxy("/gazebo/get_model_state", GetModelState)
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=5)
        self.vel = Twist()
        print("moveRobot node is setup!")

    def path_plan(self, type=""):
        if type == "spiral":
            number_of_points = 300
            growth_factor = 0.5
            for i in range(number_of_points):
                vt = (i / (number_of_points / 10)) * pi
                x = (vt * growth_factor) * cos(vt)
                y = (vt * growth_factor) * sin(vt)

                self.plan.append((x, y))

        elif type == "ellipse":
            center_x, center_y, radius_x, radius_y = 0, 0, 1, 3
            t = np.linspace(0, 2 * pi, 100)
            for i in range(len(t)):
                self.plan.append((center_x + radius_x * np.cos(t[i]), center_y + radius_y * np.sin(t[i])))

    def shutdown(self):
        print("Shutdown!")
        rospy.loginfo("Stop TurtleBot")
        self.vel.linear.x = 0.0
        self.vel.angular.z = 0.0
        self.vel_pub.publish(self.vel)
        rospy.sleep(1)

    def run(self):
        try:
            self.path_plan()
            _sum = last_theta_star = last_yaw = 0
            for gx, gy in self.plan:
                if rospy.is_shutdown():
                    exit(-1)
                while True:
                    gt = self.get_ground_truth("turtlebot3_waffle_pi", "world")
                    pose_x = gt.pose.position.x
                    pose_y = gt.pose.position.y
                    ori_x = gt.pose.orientation.x
                    ori_y = gt.pose.orientation.y
                    ori_z = gt.pose.orientation.z
                    ori_w = gt.pose.orientation.w
                    roll, pitch, yaw = euler_from_quaternion(ori_x, ori_y, ori_z, ori_w)

                    while yaw < last_yaw - pi:
                        yaw += 2 * pi
                    while yaw > last_yaw + pi:
                        yaw -= 2 * pi
                    last_yaw = yaw

                    self.theta = yaw

                    distance = self.distance(gx, gy, pose_x, pose_y)

                    if rospy.is_shutdown():
                        exit(-1)
                    if distance < self.d_star:
                        print("\n Reached to the Goal \n")
                        break
                    theta_star = atan2(gy - pose_y, gx - pose_x)

                    while theta_star < last_theta_star - pi:
                        theta_star += 2 * pi
                    while theta_star > last_theta_star + pi:
                        theta_star -= 2 * pi
                    last_theta_star = theta_star

                    gamma = self.kt * (theta_star - yaw)
                    error = distance - self.d_star
                    _sum += error * self.dt
                    self.vel.linear.x = error * self.kp + _sum * self.ki
                    self.path.append((pose_x, pose_y))
                    self.vel.angular.z = gamma
                    self.vel_pub.publish(self.vel)
                    rospy.sleep(self.dt)

        except Exception as e:
            print(e)
            rospy.loginfo("move_robot node terminated")

    @staticmethod
    def distance(x1, y1, x2, y2):
        return sqrt(((x1 - x2) ** 2) + ((y1 - y2) ** 2))


if __name__ == '__main__':
    Robot().run()
