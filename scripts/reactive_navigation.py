#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class ReactiveNavigation():
    def __init__(self):
        # cmd_vel = Twist()
        self.laser_msg_0 = LaserScan()
        self.laser_msg_1 = LaserScan()
        self.laser_msg_2 = LaserScan()
        self.laser_msg_3 = LaserScan()
        self.robot_stopped = False
        self.obstacle_distance = 100

        # Params from config file
        self.active_robot = str(rospy.get_param(
            "reactive_controller_py/active_robot"))
        self.available_robots = list(rospy.get_param(
            "reactive_controller_py/available_robots"))

        # # Topics
        # self._cmd_topic = self.active_robot+"/cmd_vel"
        # self._laser_topic = self.active_robot+"/base_scan"

        # self.rospy_sub_laser = rospy.Subscriber(
        #     self._laser_topic, LaserScan, self.laser_cb, queue_size=1)
        # self.pub_CMD = rospy.Publisher(self._cmd_topic, Twist, queue_size=1)
        self.rate = rospy.Rate(5)

    def laser_cb_0(self, callback):
        self.laser_msg_0 = callback

    def laser_cb_1(self, callback):
        self.laser_msg_1 = callback

    def laser_cb_2(self, callback):
        self.laser_msg_2 = callback

    def laser_cb_3(self, callback):
        self.laser_msg_3 = callback

    def calculate_command(self):

        for robot in self.available_robots:

            if robot == "robot_0":
                self.publisher_robot(robot, self.laser_msg_0)
            elif robot == "robot_1":
                self.publisher_robot(robot, self.laser_msg_1)
            elif robot == "robot_2":
                self.publisher_robot(robot, self.laser_msg_2)
            elif robot == "robot_3":
                self.publisher_robot(robot, self.laser_msg_3)

        #     self.obstacle_distance = 100

    def publisher_robot(self, robot, lase_msg):
        cmd_vel = Twist()
        # Topics
        _cmd_topic = robot+"/cmd_vel"
        _laser_topic = robot+"/base_scan"

        if robot == "robot_0":
            rospy.Subscriber(_laser_topic, LaserScan,
                             self.laser_cb_0, queue_size=1)
        elif robot == "robot_1":
            rospy.Subscriber(_laser_topic, LaserScan,
                             self.laser_cb_1, queue_size=1)
        elif robot == "robot_2":
            rospy.Subscriber(_laser_topic, LaserScan,
                             self.laser_cb_2, queue_size=1)
        elif robot == "robot_3":
            rospy.Subscriber(_laser_topic, LaserScan,
                             self.laser_cb_3, queue_size=1)

        pub_CMD = rospy.Publisher(
            _cmd_topic, Twist, queue_size=1)

        if type(lase_msg.ranges) == tuple:
            self.obstacle_distance = min(lase_msg.ranges)
            if self.obstacle_distance > 1.0:
                cmd_vel.linear.x = 1.0
                cmd_vel.angular.z = 0.0
            else:
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = -1.0
        pub_CMD.publish(cmd_vel)

    def run(self):

        while rospy.is_shutdown:
            self.calculate_command()
            # print(self.cmd_vel)
            # self.pub_CMD.publish(self.cmd_vel)
            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("reactive_controller_py")
    controller = ReactiveNavigation()
    controller.run()
