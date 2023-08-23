#!/usr/bin/env python

"""
Created on 25/07/16
@author: Sammy Pfeiffer

Grasp controller to close with a determined error on position only
so to skip overheating.

"""

import rospy
from control_msgs.msg import JointTrajectoryControllerState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_srvs.srv import Empty, EmptyResponse
from std_msgs.msg import Bool
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure


class GripperGrasper(object):
    def __init__(self):
        rospy.loginfo("Initializing GripperGrasper...")
        # This node Dynamic params
        self.ddr = DDynamicReconfigure("grasper")
        self.max_position_error = self.ddr.add_variable("max_position_error",
                                                        "Max absolute value of controller state of any joint to stop closing",
                                                        0.0015, 0.00001, 0.045)
        self.timeout = self.ddr.add_variable("timeout",
                                             "timeout for the closing action",
                                             5.0, 0.0, 30.0)
        self.closing_time = self.ddr.add_variable("closing_time",
                                                  "Time for the closing goal",
                                                  2.0, 0.01, 30.0)
        self.rate = self.ddr.add_variable("rate",
                                          "Rate Hz at which the node closing will do stuff",
                                          5, 1, 50)

        self.tolerance = self.ddr.add_variable("tolerance",
                                          "Tolerance of the gripper to detect that the object slipped",
                                          0.0005, 0.0001, 0.002)

        self.ddr.start(self.ddr_cb)
        rospy.loginfo("Initialized dynamic reconfigure on: " + str(rospy.get_name()))

        # Set time to open gripper
        self.opening_time = 0.2

        # Subscriber to the gripper state
        self.last_state = None
        self.controller_name = rospy.get_param("~controller_name", None)
        if not self.controller_name:
            rospy.logerr("No controller name found in param: ~controller_name")
            exit(1)
        self.real_controller_name = rospy.get_param("~real_controller_name", None)
        if not self.real_controller_name:
            rospy.logerr("No controller name found in param: ~real_controller_name")
            exit(1)
        self.real_joint_names = rospy.get_param("~real_joint_names", None)
        if not self.real_joint_names:
            rospy.logerr(
                "No real joint names given in param: ~real_joint_names")
            exit(1)

        self.state_sub = rospy.Subscriber('/' + self.real_controller_name + '_controller/state',
                                          JointTrajectoryControllerState,
                                          self.state_cb,
                                          queue_size=1)
        rospy.loginfo("Subscribed to topic: " + str(self.state_sub.resolved_name))

        # Publisher on the gripper command topic
        self.cmd_pub = rospy.Publisher('/' + self.real_controller_name + '_controller/command',
                                       JointTrajectory,
                                       queue_size=1)
        rospy.loginfo("Publishing to topic: " + str(self.cmd_pub.resolved_name))

        # Grasping service to offer
        self.grasp_srv = rospy.Service('/' + self.controller_name + '_controller/grasp',
                                       Empty,
                                       self.grasp_cb)
        rospy.loginfo("Offering grasp service on: " + str(self.grasp_srv.resolved_name))

        # Releasing service to offer
        self.release_srv = rospy.Service('/' + self.controller_name + '_controller/release',
                                         Empty,
                                         self.release_cb)
        rospy.loginfo("Offering release service on: " +
                      str(self.release_srv.resolved_name))

        # Publish a boolean to know if an object is grasped or not
        self.pub_js = rospy.Publisher("{}/is_grasped".format(self.controller_name), Bool , queue_size=1)
        self.is_grasped_msg = Bool()
        self.on_optimal_close = False
        self.on_optimal_open = False

        rospy.loginfo("Done initializing GripperGrasper!")

    def ddr_cb(self, config, level):
        self.max_position_error = config['max_position_error']
        self.timeout = config['timeout']
        self.closing_time = config['closing_time']
        self.rate = config['rate']
        self.tolerance = config['tolerance']
        return config

    def state_cb(self, data):
        self.last_state = data
        if self.on_optimal_close:
            self.is_grasped_msg.data = True
            if -self.last_state.error.positions[0] < self.tolerance and -self.last_state.error.positions[1] < self.tolerance:
                self.is_grasped_msg.data = False
                self.on_optimal_close = False
        else:
            self.is_grasped_msg.data = False
        self.pub_js.publish(self.is_grasped_msg)

    def grasp_cb(self, req):
        rospy.logdebug("Received grasp request!")
        # From wherever we are close gripper

        # Keep closing until the error of the state reaches
        # max_position_error on any of the gripper joints
        # or we reach timeout
        initial_time = rospy.Time.now()
        closing_amount = [0.0, 0.0]
        # If statement in case the service is called again after a sucessful grasp
        if not self.on_optimal_close:
            # Initial command, wait for it to do something
            rospy.loginfo("Closing: " + str(closing_amount))
            self.send_joint_trajectory(closing_amount, self.closing_time)
            self.on_optimal_open = False
            rospy.sleep(self.closing_time)
        r = rospy.Rate(self.rate)
        while not rospy.is_shutdown() and (rospy.Time.now() - initial_time) < rospy.Duration(self.timeout) and not self.on_optimal_close:
            if -self.last_state.error.positions[0] > self.max_position_error:
                rospy.logdebug("Over error joint 0...")
                closing_amount = self.get_optimal_close()
                self.on_optimal_close = True

            elif -self.last_state.error.positions[1] > self.max_position_error:
                rospy.logdebug("Over error joint 1...")
                closing_amount = self.get_optimal_close()
                self.on_optimal_close = True
            rospy.loginfo("Closing: " + str(closing_amount))
            self.send_joint_trajectory(closing_amount, self.closing_time)
            r.sleep()

        return EmptyResponse()

    def release_cb(self, req):
        rospy.logdebug("Received release request!")

        # From wherever we are open gripper
        opening_amount = [0.044, 0.044]

        # If statement in case the service is called again after a sucessful grasp
        if not self.on_optimal_open:
            # Initial command, wait for it to do something
            self.send_joint_trajectory(opening_amount, self.opening_time)
            self.on_optimal_close = False
            rospy.sleep(self.opening_time)

        return EmptyResponse()

    def get_optimal_close(self):
        optimal_0 = self.last_state.actual.positions[0] - self.max_position_error
        optimal_1 = self.last_state.actual.positions[1] - self.max_position_error
        return [optimal_0, optimal_1]

    def send_joint_trajectory(self, joint_positions, execution_time):
        jt = JointTrajectory()
        jt.joint_names = self.real_joint_names
        p = JointTrajectoryPoint()
        p.positions = joint_positions
        p.time_from_start = rospy.Duration(execution_time)
        jt.points.append(p)

        self.cmd_pub.publish(jt)


if __name__ == '__main__':
    rospy.init_node('gripper_grasping')
    gg = GripperGrasper()
    rospy.spin()
