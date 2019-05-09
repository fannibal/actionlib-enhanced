#!/usr/bin/env python
# coding: utf-8

import random
import rospy
import actionlib
from actionlib_enhanced.msg import BasicComAction, BasicComGoal
from cv_bridge import CvBridge
from actionlib_enhanced import ActionClientCustom


class ActionClient(object):

    TAG = "TestActionlibClient"

    def __init__(self):
        rospy.init_node("test_actionlib_client", anonymous=True)
        self.ns = rospy.get_param("~namespace", "test")
        rospy.on_shutdown(self.onShutdown)
        self.bridge = CvBridge()

        rospy.set_param('actionlib_client_sub_queue_size', 10)
        self.goal = BasicComGoal()
        # Simple Actionlib client
        self.actionClient = actionlib.SimpleActionClient("/{}/basic_com".format(self.ns), BasicComAction)
        if not self.actionClient.wait_for_server(timeout=rospy.Duration(10)):
            rospy.signal_shutdown("Simple server not started")
        # Custom Actionlib client
        self.actionClientCustom = ActionClientCustom("/{}/enhanced_com".format(self.ns), BasicComAction)
        if not self.actionClientCustom.wait_for_server(timeout=rospy.Duration(10)):
            rospy.signal_shutdown("Server custom not started")

        rospy.loginfo("{} initialized".format(self.TAG))

    def onShutdown(self):
        pass

    def sendRequest(self, count):
        """
        :param count: number corresponding to the BasicComGoal message
        """
        self.goal.numRequest = count
        self.actionClient.send_goal(self.goal, done_cb=lambda x, y: self.doneCallback(x, y))

    def sendRequest2(self, count):
        """
        :param count: number corresponding to the BasicComGoal message
        """
        self.goal.numRequest = count

        # the ID returned from send_goal can be compared to the doneCallback 3rd arg
        _ = self.actionClientCustom.send_goal(self.goal, done_cb=lambda x, y, z: self.doneCallback(x, y, z))

    def doneCallback(self, goalStatus, data, name=None):
        """
        :param goalStatus: Status of the received goal
        :param data: BasicComResult message
        :param name: ID of the goal, useful for multithreaded requests
        """
        if goalStatus == 3:  # SUCCESS
            rospy.loginfo("received {}".format(data.numReceived))


if __name__ == "__main__":
    actionClient = ActionClient()
    for i in range(1, 21):
        rospy.loginfo("send {}".format(i))
        actionClient.sendRequest2(i)
        rospy.sleep(random.randint(1, 5))
    rospy.spin()
