#!/usr/bin/env python
# coding: utf-8

import random
import rospy
from actionlib_enhanced.msg import BasicComAction, BasicComGoal
from actionlib_msgs.msg import GoalStatus
from cv_bridge import CvBridge
from actionlib_enhanced import EnhancedActionClient


class ActionClient(object):

    TAG = "TestActionlibClient"

    def __init__(self):
        rospy.init_node("test_actionlib_client", anonymous=True)
        self.ns = rospy.get_param("~namespace", "test")
        rospy.on_shutdown(self.onShutdown)
        self.bridge = CvBridge()

        rospy.set_param('actionlib_client_sub_queue_size', 10)
        self.goal = BasicComGoal()
        # Custom Actionlib client
        self.actionClientCustom = EnhancedActionClient("/{}/enhanced_com".format(self.ns), BasicComAction)
        if not self.actionClientCustom.wait_for_server(timeout=rospy.Duration(10)):
            rospy.signal_shutdown("Server not started")

        rospy.loginfo("{} initialized".format(self.TAG))

    def onShutdown(self):
        pass

    def sendRequest(self, count):
        """
        :param count: number corresponding to the BasicComGoal message
        """
        self.goal.numRequest = count

        # the ID returned from send_goal can be compared to the doneCallback 3rd arg
        _ = self.actionClientCustom.send_goal(self.goal, done_cb=self.doneCallback)

    def doneCallback(self, goalStatus, data, name=None):
        """
        :param goalStatus: Status of the received goal
        :param data: BasicComResult message
        :param name: ID of the goal, useful for multithreaded requests
        """
        if goalStatus == GoalStatus.SUCCEEDED:
            rospy.loginfo("Received {}".format(data.numReceived))
        elif goalStatus == GoalStatus.ABORTED:
            rospy.logwarn("Server aborted goal")


if __name__ == "__main__":
    actionClient = ActionClient()
    for i in range(1, 6):
        rospy.loginfo("send {}".format(i))
        actionClient.sendRequest(i)
        rospy.sleep(random.randint(1, 3))
    rospy.spin()
