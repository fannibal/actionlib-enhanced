#!/usr/bin/env python
# coding: utf-8

import rospy
import actionlib
import random
from actionlib_enhanced import EnhancedActionServer
from actionlib_enhanced.msg import BasicComAction, BasicComResult, BasicComGoal

class ActionServer():

    TAG = "TestActionlibServer"

    def __init__(self):
        rospy.init_node("test_actionlib_server")
        self.ns = rospy.get_param("~namespace", "test")
        self.delay = rospy.Duration(5)
        # Actionlib Enhanced
        self.actionServerCustom = EnhancedActionServer("/{}/enhanced_com".format(self.ns),
                                                     BasicComAction,
                                                     self.onRequest,
                                                     auto_start=False)
        self.actionServerCustom.start()

        rospy.on_shutdown(self.onShutdown)
        rospy.loginfo("{} initialized".format(self.TAG))

    def onShutdown(self):
        pass

    def onRequest(self, goal):  # multithreaded on requests
        """
        sends back the goal it receives or raises an error
        :type goal: BasicComGoal
        :param goal: goal containing the BasicComGoal
        """
        rospy.loginfo("=====================================================")
        if goal.numRequest % 3 == 0:
            rospy.loginfo("Received goal {} => Raising error".format(goal.numRequest))
            raise Exception("Random error")
        elif goal.numRequest % 3 == 2:
            rospy.loginfo("Received goal {} => No call to succeeded".format(goal.numRequest))
        elif goal.numRequest % 3 == 1:
            rospy.loginfo("Received goal {} => Success".format(goal.numRequest))
            result = BasicComResult(numReceived=goal.numRequest)
            self.actionServerCustom.set_succeeded(result)


if __name__ == "__main__":
    actionServer = ActionServer()
    rospy.spin()
