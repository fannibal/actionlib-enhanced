#!/usr/bin/env python
# coding: utf-8

import rospy
import actionlib
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
        send back the goal it receive
        :type goal: BasicComGoal
        :param goal: goal containing the BasicComGoal
        """
        result = BasicComResult(numReceived=goal.numRequest)
        rospy.sleep(self.delay)
        self.actionServerCustom.set_succeeded(result)


if __name__ == "__main__":
    actionServer = ActionServer()
    rospy.spin()
