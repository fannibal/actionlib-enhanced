#!/usr/bin/env python
# coding: utf-8

import rospy
import actionlib
from actionlib_enhanced import ActionServerCustom
from actionlib_enhanced.msg import BasicComAction, BasicComResult

class ActionServer():

    TAG = "TestActionlibServer"

    def __init__(self):
        rospy.init_node("test_actionlib_server")
        self.ns = rospy.get_param("~namespace", "test")
        # Actionlib
        self.actionServer = actionlib.SimpleActionServer("/{}/basic_com".format(self.ns),
                                                         BasicComAction,
                                                         self.onRequest,
                                                         auto_start=False)
        self.actionServer.start()
        # ActionlibCustom
        self.actionServerCustom = ActionServerCustom("/{}/enhanced_com".format(self.ns),
                                                     BasicComAction,
                                                     self.onRequest2,
                                                     auto_start=False)
        self.actionServerCustom.start()

        rospy.on_shutdown(self.onShutdown)
        rospy.loginfo("{} initialized".format(self.TAG))

    def onShutdown(self):
        pass

    def onRequest(self, goal):
        """
        send back the goal it receive
        :param goal: BasicComGoal
        """
        result = BasicComResult(numReceived=goal.numRequest)
        rospy.sleep(self.delay)
        self.actionServer.set_succeeded(result)

    def onRequest2(self, goalHandle):  # multithreaded on requests
        """
        send back the goal it receive
        :param goalHandle: goalHandle containing the BasicComGoal
        """
        # ID info if needed (/!\ : ch3.1 in http://wiki.ros.org/actionlib/DetailedDescription)
        who, nbrRequest, stamp = goalHandle.get_goal_id().id.split("-")

        goal = goalHandle.get_goal()
        result = BasicComResult(numReceived=goal.numRequest)
        rospy.sleep(rospy.Duration(5))
        rate = rospy.Rate(100)
        while not self.actionServerCustom.isElected(goalHandle):
            rate.sleep()
        self.actionServerCustom.set_succeeded(goalHandle, result)


if __name__ == "__main__":
    actionServer = ActionServer()
    rospy.spin()
