#!/usr/bin/env python
# coding: utf-8

import random
import rospy
import threading
from actionlib_enhanced.msg import BasicComAction, BasicComGoal
from actionlib_enhanced import EnhancedActionClient


class ActionClient(object):

    TAG = "TestActionlibClientMulti"
    goal = BasicComGoal()

    def __init__(self):
        rospy.init_node("test_actionlib_client_multi", anonymous=True)
        self.ns = rospy.get_param("~namespace", "test")
        rospy.on_shutdown(self.onShutdown)
        rospy.set_param("actionlib_client_sub_queue_size", 10)
        # Custom Actionlib client
        self.actionClientCustom = EnhancedActionClient("/{}/enhanced_com".format(self.ns), BasicComAction)
        self.actionClientCustom.wait_for_server()
        rospy.loginfo("{} initialized".format(self.TAG))

    def onShutdown(self):
        pass

    def sendRequest(self, count):
        """
        :param count: number corresponding to the BasicComGoal message
        """
        self.goal.numRequest = count

        # the ID returned from send_goal can be compared to the doneCallback 3rd arg
        _ = self.actionClientCustom.send_goal_and_wait(self.goal)
        result = self.actionClientCustom.get_result()
        rospy.loginfo("{}".format(result))

    @staticmethod
    def doneCallback(goalStatus, data, name=None):
        """
        :param goalStatus: Status of the received goal
        :param data: BasicComResult message
        :param name: ID of the goal, useful for multithreaded requests
        """
        if goalStatus == 3:  # SUCCESS
            rospy.loginfo("Waiting for something :)")
            rospy.sleep(5)
            rospy.loginfo("received {}".format(data.numReceived))


if __name__ == "__main__":
    actionClient = ActionClient()

    for i in range(1, 4):
        rospy.loginfo("send {}".format(i))
        thread = threading.Thread(target=actionClient.sendRequest, args=(i,))
        thread.setDaemon(True)
        thread.start()
        rospy.sleep(random.randint(1, 3))
    rospy.spin()
