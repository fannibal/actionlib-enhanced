#!/usr/bin/env python
# coding: utf-8

import random
import rospy
import threading
from actionlib_enhanced.msg import BasicComAction, BasicComGoal
from actionlib_enhanced import EnhancedActionClient
from actionlib_msgs.msg import GoalStatus


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
        status = self.actionClientCustom.send_goal_and_wait(self.goal)
        result = self.actionClientCustom.get_result()
        if status == GoalStatus.SUCCEEDED:
            rospy.loginfo("Received: {}".format(result))
        else:
            rospy.logerr("Goal status: {}".format(status))


if __name__ == "__main__":
    actionClient = ActionClient()

    for i in range(1, 7):
        rospy.loginfo("Sending {}".format(i))
        thread = threading.Thread(target=actionClient.sendRequest, args=(i,))
        thread.setDaemon(True)
        thread.start()
        rospy.sleep(random.randint(1, 3))
    rospy.spin()
