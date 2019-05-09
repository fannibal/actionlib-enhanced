#!/usr/bin/env python
# coding: utf-8

import threading
from multiprocessing import Queue
import rospy
from actionlib import ActionServer
from actionlib.action_server import ServerGoalHandle


class ActionServerCustom(object):
    def __init__(self, name, actionMsg, execute_cb=None, auto_start=True):

        self.execute_callback = execute_cb
        self.goal_callback = None
        self.preempt_callback = None
        self.queue = Queue()
        self.lock = threading.Lock()
        self.electionList = dict()

        # create the action server
        self.action_server = ActionServer(name, actionMsg, self.get_next_gh, self.internal_preempt_callback, auto_start)

        # create the scheduling daemon thread
        if "schedThread" not in [t.name for t in threading.enumerate()]:
            workingThread = threading.Thread(name="schedThread", target=self.scheduling)
            workingThread.setDaemon(True)
            workingThread.start()

    def start(self):
        """
        start server if not already started
        """
        self.action_server.start()

    def set_succeeded(self, gh, result=None, text=""):
        """
        :type gh: ServerGoalHandle
        :param gh: GoalHandler
        :param result: An optional result to send back to any clients of the goal
        :param text: An optionnal text associated to the SUCCESS status
        """
        if result is None:
            result = self.get_default_result()
        if self.isElected(gh):
            sender, _, _ = gh.get_goal_id().id.split("-")
            self.lock.acquire()
            self.electionList.get(sender).pop(0)
            gh.set_succeeded(result, text)
            rospy.logdebug("{}, {}".format(gh.get_goal_status(), result))
            self.lock.release()
        else:
            rospy.logerr("bad handling of goals, you should check if the goal is elected beforehand")

    def get_default_result(self):
        """
        :return: default content for result message
        """
        return self.action_server.ActionResultType()

    def get_next_gh(self, goal):
        """
        start new thread on new goal reception
        :param goal:
        :return:
        """
        try:
            rospy.logdebug("A new goal %s has been recieved by the single goal action server", goal.get_goal_id().id)
            if self.execute_callback:
                goal.status_tracker.status.status = 6
                self.queue.put(goal.get_goal_id().id, block=False)
                try:
                    t = threading.Thread(target=self.execute_callback, args=(goal,))
                    t.setDaemon(True)
                    t.start()
                except threading.ThreadError:
                    rospy.logerr("Error: unable to start thread")
            else:
                rospy.logerr("DEFINE an execute callback")

        except Exception as e:
            rospy.logerr("CustomActionServer.internal_goal_callback - exception %s", str(e))

    # no preemption needed
    def internal_preempt_callback(self):
        pass

    def scheduling(self):
        """
        method for the daemon thread to schedule the goal threads
        """
        while True:
            goalId = self.queue.get()
            sender, _, stamp = goalId.split("-")
            stamp = float(stamp)
            self.lock.acquire()
            if self.electionList.has_key(sender):
                senderList = self.electionList.get(sender, [])
                senderList.append((stamp, goalId))
                senderList.sort(key=lambda x : x[0])
            else:
                self.electionList[sender] = [(stamp, goalId)]
            self.lock.release()

    def isElected(self, goalHandle):
        """
        :type goalHandle: ServerGoalHandle
        :param goalHandle: goal to check if allowed to be finalized
        :return: boolean value to check if the goal is elected
        """
        isElected = False
        sender, _, _ = goalHandle.get_goal_id().id.split("-")
        self.lock.acquire()
        senderList = self.electionList.get(sender)
        if senderList:
            electedGoal = senderList[0][1]
            isElected = (goalHandle.get_goal_id().id == electedGoal)
        self.lock.release()
        return isElected
