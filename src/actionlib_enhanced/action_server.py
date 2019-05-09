#!/usr/bin/env python
# coding: utf-8

import threading
from multiprocessing import Queue
import rospy
from actionlib import ActionServer


class ActionServerCustom(object):
    def __init__(self, name, actionMsg, execute_cb=None, auto_start=True):

        self.execute_callback = execute_cb
        self.goal_callback = None
        self.preempt_callback = None
        self.queue = Queue()
        self.lock = threading.Lock()
        self.electionList = []

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
        :param gh: GoalHandler
        :param result: An optional result to send back to any clients of the goal
        :param text: An optionnal text associated to the SUCCESS status
        """
        if not result:
            result = self.get_default_result()
        self.lock.acquire()
        if gh.get_goal_id().id == self.electionList[0]:
            rospy.logdebug("{}, {}".format(gh.get_goal_status(), result))
            gh.set_succeeded(result, text)
            _ = self.electionList.pop(0)
        else:
            rospy.logerr("bad handling of goals, you should check if the goal is elected beforehand")
        self.lock.release()

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
            senderTemp = dict()
            goalTemp = []
            if not self.queue.empty():  # check queue
                goalID = 0
                while not self.queue.empty():  # handle current queue
                    goalName = self.queue.get_nowait()
                    goalTemp.append(goalName)
                    sender, _, stamp = goalName.split("-")
                    senderTemp.setdefault(sender, []).append([goalID, float(stamp)])
                    goalID += 1
            while senderTemp.values():  # handle goals extracted from queue
                for sender in senderTemp.keys():  # handle each first goal of each sender
                    stamps = [goal[1] for goal in senderTemp[sender]]
                    idx = min(xrange(len(stamps)), key=stamps.__getitem__)
                    goalID, _ = senderTemp[sender].pop(idx)
                    curGoal = goalTemp[goalID]
                    self.lock.acquire()
                    self.electionList.append(curGoal)
                    self.lock.release()
                    if not senderTemp[sender]:
                        del senderTemp[sender]

    def isElected(self, goalHandle):
        """
        :param goalHandle: goal to check if allowed to be finalized
        :return: boolean value to check if the goal is elected
        """
        isElected = False
        self.lock.acquire()
        if self.electionList:
            isElected = (goalHandle.get_goal_id().id == self.electionList[0])
        self.lock.release()
        return isElected
