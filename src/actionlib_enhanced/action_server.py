#!/usr/bin/env python
# coding: utf-8

import threading
from multiprocessing import Queue
import rospy
from actionlib import ActionServer
from actionlib.action_server import ServerGoalHandle


class EnhancedActionServer(object):

    def __init__(self, name, actionMsg, execute_cb=None, auto_start=True, call_on_elected=False):
        self.execute_callback = execute_cb
        self.goal_callback = None
        self.preempt_callback = None
        self.call_on_elected = call_on_elected
        self.queue = Queue()
        self.lock = threading.Lock()
        self.electionList = dict()
        self.ghDict = dict()

        # create the action server
        self.action_server = ActionServer(name, actionMsg, self.get_next_gh, self.internal_preempt_callback, auto_start)

        # create the scheduling daemon thread
        if name+"/schedThread" not in [t.name for t in threading.enumerate()]:
            workingThread = threading.Thread(name=name+"/schedThread", target=self.scheduling)
            workingThread.setDaemon(True)
            workingThread.start()

    def start(self):
        """
        start server if not already started
        """
        self.action_server.start()

    def _set_gh_state(self, result=None, text="", succeeded=True):
        """
        :param result: An optional result to send back to any clients of the goal
        :param text: An optionnal text associated to the SUCCESS|ABORTED status
        :param succeeded: the goal handle is set to SUCCEEDED if true, else ABORTED
        """
        threadId = threading.current_thread().ident
        self.lock.acquire()
        if not self.ghDict.has_key(threadId):
            rospy.logwarn("set_succeeded called more than once in an actionlib server callback or outside one, ignoring call")
            return
        gh = self.ghDict[threadId]
        if result is None:
            result = self.get_default_result()
        rate = rospy.Rate(1000)
        self.lock.release()
        while not self.is_elected(gh):
            rate.sleep()
        sender, _, _ = gh.get_goal_id().id.split("-")
        self.electionList.get(sender).pop(0)
        if succeeded:
            gh.set_succeeded(result, text)
        else:
            gh.set_aborted(result, text)
        self.lock.acquire()
        self.ghDict.pop(threadId)
        self.lock.release()

    def set_succeeded(self, result=None, text=""):
        self._set_gh_state(result, text)

    def set_aborted(self, result=None, text=""):
        self._set_gh_state(result, text, succeeded=False)
        
    def get_default_result(self):
        """
        :return: default content for result message
        """
        return self.action_server.ActionResultType()

    def execute_callback_on_elected(self, goal_handle):
        rate = rospy.Rate(1000)
        while not self.is_elected(goal_handle):
            rate.sleep()
        self.execute_server_callback(goal_handle)

    def execute_server_callback(self, goal_handle):
        self.lock.acquire()
        self.ghDict[threading.current_thread().ident] = goal_handle
        self.lock.release()
        try:
            self.execute_callback(goal_handle.get_goal())
        except Exception as e:
            rospy.logerr("Error in the actionlib server callback: {}".format(e))
        finally:
            if self.ghDict.has_key(threading.current_thread().ident):
                rospy.logwarn("The actionlib server callback did not set the goal as succeeded, sending unsuccessful result")
                self.set_aborted()

    def get_next_gh(self, goal_handle):
        """
        start new thread on new goal reception
        :type goal_handle: ServerGoalHandle
        :return:
        """
        try:
            rospy.logdebug("A new goal %s has been recieved by the single goal action server", goal_handle.get_goal_id().id)
            if self.execute_callback:
                goal_handle.status_tracker.status.status = 6
                self.queue.put(goal_handle.get_goal_id().id, block=False)
                try:
                    if self.call_on_elected:
                        t = threading.Thread(target=self.execute_callback_on_elected, args=(goal_handle,))
                    else:
                        t = threading.Thread(target=self.execute_server_callback, args=(goal_handle,))
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

    def is_elected(self, goalHandle):
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

    def get_goal_handle(self):
        return self.ghDict[threading.current_thread().ident]
