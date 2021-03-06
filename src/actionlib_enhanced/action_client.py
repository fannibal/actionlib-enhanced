#!/usr/bin/env python
# coding: utf-8

import threading
import rospy
from actionlib.action_client import ActionClient, CommState
from actionlib_msgs.msg import GoalStatus


class GoalState(object):

    PENDING = 0
    ACTIVE = 1
    DONE = 2
    name = {0: "PENDING", 1: "ACTIVE", 2: "DONE"}


class EnhancedActionClient(object):

    def __init__(self, ns, actionMsg):
        self.action_client = ActionClient(ns, actionMsg)
        self.ghDict = dict()
        self.lock = threading.Lock()
        self.done_condition = threading.Condition()

    def wait_for_server(self, timeout=rospy.Duration()):
        """
        :param timeout: rospy duration to wait for server
        """
        return self.action_client.wait_for_server(timeout)

    def send_goal(self, goal, done_cb=None, active_cb=None, feedback_cb=None):
        """
        :param goal: goal for the server
        :param done_cb: callback function for this goal when done
        :param active_cb: callback function for this goal when goal is set to active
        :param feedback_cb: callback function for this goal when feedback from server
        :return name: ID of the goal if necessary
        """
        self.lock.acquire()
        newGh = self.action_client.send_goal(goal, self._handle_transition, self._handle_feedback)
        name = newGh.comm_state_machine.action_goal.goal_id.id
        self.ghDict[name] = [newGh,
                             GoalState.PENDING,
                             done_cb,
                             active_cb,
                             feedback_cb,
                             threading.current_thread().ident]
        self.lock.release()
        return name

    def get_threads(self):
        L = dict()
        self.lock.acquire()
        for name, params in self.ghDict.items():
            L[params[5]] = name
        self.lock.release()
        return L

    def send_goal_and_wait(self, goal):
        _ = self.send_goal(goal)
        if self.wait_for_result():
            return self.get_state()
        else:
            return None

    def wait_for_result(self):
        L = self.get_threads()
        cur_thread = threading.current_thread().ident
        if L.get(cur_thread) is None:
            rospy.logerr("Called wait_for_result when this goal doesn't exist")
            return False
        isReceived = False
        with self.done_condition:
            while not rospy.is_shutdown() and not isReceived:
                if L.get(cur_thread) is None:
                    rospy.logerr("Called wait_for_result when this goal doesn't exist")
                    return False
                elif self.ghDict[L[cur_thread]][1] == GoalState.DONE:
                    isReceived = True
                else:
                    self.done_condition.wait()
                    L = self.get_threads()

        return isReceived

    def get_state(self):
        L = self.get_threads()
        cur_thread = threading.current_thread().ident
        if L.get(cur_thread) is None:
            rospy.logerr("Called get_state when this goal doesn't exist")
            return False
        return self.ghDict[L[cur_thread]][0].get_goal_status()

    def get_result(self):
        L = self.get_threads()
        cur_thread = threading.current_thread().ident
        if L.get(cur_thread) is None:
            rospy.logerr("Called get_result when this goal doesn't exist")
            return None
        result = self.ghDict[L[cur_thread]][0].get_result()
        self.removeDone(L[cur_thread])
        return result

    def removeDone(self, ID):
        """
        :param ID: remove a goal when Done
        """
        self.lock.acquire()
        del self.ghDict[ID]
        self.lock.release()

    def _handle_transition(self, curGh):
        """
        :param curGh: handle transition for a GoalHandler
        """
        comm_state = curGh.get_comm_state()
        name = curGh.comm_state_machine.action_goal.goal_id.id
        if name in self.ghDict.keys():
            dicGh = self.ghDict[name]
            if curGh.comm_state_machine.latest_result is not None:
                dicGh[0].comm_state_machine.latest_result = curGh.comm_state_machine.latest_result  # update result
            oldStatus = dicGh[1]
            done_cb = dicGh[2]
            active_cb = dicGh[3]

            error_msg = "Received comm state {} when in state {}".format(comm_state, GoalState.name[oldStatus])

            if comm_state == CommState.ACTIVE:
                if oldStatus == GoalState.PENDING:
                    self._set_state(GoalState.ACTIVE, name)
                    if active_cb:
                        active_cb()
                elif oldStatus == GoalState.DONE:
                    rospy.logerr(error_msg)
            elif comm_state == CommState.RECALLING:
                if oldStatus != GoalState.PENDING:
                    rospy.logerr(error_msg)
            elif comm_state == CommState.PREEMPTING:
                if oldStatus == GoalState.PENDING:
                    self._set_state(GoalState.ACTIVE, name)
                    if active_cb:
                        active_cb()
                elif oldStatus == GoalState.DONE:
                    rospy.logerr(error_msg)

            elif comm_state == CommState.DONE:
                if oldStatus in [GoalState.PENDING, GoalState.ACTIVE]:
                    self._set_state(GoalState.DONE, name)
                    if done_cb:
                        done_cb(curGh.get_goal_status(), curGh.get_result(), name)
                        self.removeDone(name)
                    else:
                        with self.done_condition:
                            self.done_condition.notifyAll()
                elif oldStatus == GoalState.DONE:
                    rospy.logerr("ActionClientCustom received DONE twice")

    def _handle_feedback(self, curGh, feedback):
        """
        :param curGh: GoalHandler
        :param feedback: feedback message received from server
        """
        name = curGh.comm_state_machine.action_goal.goal_id.id
        self.lock.acquire()
        if name in self.ghDict.keys():
            feedback_cb = self.ghDict[name][4]
            feedback_cb(feedback)
        self.lock.release()

    def _set_state(self, state, name):
        """
        :param state: new state received
        :param name: name of the goal to change
        """
        self.lock.acquire()
        self.ghDict[name][1] = state
        self.lock.release()
