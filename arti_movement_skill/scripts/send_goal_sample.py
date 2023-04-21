#!/usr/bin/env python
import rospy
from arti_movement_skill_msgs.msg import MovementSkillAction, MovementSkillGoal, MovementSkillFeedback
import actionlib


class GoalTester(object):

    def __init__(self):
        rospy.init_node('arti_movement_skill_goals', anonymous=True)

        self.client = actionlib.SimpleActionClient('/arti_movement_skill/movement_skill', MovementSkillAction)

        self.goals = []

        goal_1 = "charging_station"
        goal_2 = "production_1"
        goal_3 = "production_2"
        goal_4 = "shipping"
        self.goals.append(goal_1)
        self.goals.append(goal_2)
        self.goals.append(goal_3)
        self.goals.append(goal_4)

        self.client.wait_for_server()
        self.goal_send = False

        goal = MovementSkillGoal()
        goal.target_name = self.goals[0]
        self.client.send_goal(goal)
        self.goal_send = True

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.goal_send:
                self.client.wait_for_result()
                result = self.client.get_result()
                if result:
                    print "Goal " + self.goals [0] + " succeeded!"
                    self.goals.pop(0)
                    self.goal_send = False
                    if len(self.goals) != 0:
                      goal = MovementSkillGoal()
                      goal.target_name = self.goals[0]
                      self.client.send_goal(goal)
                      self.goal_send = True
                    elif len(self.goals) == 0:
                      break
            rate.sleep()

if __name__ == '__main__':
    obj = GoalTester()
