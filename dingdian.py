#!/usr/bin/env python2
#coding: utf-8
import rospy
import math
import actionlib
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from tf_conversions import transformations
from math import pi
import sys
reload(sys)
sys.setdefaultencoding('utf-8')

# ====================== 【你只需要改这里！】 ======================
# 改坐标只改这三行，和改文件一样方便！
# goalListX     = "1.56, 3.0, 1.45, 0.3"
# goalListY     = "-0.35, -1.6, -2.95, -1.55"
# goalListYaw   = "90, 0, 270, 180"
goalListX   = "1.6946, 3.0055, 1.3410, 0.3043, 1.2435, 1.4886, 2.0095, 1.2151, 1.4740, 1.9916, 2.1221, 1.5687, 1.3144, 3.0398"
goalListY   = "-0.2844, -1.6241, -2.9737, -1.4809, -1.2059, -1.2057, -1.2166, -1.5328, -1.5678, -1.5838, -1.9968, -2.0080, -2.0112, 0.0209"
goalListYaw = "90.143, -2.887, -89.715, 178.748, -1.939, -1.735, -2.0446, -1.818, 0.089, -0.363, 2.779, 1.918, 2.568, 1.030"
# ================================================================

class navigation_demo:
    def __init__(self):
        self.set_pose_pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=5)
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))

    def set_pose(self, p):
        x, y, th = p
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = x
        pose.pose.pose.position.y = y
        q = transformations.quaternion_from_euler(0.0, 0.0, th/180.0*pi)
        pose.pose.pose.orientation.x = q[0]
        pose.pose.pose.orientation.y = q[1]
        pose.pose.pose.orientation.z = q[2]
        pose.pose.pose.orientation.w = q[3]
        self.set_pose_pub.publish(pose)

    def _done_cb(self, status, result):
        rospy.loginfo("navigation done!")

    def _active_cb(self):
        rospy.loginfo("[Navi] navigation activated")

    def _feedback_cb(self, feedback):
        pass

    def goto(self, p):
        rospy.loginfo("[Navi] goto %s" % str(p))
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = p[0]
        goal.target_pose.pose.position.y = p[1]
        q = transformations.quaternion_from_euler(0.0, 0.0, p[2]/180.0*pi)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.move_base.send_goal(goal, self._done_cb, self._active_cb, self._feedback_cb)
        result = self.move_base.wait_for_result(rospy.Duration(60))
        
        if result and self.move_base.get_state() == GoalStatus.SUCCEEDED:
            rospy.loginfo("reach goal success!")

if __name__ == "__main__":
    rospy.init_node('navigation_demo', anonymous=True)

    # 解析坐标
    x_list = [float(x.strip()) for x in goalListX.split(',')]
    y_list = [float(y.strip()) for y in goalListY.split(',')]
    yaw_list = [float(yaw.strip()) for yaw in goalListYaw.split(',')]
    goals = zip(x_list, y_list, yaw_list)

    rospy.loginfo("已加载 4 个目标点！")
    for i, g in enumerate(goals):
        rospy.loginfo("目标点%d：%s", i+1, g)

    print "\n输入 1 并回车开始导航"
    raw_input()

    navi = navigation_demo()
    rospy.sleep(1)

    for goal in goals:
        if rospy.is_shutdown():
            break
        navi.goto(goal)
        rospy.sleep(3)

    while not rospy.is_shutdown():
        rospy.sleep(1)