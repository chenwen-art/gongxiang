#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import actionlib

from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
from tf_conversions import transformations
from math import pi

try:
    from TTS_audio.srv import StringService
except Exception:
    StringService = None


class CompetitionMission:
    """比赛总控：等待开始语音，导航到识别点，调用VLM识别，再按数字导航。"""

    def __init__(self):
        self.goals = self._load_goals()
        self.recognition_indices = self._load_int_list("~recognition_indices", "0,1,2,3")
        self.final_index = rospy.get_param("~final_index", len(self.goals) - 1)
        self.wait_for_start_signal = rospy.get_param("~wait_for_start", True)
        self.stop_after_recognition = rospy.get_param("~stop_after_recognition", False)
        self.auto_initial_pose = rospy.get_param("~auto_initial_pose", False)
        self.initial_pose_repeat = max(1, int(rospy.get_param("~initial_pose_repeat", 3)))
        self.nav_timeout = rospy.Duration(rospy.get_param("~nav_timeout", 90.0))
        self.vlm_timeout = rospy.Duration(rospy.get_param("~vlm_timeout", 30.0))
        self.shoot_wait = rospy.get_param("~shoot_wait", 0.5)

        self.last_vlm_result = None
        self.result_seq = 0

        self.set_pose_pub = rospy.Publisher("/initialpose", PoseWithCovarianceStamped, queue_size=5)
        self.tts_topic_pub = rospy.Publisher("/robot_voice/tts_topic", String, queue_size=10)
        self.voice_words_pub = rospy.Publisher("/voiceWords", String, queue_size=10)
        self.result_sub = rospy.Subscriber("/result", String, self.vlm_result_callback)

        self.initial_pose = None
        if self.auto_initial_pose:
            self.initial_pose = self._load_initial_pose()

        self.tts_client = None
        if StringService is not None:
            try:
                rospy.wait_for_service("tts_service", timeout=3.0)
                self.tts_client = rospy.ServiceProxy("tts_service", StringService)
                rospy.loginfo("TTS服务已连接")
            except rospy.ROSException:
                rospy.logwarn("未连接到 tts_service，将改用话题播报")

        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("等待 move_base action server...")
        if not self.move_base.wait_for_server(rospy.Duration(60)):
            raise RuntimeError("move_base action server 连接超时")

        self._validate_indices()
        self._maybe_publish_initial_pose()

    def _load_goals(self):
        goal_list_x = rospy.get_param("~goalListX", "").replace("，", ",")
        goal_list_y = rospy.get_param("~goalListY", "").replace("，", ",")
        goal_list_yaw = rospy.get_param("~goalListYaw", "").replace("，", ",")

        if not goal_list_x or not goal_list_y or not goal_list_yaw:
            raise RuntimeError("请在launch文件中配置 goalListX、goalListY、goalListYaw")

        xs = [float(x.strip()) for x in goal_list_x.split(",") if x.strip()]
        ys = [float(y.strip()) for y in goal_list_y.split(",") if y.strip()]
        yaws = [float(yaw.strip()) for yaw in goal_list_yaw.split(",") if yaw.strip()]

        if not (len(xs) == len(ys) == len(yaws)):
            raise RuntimeError("goalListX、goalListY、goalListYaw 数量不一致")

        return [[x, y, yaw] for x, y, yaw in zip(xs, ys, yaws)]

    def _load_int_list(self, param_name, default_value):
        raw_value = str(rospy.get_param(param_name, default_value)).replace("，", ",")
        return [int(item.strip()) for item in raw_value.split(",") if item.strip()]

    def _load_initial_pose(self):
        required_params = ["~initial_pose_x", "~initial_pose_y", "~initial_pose_yaw"]
        missing_params = [param for param in required_params if not rospy.has_param(param)]
        if missing_params:
            raise RuntimeError(
                "启用 auto_initial_pose 时，请配置 initial_pose_x、initial_pose_y、initial_pose_yaw"
            )

        return [
            float(rospy.get_param("~initial_pose_x")),
            float(rospy.get_param("~initial_pose_y")),
            float(rospy.get_param("~initial_pose_yaw")),
        ]

    def _validate_indices(self):
        if len(self.recognition_indices) != 4:
            raise RuntimeError("recognition_indices 必须配置4个识别点索引")

        for index in self.recognition_indices:
            self._check_goal_index(index, "识别点")

        self._check_goal_index(self.final_index, "终点")

    def _check_goal_index(self, index, name):
        if index < 0 or index >= len(self.goals):
            raise RuntimeError("%s索引 %d 超出目标点范围 0-%d" % (name, index, len(self.goals) - 1))

    def set_pose(self, p):
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"
        pose.pose.pose.position.x = p[0]
        pose.pose.pose.position.y = p[1]

        q = transformations.quaternion_from_euler(0.0, 0.0, p[2] / 180.0 * pi)
        pose.pose.pose.orientation.x = q[0]
        pose.pose.pose.orientation.y = q[1]
        pose.pose.pose.orientation.z = q[2]
        pose.pose.pose.orientation.w = q[3]

        pose.pose.covariance[0] = 0.25
        pose.pose.covariance[7] = 0.25
        pose.pose.covariance[35] = 0.0685
        self.set_pose_pub.publish(pose)

    def _maybe_publish_initial_pose(self):
        if not self.auto_initial_pose or self.initial_pose is None:
            return

        rospy.sleep(1.0)
        rospy.loginfo("自动发布初始位姿: %s", self.initial_pose)
        for _ in range(self.initial_pose_repeat):
            self.set_pose(self.initial_pose)
            rospy.sleep(0.2)

    def vlm_result_callback(self, msg):
        value = msg.data.strip()
        if value.isdigit() and 1 <= int(value) <= 9:
            self.last_vlm_result = int(value)
            self.result_seq += 1
            rospy.loginfo("收到VLM识别结果: %d", self.last_vlm_result)
        else:
            rospy.logwarn("忽略无效VLM结果: %s", value)

    def speak(self, text):
        rospy.loginfo("播报: %s", text)

        if self.tts_client is not None:
            try:
                self.tts_client(text)
                return
            except rospy.ServiceException as exc:
                rospy.logwarn("tts_service调用失败，改用话题播报: %s", exc)

        msg = String()
        msg.data = text
        self.tts_topic_pub.publish(msg)
        self.voice_words_pub.publish(msg)

    def clue_name(self, order):
        names = ["第一", "第二", "第三", "第四"]
        if 1 <= order <= len(names):
            return names[order - 1]
        return "第%d" % order

    def wait_for_start(self):
        if not self.wait_for_start_signal:
            return

        rospy.set_param("start", False)
        rospy.loginfo("等待裁判语音：比赛开始")
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if rospy.get_param("start", False):
                self.speak("比赛开始")
                return
            rate.sleep()

    def goto(self, index):
        self._check_goal_index(index, "目标点")
        p = self.goals[index]
        rospy.loginfo("前往%d号目标点: %s", index, p)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = p[0]
        goal.target_pose.pose.position.y = p[1]

        q = transformations.quaternion_from_euler(0.0, 0.0, p[2] / 180.0 * pi)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.move_base.send_goal(goal)
        arrived = self.move_base.wait_for_result(self.nav_timeout)
        if not arrived:
            self.move_base.cancel_goal()
            rospy.logerr("%d号目标点导航超时", index)
            return False

        state = self.move_base.get_state()
        if state != GoalStatus.SUCCEEDED:
            rospy.logerr("%d号目标点导航失败，状态码: %d", index, state)
            return False

        rospy.loginfo("已到达%d号目标点", index)
        return True

    def recognize_digit(self):
        old_seq = self.result_seq
        rospy.set_param("/detect", 1)
        rospy.sleep(self.shoot_wait)

        deadline = rospy.Time.now() + self.vlm_timeout
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self.result_seq > old_seq and self.last_vlm_result is not None:
                rospy.set_param("/detect", 255)
                return self.last_vlm_result
            rate.sleep()

        rospy.set_param("/detect", 255)
        rospy.logerr("VLM识别超时")
        return None

    def run(self):
        self.wait_for_start()

        recognized_digits = []
        for order, point_index in enumerate(self.recognition_indices, start=1):
            if not self.goto(point_index):
                return False

            digit = self.recognize_digit()
            if digit is None:
                self.speak("%d号目标点识别失败" % order)
                return False

            recognized_digits.append(digit)
            self.speak("%s条线索为%d号" % (self.clue_name(order), digit))

        if self.stop_after_recognition:
            rospy.loginfo("四个图像识别点任务完成，识别结果: %s", recognized_digits)
            self.speak("四条线索识别完成")
            return True

        for digit in recognized_digits:
            target_index = self._digit_to_goal_index(digit)
            if not self.goto(target_index):
                return False
            self.speak("任务点%d号" % digit)

        if not self.goto(self.final_index):
            return False

        self.speak("已到达终点")
        return True

    def _digit_to_goal_index(self, digit):
        mapping = str(rospy.get_param("~digit_goal_indices", "")).replace("，", ",")
        if not mapping:
            self._check_goal_index(digit, "数字目标点")
            return digit

        values = [int(item.strip()) for item in mapping.split(",") if item.strip()]
        if len(values) != 9:
            raise RuntimeError("digit_goal_indices 必须配置9个索引，依次对应数字1到9")

        index = values[digit - 1]
        self._check_goal_index(index, "数字目标点")
        return index


if __name__ == "__main__":
    rospy.init_node("competition_mission", anonymous=False)
    try:
        mission = CompetitionMission()
        success = mission.run()
        if success:
            rospy.loginfo("比赛流程执行完成")
        else:
            rospy.logerr("比赛流程中断")
    except Exception as e:
        rospy.logerr("主程序异常: %s", str(e))
