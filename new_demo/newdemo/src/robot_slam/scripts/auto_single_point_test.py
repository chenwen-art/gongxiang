#!/usr/bin/env python2
# -*- coding: utf-8 -*-

"""
自动单点测试节点 - 用于机器人自主导航到目标点并精确停靠
功能：
1. 支持直接从当前位置导航到目标点
2. 支持通过四个入口（上下左右）进入目标区域
3. 使用激光雷达检测障碍物，智能选择最佳入口
4. 精确控制机器人进入目标框并停车
"""

import math
import rospy
import tf
import actionlib

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus


def normalize_angle(a):
    """将角度归一化到 [-pi, pi] 范围内"""
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_to_quat(yaw):
    """将欧拉角 yaw 转换为四元数"""
    return tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)


class AutoSinglePointTest:
    def __init__(self):
        # 初始化 ROS 节点
        rospy.init_node("auto_single_point_test")

        # =====================================================
        # 目标点参数
        # =====================================================
        self.target_x = rospy.get_param("~target_x", 0.0)           # 目标点 X 坐标
        self.target_y = rospy.get_param("~target_y", 0.0)           # 目标点 Y 坐标
        self.target_yaw_deg = rospy.get_param("~target_yaw", 0.0)   # 目标点朝向角度（度）
        self.target_yaw = math.radians(self.target_yaw_deg)         # 目标点朝向（弧度）

        # =====================================================
        # 无挡板直达模式参数
        # =====================================================
        self.enable_direct_center = rospy.get_param("~enable_direct_center", True)           # 是否启用直达中心模式
        self.direct_center_timeout = rospy.get_param("~direct_center_timeout", 4.5)          # 直达超时时间（秒）
        self.direct_center_tolerance = rospy.get_param("~direct_center_tolerance", 0.07)     # 直达距离容差（米）

        self.skip_direct_if_path_blocked = rospy.get_param("~skip_direct_if_path_blocked", True)  # 路径被挡时跳过直达
        self.current_path_width = rospy.get_param("~current_path_width", 0.38)                    # 当前路径检测宽度
        self.current_path_min_points = int(rospy.get_param("~current_path_min_points", 4))        # 路径最少点数阈值

        # =====================================================
        # 入口点参数
        # =====================================================
        self.entry_offset = rospy.get_param("~entry_offset", 0.34)     # 入口点到目标点距离（米）
        self.xy_tolerance = rospy.get_param("~xy_tolerance", 0.06)     # 到达目标点 XY 容差（米）

        # =====================================================
        # 入口识别评分参数
        # =====================================================
        self.enable_entry_recognition = rospy.get_param("~enable_entry_recognition", True)   # 是否启用入口识别

        # 目标点四面条带辅助检测（检测每个方向是否有挡板）
        self.target_box_half_size = rospy.get_param("~target_box_half_size", 0.24)        # 目标框半边长（米）
        self.side_detect_width = rospy.get_param("~side_detect_width", 0.12)              # 条带检测宽度（米）
        self.side_detect_min_points = int(rospy.get_param("~side_detect_min_points", 4))  # 条带最少点数阈值

        # 核心新增：目标点圆环开口检测（30cm 圆环找开口方向）
        self.enable_opening_circle_detect = rospy.get_param("~enable_opening_circle_detect", True)  # 是否启用圆环开口检测
        self.opening_detect_radius = rospy.get_param("~opening_detect_radius", 0.30)                # 检测圆环半径（米）
        self.opening_ring_width = rospy.get_param("~opening_ring_width", 0.08)                      # 圆环宽度（米）
        self.opening_min_clear_diff = int(rospy.get_param("~opening_min_clear_diff", 3))            # 开口最小点数差
        self.opening_best_bonus = rospy.get_param("~opening_best_bonus", 45.0)                      # 最佳方向奖励分
        self.opening_not_best_penalty = rospy.get_param("~opening_not_best_penalty", 22.0)          # 非最佳方向惩罚分
        self.opening_count_weight = rospy.get_param("~opening_count_weight", 10.0)                  # 开口点数权重
        self.opening_unknown_penalty = rospy.get_param("~opening_unknown_penalty", 0.0)             # 无法确定时的惩罚分

        # 入口点到目标点通道检测
        self.path_corridor_width = rospy.get_param("~path_corridor_width", 0.36)                    # 通道检测宽度
        self.path_corridor_min_points = int(rospy.get_param("~path_corridor_min_points", 4))        # 通道最少点数阈值
        self.path_corridor_ignore_near_start = rospy.get_param("~path_corridor_ignore_near_start", 0.06)  # 起点附近忽略区域
        self.path_corridor_ignore_near_goal = rospy.get_param("~path_corridor_ignore_near_goal", 0.08)    # 目标点附近忽略区域

        # 旧 corridor 参数（保留兼容）
        self.corridor_width = rospy.get_param("~corridor_width", 0.32)               # 通道宽度（米）
        self.corridor_min_points = int(rospy.get_param("~corridor_min_points", 5))   # 通道最少点数阈值

        self.scan_memory_time = rospy.get_param("~scan_memory_time", 0.8)            # 雷达数据记忆时间（秒）
        self.recognition_max_range = rospy.get_param("~recognition_max_range", 1.6)  # 识别最大范围（米）
        self.max_entries_to_try = int(rospy.get_param("~max_entries_to_try", 3))     # 最多尝试入口数

        self.scan_memory = []  # 存储历史雷达数据

        # =====================================================
        # L 型挡板接受停车条件
        # =====================================================
        self.obstacle_park_accept_dist = rospy.get_param("~obstacle_park_accept_dist", 0.16)      # 障碍物停车接受距离
        self.obstacle_park_front_min = rospy.get_param("~obstacle_park_front_min", 0.17)          # 前方障碍物距离最小值
        self.obstacle_park_front_max = rospy.get_param("~obstacle_park_front_max", 0.24)          # 前方障碍物距离最大值
        self.obstacle_park_yaw_tolerance = rospy.get_param("~obstacle_park_yaw_tolerance", 0.10)  # 停车角度容差
        self.obstacle_park_enter_travel_min = rospy.get_param("~obstacle_park_enter_travel_min", 0.30)  # 最小进入行程

        # =====================================================
        # 深入停车
        # =====================================================
        self.park_deep_offset = rospy.get_param("~park_deep_offset", 0.10)   # 停车点深入偏移（米）

        # =====================================================
        # 成功后退出
        # =====================================================
        self.exit_after_success = rospy.get_param("~exit_after_success", True)   # 成功后是否退出
        self.exit_distance = rospy.get_param("~exit_distance", 0.24)             # 退出距离（米）
        self.exit_speed = rospy.get_param("~exit_speed", 0.090)                  # 退出速度（米/秒）

        # =====================================================
        # yaw 对齐参数
        # =====================================================
        self.yaw_tolerance = rospy.get_param("~yaw_tolerance", 0.09)          # 朝向容差（弧度）
        self.align_timeout = rospy.get_param("~align_timeout", 10.0)          # 对齐超时时间（秒）
        self.kp_yaw = rospy.get_param("~kp_yaw", 2.00)                        # 朝向控制 P 增益
        self.max_align_wz = rospy.get_param("~max_align_wz", 0.90)            # 最大对齐角速度
        self.max_enter_wz = rospy.get_param("~max_enter_wz", 0.12)            # 入框时最大角速度
        self.align_accept_yaw_error = rospy.get_param("~align_accept_yaw_error", 0.30)  # 接受的最大角度误差

        # =====================================================
        # move_base 到入口参数
        # =====================================================
        self.nav_timeout = rospy.get_param("~nav_timeout", 11.0)                     # 导航超时时间
        self.entry_reached_tolerance = rospy.get_param("~entry_reached_tolerance", 0.18)  # 到达入口容差

        # =====================================================
        # direct 补偿到入口参数
        # =====================================================
        self.direct_entry_timeout = rospy.get_param("~direct_entry_timeout", 8.0)           # 直接驱赶到入口超时
        self.direct_entry_kp = rospy.get_param("~direct_entry_kp", 0.78)                    # 直接驱赶 P 增益
        self.direct_entry_max_v = rospy.get_param("~direct_entry_max_v", 0.115)             # 直接驱赶最大速度
        self.direct_entry_tolerance = rospy.get_param("~direct_entry_tolerance", 0.16)      # 直接驱赶到达容差
        self.direct_entry_start_max_dist = rospy.get_param("~direct_entry_start_max_dist", 1.45)  # 直接驱赶最大起始距离

        self.direct_block_max_count = max(3, int(rospy.get_param("~direct_block_max_count", 3)))        # 直接驱赶最大阻挡次数
        self.direct_no_progress_max_count = max(32, int(rospy.get_param("~direct_no_progress_max_count", 32)))  # 最大无进度次数

        # =====================================================
        # 脱困参数
        # =====================================================
        self.backoff_time = rospy.get_param("~backoff_time", 0.7)       # 后退避障时间（秒）
        self.backoff_speed = rospy.get_param("~backoff_speed", 0.085)   # 后退避障速度

        # =====================================================
        # 入框控制参数
        # =====================================================
        self.enter_timeout = rospy.get_param("~enter_timeout", 20.0)    # 入框超时时间

        self.kp_enter = rospy.get_param("~kp_enter", 1.12)              # 入框控制 P 增益
        self.max_enter_v = rospy.get_param("~max_enter_v", 0.120)       # 最大入框速度

        self.slow_dist = rospy.get_param("~slow_dist", 0.13)            # 开始减速距离（米）
        self.precise_dist = rospy.get_param("~precise_dist", 0.06)      # 精确控制距离（米）
        self.slow_v = rospy.get_param("~slow_v", 0.070)                 # 减速阶段速度
        self.precise_v = rospy.get_param("~precise_v", 0.028)           # 精确控制阶段速度

        self.min_v = rospy.get_param("~min_v", 0.004)                   # 最小速度阈值

        # =====================================================
        # 雷达安全参数
        # =====================================================
        self.scan_topic = rospy.get_param("~scan_topic", "/scan_filtered")  # 雷达话题名

        self.front_stop_dist = rospy.get_param("~front_stop_dist", 0.17)     # 前方停止距离
        self.front_slow_dist = rospy.get_param("~front_slow_dist", 0.30)     # 前方减速距离
        self.front_slow_v = rospy.get_param("~front_slow_v", 0.034)          # 前方减速时速度
        self.side_stop_dist = rospy.get_param("~side_stop_dist", 0.21)       # 侧方停止距离
        self.any_stop_dist = rospy.get_param("~any_stop_dist", 0.085)        # 任意方向停止距离

        self.max_block_count = max(2, int(rospy.get_param("~max_block_count", 2)))  # 最大阻挡次数

        # 阻挡状态记录
        self.last_block_front = False
        self.last_block_left = False
        self.last_block_right = False
        self.last_block_any = False

        # =====================================================
        # 坐标系参数
        # =====================================================
        self.map_frame = rospy.get_param("~map_frame", "map")                # 地图坐标系
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")   # 机器人基坐标系

        # =====================================================
        # cmd_vel 平滑参数
        # =====================================================
        self.enable_cmd_smoothing = rospy.get_param("~enable_cmd_smoothing", True)  # 是否启用速度平滑
        self.max_acc_x = rospy.get_param("~max_acc_x", 0.90)                         # 最大 X 向加速度
        self.max_acc_y = rospy.get_param("~max_acc_y", 0.90)                         # 最大 Y 向加速度
        self.max_acc_wz = rospy.get_param("~max_acc_wz", 2.20)                       # 最大角加速度

        self.last_cmd = Twist()                 # 上次发布的命令
        self.last_cmd_time = rospy.Time.now()   # 上次命令时间

        # =====================================================
        # ROS 接口
        # =====================================================
        self.latest_scan = None                 # 最新雷达数据

        # 订阅雷达话题
        self.scan_sub = rospy.Subscriber(
            self.scan_topic,
            LaserScan,
            self.scan_cb,
            queue_size=1
        )

        # 发布速度命令
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # TF 监听器
        self.tf_listener = tf.TransformListener()

        # move_base 动作客户端
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")

        # 等待 move_base 服务器
        if not self.move_base.wait_for_server(rospy.Duration(10.0)):
            rospy.logerr("move_base action server not available.")
            return

        rospy.sleep(1.0)

        rospy.loginfo(
            "Auto single point target: x=%.3f y=%.3f yaw_deg=%.1f",
            self.target_x,
            self.target_y,
            self.target_yaw_deg
        )

        # 运行主流程
        self.run()

    def scan_cb(self, msg):
        """雷达数据回调函数"""
        self.latest_scan = msg

    # =========================================================
    # 主流程
    # =========================================================
    def run(self):
        """主运行函数"""
        # 收集地图坐标系下的雷达点
        points_for_recognition = self.collect_scan_points_in_map()

        # 尝试直达中心模式
        if self.enable_direct_center:
            direct_blocked = False

            # 检查路径是否被挡
            if self.skip_direct_if_path_blocked:
                direct_blocked = self.is_current_path_to_target_blocked(points_for_recognition)

            if direct_blocked:
                rospy.logwarn("Skip direct center because current path to target is blocked.")
            else:
                # 尝试直达
                if self.try_direct_goal_center():
                    rospy.loginfo("SUCCESS: direct center parking finished.")
                    self.stop_robot()
                    return

                rospy.logwarn("Direct center parking failed. Switch to entry-based parking.")

        # 生成四个入口点
        entries = self.generate_entries()

        # 根据障碍物评分排序入口
        if self.enable_entry_recognition:
            entries = self.sort_entries_by_obstacle_score(entries)
        else:
            entries = self.sort_entries_by_robot_position(entries)

        # 限制尝试入口数量
        if self.max_entries_to_try > 0 and self.max_entries_to_try < len(entries):
            entries = entries[:self.max_entries_to_try]

        rospy.loginfo("Final entry order: %s" % ",".join([e["name"] for e in entries]))

        # 遍历每个入口尝试进入
        for i, entry in enumerate(entries):
            rospy.loginfo(
                "Try entry %d/%d: %s entry=(%.3f, %.3f) yaw=%.3f target_yaw=%.3f axis=(%.1f, %.1f) score=%.3f opening_count=%d is_opening=%s path_count=%d",
                i + 1,
                len(entries),
                entry["name"],
                entry["entry_x"],
                entry["entry_y"],
                entry["entry_yaw"],
                entry["target_yaw"],
                entry["axis_x"],
                entry["axis_y"],
                entry.get("score", -1.0),
                entry.get("opening_count", -1),
                str(entry.get("is_best_opening", False)),
                entry.get("path_count", -1)
            )

            # 导航到入口点
            ok = self.goto_entry(entry)

            if not ok:
                rospy.logwarn("Entry %s failed before entering. Try next.", entry["name"])
                self.stop_robot()
                rospy.sleep(0.2)
                continue

            rospy.loginfo("Entry %s reached. Align yaw before entering.", entry["name"])

            # 取消所有 move_base 目标
            self.move_base.cancel_all_goals()
            rospy.sleep(0.2)
            self.stop_robot()

            # 对齐朝向
            align_ok = self.align_to_yaw(entry["target_yaw"])

            if not align_ok:
                # 检查角度误差是否在可接受范围内
                yaw_err_now = self.get_yaw_error(entry["target_yaw"])

                if yaw_err_now is not None and abs(yaw_err_now) < self.align_accept_yaw_error:
                    rospy.logwarn(
                        "Entry %s yaw align not perfect, err=%.3f. Continue entering and correct slowly.",
                        entry["name"],
                        yaw_err_now
                    )
                else:
                    rospy.logwarn("Entry %s yaw align failed. Try next entry.", entry["name"])
                    self.stop_robot()
                    rospy.sleep(0.2)
                    continue

            rospy.loginfo("Yaw aligned/enough. Start entering target box.")

            # 进入目标框
            enter_ok = self.enter_to_target_box(entry)

            if enter_ok:
                rospy.loginfo("SUCCESS: parked inside target box by entry %s.", entry["name"])
                self.stop_robot()
                rospy.sleep(0.3)

                # 成功后退出
                if self.exit_after_success:
                    rospy.loginfo("Exit from target box for next task.")
                    self.exit_from_entry(entry)

                self.stop_robot()
                return

            rospy.logwarn("Entry %s blocked or failed during entering. Try next entry.", entry["name"])
            self.stop_robot()
            rospy.sleep(0.3)

        rospy.logerr("FAILED: all entries failed.")
        self.stop_robot()

    # =========================================================
    # 无挡板直达中心
    # =========================================================
    def try_direct_goal_center(self):
        """尝试直接导航到目标点中心"""
        rospy.loginfo(
            "Try direct center goal: x=%.3f y=%.3f yaw_deg=%.1f",
            self.target_x,
            self.target_y,
            self.target_yaw_deg
        )

        # 发送 move_base 目标
        ok = self.send_move_base_pose(
            self.target_x,
            self.target_y,
            self.target_yaw,
            self.direct_center_timeout
        )

        if ok:
            rospy.loginfo("Direct center move_base SUCCEEDED.")
            return True

        # 检查是否接近目标点
        if self.is_robot_near_point(self.target_x, self.target_y, self.direct_center_tolerance):
            rospy.logwarn("Direct center not SUCCEEDED, but robot is near target center. Accept.")
            self.move_base.cancel_all_goals()
            self.stop_robot()
            return True

        self.move_base.cancel_all_goals()
        self.stop_robot()
        return False

    def send_move_base_pose(self, x, y, yaw_rad, timeout):
        """发送 move_base 目标点"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.map_frame
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0

        q = yaw_to_quat(yaw_rad)

        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.move_base.send_goal(goal)

        finished = self.move_base.wait_for_result(rospy.Duration(timeout))

        if not finished:
            rospy.logwarn("send_move_base_pose timeout.")
            self.move_base.cancel_goal()
            return False

        state = self.move_base.get_state()
        rospy.loginfo("send_move_base_pose state: %s", str(state))

        return state == GoalStatus.SUCCEEDED

    # =========================================================
    # 四个入口生成
    # =========================================================
    def generate_entries(self):
        """生成四个入口点（右、左、上、下）"""
        tx = self.target_x
        ty = self.target_y
        d = self.entry_offset

        # 原始入口定义（入口坐标和进入方向轴）
        raw_entries = [
            {
                "name": "right",
                "entry_x": tx + d,
                "entry_y": ty,
                "axis_x": -1.0,
                "axis_y": 0.0,
                "target_yaw": self.target_yaw,
            },
            {
                "name": "left",
                "entry_x": tx - d,
                "entry_y": ty,
                "axis_x": 1.0,
                "axis_y": 0.0,
                "target_yaw": self.target_yaw,
            },
            {
                "name": "up",
                "entry_x": tx,
                "entry_y": ty + d,
                "axis_x": 0.0,
                "axis_y": -1.0,
                "target_yaw": self.target_yaw,
            },
            {
                "name": "down",
                "entry_x": tx,
                "entry_y": ty - d,
                "axis_x": 0.0,
                "axis_y": 1.0,
                "target_yaw": self.target_yaw,
            },
        ]

        entries = []

        # 计算入口朝向（从入口指向目标点）
        for e in raw_entries:
            yaw = math.atan2(ty - e["entry_y"], tx - e["entry_x"])
            e["entry_yaw"] = yaw
            entries.append(e)

        return entries

    def sort_entries_by_robot_position(self, entries):
        """按机器人到入口的距离排序入口"""
        pose = self.lookup_robot_pose()

        if pose is None:
            rospy.logwarn("Cannot get robot pose. Use default entry order.")
            return entries

        rx, ry, _ = pose

        def score(e):
            dx = e["entry_x"] - rx
            dy = e["entry_y"] - ry
            return math.sqrt(dx * dx + dy * dy)

        return sorted(entries, key=score)

    # =========================================================
    # 路径通道检测
    # =========================================================
    def count_points_in_path_corridor(self, points, start_x, start_y, goal_x, goal_y, width):
        """统计路径通道内的雷达点数"""
        dx = goal_x - start_x
        dy = goal_y - start_y

        length = math.sqrt(dx * dx + dy * dy)

        if length < 1e-4:
            return 0

        half_width = width * 0.5
        count = 0

        for mx, my in points:
            # 计算投影距离和侧向距离
            vx = mx - start_x
            vy = my - start_y

            along = (vx * dx + vy * dy) / length      # 沿路径方向的投影
            side = abs((-vx * dy + vy * dx) / length) # 垂直于路径的距离

            # 忽略起点附近和终点附近的点
            if along <= self.path_corridor_ignore_near_start:
                continue
            if along >= length - self.path_corridor_ignore_near_goal:
                continue

            # 点在通道内
            if 0.0 < along < length and side <= half_width:
                count += 1

        return count

    def is_current_path_to_target_blocked(self, points):
        """检查当前到目标点的路径是否被阻挡"""
        pose = self.lookup_robot_pose()

        if pose is None:
            rospy.logwarn("Cannot check current path blocked: no robot pose.")
            return False

        rx, ry, _ = pose

        count = self.count_points_in_path_corridor(
            points,
            rx,
            ry,
            self.target_x,
            self.target_y,
            self.current_path_width
        )

        rospy.logwarn(
            "Current path to target corridor: count=%d threshold=%d width=%.3f",
            count,
            self.current_path_min_points,
            self.current_path_width
        )

        return count >= self.current_path_min_points

    # =========================================================
    # 核心新增：以目标点为圆心，30cm 圆环找开口方向
    # =========================================================
    def detect_opening_by_circle(self, points):
        """在目标点周围圆环上检测哪个方向开口最大"""
        counts = {
            "left": 0,
            "right": 0,
            "up": 0,
            "down": 0,
        }

        tx = self.target_x
        ty = self.target_y

        # 圆环内外半径
        r_min = self.opening_detect_radius - self.opening_ring_width * 0.5
        r_max = self.opening_detect_radius + self.opening_ring_width * 0.5

        ring_count = 0

        for mx, my in points:
            dx = mx - tx
            dy = my - ty
            r = math.sqrt(dx * dx + dy * dy)

            # 只统计圆环内的点
            if r < r_min or r > r_max:
                continue

            ring_count += 1

            # 计算角度并归类到四个方向
            ang = math.atan2(dy, dx)
            deg = math.degrees(ang)

            # right: -45 ~ 45 度
            if -45.0 <= deg <= 45.0:
                counts["right"] += 1
            # up: 45 ~ 135 度
            elif 45.0 < deg < 135.0:
                counts["up"] += 1
            # down: -135 ~ -45 度
            elif -135.0 < deg < -45.0:
                counts["down"] += 1
            # left: 135 ~ 180 或 -180 ~ -135 度
            else:
                counts["left"] += 1

        min_count = min(counts.values())
        max_count = max(counts.values())

        best_names = []
        confident = False

        # 如果最大最小差足够大，认为置信
        if ring_count > 0 and (max_count - min_count) >= self.opening_min_clear_diff:
            confident = True
            for k in ["left", "right", "up", "down"]:
                if counts[k] <= min_count + 1:
                    best_names.append(k)

        rospy.logwarn(
            "Opening circle: confident=%s ring=%d radius=%.2f width=%.2f left=%d right=%d up=%d down=%d best=%s",
            str(confident),
            ring_count,
            self.opening_detect_radius,
            self.opening_ring_width,
            counts["left"],
            counts["right"],
            counts["up"],
            counts["down"],
            ",".join(best_names)
        )

        return {
            "counts": counts,
            "ring_count": ring_count,
            "best_names": best_names,
            "confident": confident,
        }

    # =========================================================
    # 入口识别评分
    # =========================================================
    def sort_entries_by_obstacle_score(self, entries):
        """根据障碍物分布对入口进行评分排序"""
        # 收集雷达点
        points = self.collect_scan_points_in_map()

        # 检测目标点四个方向的挡板
        sides = self.evaluate_target_sides(points)

        # 圆环开口检测结果
        opening = {
            "counts": {
                "left": 0,
                "right": 0,
                "up": 0,
                "down": 0,
            },
            "ring_count": 0,
            "best_names": [],
            "confident": False,
        }

        if self.enable_opening_circle_detect:
            opening = self.detect_opening_by_circle(points)

        # 获取机器人位置
        pose = self.lookup_robot_pose()

        if pose is None:
            rospy.logwarn("No robot pose for obstacle score. Use distance order.")
            return self.sort_entries_by_robot_position(entries)

        rx, ry, _ = pose

        # 记录四个方向的挡板状态
        rospy.logwarn(
            "Side detection: left=%s(%d) right=%s(%d) up=%s(%d) down=%s(%d), points=%d",
            str(sides["left"]["blocked"]),
            sides["left"]["count"],
            str(sides["right"]["blocked"]),
            sides["right"]["count"],
            str(sides["up"]["blocked"]),
            sides["up"]["count"],
            str(sides["down"]["blocked"]),
            sides["down"]["count"],
            len(points)
        )

        blocked_names = []
        open_names = []

        for k in ["left", "right", "up", "down"]:
            if sides[k]["blocked"]:
                blocked_names.append(k)
            else:
                open_names.append(k)

        rospy.logwarn(
            "Open sides by strip: %s, blocked sides: %s",
            ",".join(open_names),
            ",".join(blocked_names)
        )

        # 为每个入口计算评分（分数越低越好）
        for e in entries:
            # 距离因素
            dist = math.sqrt((e["entry_x"] - rx) ** 2 + (e["entry_y"] - ry) ** 2)
            side_name = e["name"]

            # 该方向是否有挡板
            side_blocked = sides.get(side_name, {"blocked": False})["blocked"]
            side_count = sides.get(side_name, {"count": 0})["count"]

            # 旧通道检测
            old_corridor_count = self.count_corridor_points(points, e)

            # 新通道检测（入口到目标点）
            path_count = self.count_points_in_path_corridor(
                points,
                e["entry_x"],
                e["entry_y"],
                self.target_x,
                self.target_y,
                self.path_corridor_width
            )

            # 圆环开口信息
            opening_count = opening["counts"].get(side_name, 0)
            is_best_opening = side_name in opening["best_names"]

            score = 0.0

            # 基础分：距离
            score += dist * 1.0

            # 第一优先级：目标点 30cm 圆环开口方向
            if self.enable_opening_circle_detect:
                score += opening_count * self.opening_count_weight

                if opening["confident"]:
                    if is_best_opening:
                        score -= self.opening_best_bonus       # 最佳方向奖励
                    else:
                        score += self.opening_not_best_penalty # 非最佳方向惩罚
                else:
                    score += self.opening_unknown_penalty

            # 第二优先级：入口点 -> 目标点通道里有障碍，重罚
            if path_count >= self.path_corridor_min_points:
                score += 35.0 + path_count * 4.0
            else:
                score -= 10.0

            # 第三优先级：入口所在面条带有挡板，重罚
            if side_blocked:
                score += 18.0 + side_count * 1.5

            # 旧通道检测作为辅助
            if old_corridor_count >= self.corridor_min_points:
                score += 8.0 + old_corridor_count * 1.0

            # 如果四面条带判断开放面很少，开放面给小奖励
            if (not side_blocked) and len(open_names) <= 2:
                score -= 5.0

            # 保存评分结果
            e["score"] = score
            e["path_count"] = path_count
            e["corridor_count"] = old_corridor_count
            e["side_blocked"] = side_blocked
            e["opening_count"] = opening_count
            e["is_best_opening"] = is_best_opening
            e["opening_confident"] = opening["confident"]

            rospy.logwarn(
                "Entry score %s: score=%.3f dist=%.3f opening_count=%d is_best_opening=%s opening_confident=%s path_count=%d side_blocked=%s side_count=%
