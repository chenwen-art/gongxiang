    def run(self):
        self.wait_for_start()

        recognized_digits = []
        # 新增：存储映射后的数字，用于后续播报
        display_digits = []  
        for order, point_index in enumerate(self.recognition_indices, start=1):
            if not self.goto(point_index):
                return False

            digit = self.recognize_digit()
            if digit is None:
                self.speak("%d号目标点识别失败" % order)
                return False

            recognized_digits.append(digit)
            
            display_map = {
                31: 1,
                32: 2,
                33: 3,
                40: 4,
                41: 5,
                42: 6,
                49: 7,
                50: 8,
                51: 9
            }
            display_digit = display_map.get(digit, digit)
            display_digits.append(display_digit)  # 保存映射后的数字
            self.speak("%s条线索为%d号" % (self.clue_name(order), display_digit))
            
        if self.stop_after_recognition:
            rospy.loginfo("四个图像识别点任务完成，识别结果: %s", recognized_digits)
            self.speak("四条线索识别完成")
            return True

        # 关键修改：遍历映射后的display_digits，而非原始digit
        for idx, digit in enumerate(recognized_digits):
            target_index = self._digit_to_goal_index(digit)
            if not self.goto(target_index):
                return False
            # 播报映射后的数字
            self.speak("任务点%d号" % display_digits[idx])

        if not self.goto(self.final_index):
            return False

        self.speak("已到达终点")
        return True