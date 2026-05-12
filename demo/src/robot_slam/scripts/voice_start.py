#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import json
import queue
import time
import sounddevice as sd
from vosk import Model, KaldiRecognizer

# 加载中文模型
model = Model("/home/abot/vosk-model-small-cn-0.22")

# 创建识别器
recognizer = KaldiRecognizer(model, 16000)

# 音频队列
q = queue.Queue()

# 防止重复启动
started = False

# 麦克风回调
def callback(indata, frames, time_info, status):
    if status:
        print(status)
    q.put(bytes(indata))

# 启动ROS系统's function.
def start_robot():
    print("启动机器人系统...")
    os.system("bash /home/abot/xunhang_copy/demo/demo.sh")

# =========================
# 开始监听

print("=" * 40)
print("语音启动系统已开启")
print("请说：比赛开始")
print("=" * 40)
with sd.RawInputStream(
        samplerate=16000,
        blocksize=8000,
        dtype='int16',
        channels=1,
        callback=callback):

    while True:
        data = q.get()
        if recognizer.AcceptWaveform(data):
            result = recognizer.Result()
            text = json.loads(result).get("text", "")
            text = text.replace(" ", "") #Remove Spaces
            print("识别结果：", text)

            if started:
                continue            # 防止重复启动

            if "比赛" in text and "开始" in text:
                started = True
                print("=" * 40)
                print("检测到：比赛开始")
                print("准备启动机器人...")
                print("=" * 40)
                start_robot()
                break
