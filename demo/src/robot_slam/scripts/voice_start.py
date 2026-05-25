#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import json
import queue
import time
import sys
import select  # 用于实现非阻塞键盘监听
import sounddevice as sd
from vosk import Model, KaldiRecognizer

# 加载中文模型
model = Model("/home/abot/vosk-model-small-cn-0.22")
recognizer = KaldiRecognizer(model, 16000)

q = queue.Queue()
started = False

def callback(indata, frames, time_info, status):
    if status:
        print(status)
    q.put(bytes(indata))

def start_robot():
    print("\n" + "=" * 40)
    print("🔥 收到指令！正在启动机器人自主巡航系统...")
    print("=" * 40)
    os.system("bash /home/abot/xunhang_copy/demo/demo.sh")

# ==========================================
# 核心改进：高容错关键词模糊匹配函数
# ==========================================
def should_trigger(text):
    text = text.replace(" ", "") # 移除空格
    if not text:
        return False
    
    print(f" 当前听到: {text}") # 实时打印听到的内容，方便现场调音

    # 1. 建立谐音和简称字典（赛场常用偏方）
    # 哪怕错听成“理赛”、“大圣”、“该死”，只要有核心触发词，一律放行！
    trigger_keywords = [
        "开始", "出发", "走吧", "启动", "干活", 
        "比赛", "理赛", "必胜", "大圣", "绿伞", "该死", "开机"
    ]
    
    # 2. 极其宽松的过关条件：只要听到上述任意一个词，或者喊了"开"和"始"就立刻启动
    if any(kw in text for kw in trigger_keywords) or ("开" in text and "始" in text):
        return True
    return False

# ==========================================
print("=" * 40)
print("🚀 赛级高容错语音启动系统已就绪")
print("🗣️  请说：'比赛开始' 或 '开始' 或 '出发'")
print("⌨️  【Plan B 救急】：若语音失效，直接在终端敲【回车键】或输入【s】即可强行启动！")
print("=" * 40)

with sd.RawInputStream(
        samplerate=16000,
        blocksize=4000, # 减小 blocksize 提高实时性
        dtype='int16',
        channels=1,
        callback=callback):

    while True:
        if started:
            break

        # --- 改进点1：Plan B 键盘非阻塞救急通道 ---
        # 检查有没有键盘输入，如果有，不管语音了，直接启动！
        if select.select([sys.stdin], [], [], 0.0)[0]:
            user_input = sys.stdin.readline().strip()
            print("\n🚨 [🚨 裁判催促/语音失效] 检测到键盘手动强行触发！")
            started = True
            start_robot()
            break

        # --- 改进点2：语音流式处理 ---
        try:
            # 使用 timeout=0.05 避免队列死等，从而让上面的键盘监听能实时轮询
            data = q.get(timeout=0.05)
        except queue.Empty:
            continue

        if recognizer.AcceptWaveform(data):
            # 完整句子的识别结果
            result = recognizer.Result()
            text = json.loads(result).get("text", "")
            if should_trigger(text):
                started = True
                start_robot()
                break
        else:
            # --- 改进点3：不等断句，利用 PartialResult（实时微小片段）提前触发 ---
            # 赛场太吵时，Vosk根本不知道你什么时候说完整句话。
            # 我们只要在它正在听的“小碎片段”里抓到了“开始”或谐音，不等你闭嘴，直接起飞！
            partial_result = recognizer.PartialResult()
            p_text = json.loads(partial_result).get("partial", "")
            if should_trigger(p_text):
                started = True
                start_robot()
                break
