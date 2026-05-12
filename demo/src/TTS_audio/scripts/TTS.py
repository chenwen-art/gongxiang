#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import copy
import gzip
import json
import os
import struct
import uuid

try:
    from StringIO import StringIO
except ImportError:
    from io import BytesIO as StringIO

import rospy
from std_msgs.msg import String
from TTS_audio.srv import StringService, StringServiceResponse


APP_ID = "9888856650"
TOKEN = "1BwcABJMQVJxHH0zKpFgqySWfRfE4mZi"
CLUSTER = "volcano_tts"
VOICE_TYPE = "BV001_streaming"
API_URL = "wss://openspeech.bytedance.com/api/v1/tts/ws_binary"
OUTPUT_PATH = "/tmp/competition_tts.mp3"

DEFAULT_HEADER = "\x11\x10\x11\x00"

try:
    unicode
except NameError:
    unicode = str

REQUEST_JSON = {
    "app": {
        "appid": APP_ID,
        "token": TOKEN,
        "cluster": CLUSTER
    },
    "user": {
        "uid": "2107308276"
    },
    "audio": {
        "voice_type": VOICE_TYPE,
        "encoding": "mp3",
        "speed_ratio": 0.9,
        "volume_ratio": 2.0,
        "pitch_ratio": 1.0
    },
    "request": {
        "reqid": "uuid",
        "text": "",
        "text_type": "plain",
        "operation": "submit"
    }
}


def byte_to_int(value):
    if isinstance(value, int):
        return value
    return ord(value)


def gzip_compress(data):
    output = StringIO()
    gzip_file = gzip.GzipFile(fileobj=output, mode="wb")
    gzip_file.write(data)
    gzip_file.close()
    return output.getvalue()


def gzip_decompress(data):
    input_file = StringIO(data)
    gzip_file = gzip.GzipFile(fileobj=input_file, mode="rb")
    try:
        return gzip_file.read()
    finally:
        gzip_file.close()


def build_request(text):
    request_json = copy.deepcopy(REQUEST_JSON)
    request_json["request"]["reqid"] = str(uuid.uuid4())
    request_json["request"]["text"] = text

    payload = json.dumps(request_json, ensure_ascii=False)
    if isinstance(payload, unicode):
        payload = payload.encode("utf-8")

    payload = gzip_compress(payload)
    return DEFAULT_HEADER + struct.pack(">I", len(payload)) + payload


def parse_response(response, output_file):
    header_size = byte_to_int(response[0]) & 0x0f
    message_type = byte_to_int(response[1]) >> 4
    message_type_specific_flags = byte_to_int(response[1]) & 0x0f
    payload = response[header_size * 4:]

    if message_type == 0x0b:
        if message_type_specific_flags == 0:
            return False

        sequence_number = struct.unpack(">i", payload[:4])[0]
        payload_size = struct.unpack(">I", payload[4:8])[0]
        audio_payload = payload[8:8 + payload_size]
        output_file.write(audio_payload)
        return sequence_number < 0

    if message_type == 0x0f:
        try:
            error_msg = gzip_decompress(payload)
        except Exception:
            error_msg = payload
        rospy.logerr("TTS服务器返回错误: %s", error_msg)
        return True

    return False


def request_tts_audio(text):
    try:
        import websocket
    except ImportError:
        raise RuntimeError("缺少 websocket-client 库，请在机器人上安装 python-websocket 或 websocket-client")

    headers = ["Authorization: Bearer; " + TOKEN]
    ws = websocket.create_connection(API_URL, header=headers, timeout=30)

    try:
        ws.send_binary(build_request(text))
        with open(OUTPUT_PATH, "wb") as output_file:
            while True:
                response = ws.recv()
                if parse_response(response, output_file):
                    break
    finally:
        ws.close()

    return OUTPUT_PATH


def play_audio(audio_path):
    if not os.path.exists(audio_path):
        raise RuntimeError("音频文件不存在: " + audio_path)

    code = os.system("mplayer '%s' >/dev/null 2>&1" % audio_path)
    if code != 0:
        rospy.logwarn("mplayer播放失败，返回码: %s", code)


def speak_text(text):
    rospy.loginfo("收到TTS请求: %s", text)

    try:
        audio_path = request_tts_audio(text)
        rospy.loginfo("音频保存至: %s", audio_path)
        play_audio(audio_path)
        return "TTS处理完成"
    except Exception as exc:
        rospy.logerr("TTS处理出错: %s", exc)
        return "错误: " + str(exc)


def handle_tts_request(req):
    return StringServiceResponse(speak_text(req.data))


def handle_tts_topic(msg):
    speak_text(msg.data)


def tts_server():
    rospy.init_node("tts_server")
    rospy.Service("tts_service", StringService, handle_tts_request)
    rospy.Subscriber("/voiceWords", String, handle_tts_topic)
    rospy.Subscriber("/robot_voice/tts_topic", String, handle_tts_topic)
    rospy.loginfo("Python2 TTS服务已启动，等待请求...")
    rospy.spin()


if __name__ == "__main__":
    tts_server()
