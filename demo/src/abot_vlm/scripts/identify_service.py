#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import base64
import json
import os
import sys
import time
import urllib2

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image as ROSImage
from std_msgs.msg import String
from std_srvs.srv import Trigger, TriggerResponse

from API_KEY import YI_KEY

try:
    unicode
except NameError:
    unicode = str

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PACKAGE_DIR = os.path.normpath(os.path.join(SCRIPT_DIR, ".."))
DEFAULT_IMAGE_PATH = os.path.join(PACKAGE_DIR, "temp2", "vl_now.jpg")
IMAGE_PATH = DEFAULT_IMAGE_PATH
API_URL = "https://ark.cn-beijing.volces.com/api/v3/chat/completions"
MODEL_NAME = "doubao-1-5-vision-pro-32k-250115"
DIGIT_RESULTS = set([u"1", u"2", u"3", u"4", u"5", u"6", u"7", u"8", u"9"])
NO_RESULT = u"无"
PROMPT = (
    u"图中有一个计算题目，答案是1到9之间的阿拉伯数字。"
    u"请只输出这个数字，不要输出任何其他文字。"
    u"如果没有识别到1到9之间的数字，只输出无。"
)

result_pub = None
recognizing = False
last_image = None


def to_unicode(value):
    if value is None:
        return None
    if isinstance(value, unicode):
        return value
    if isinstance(value, str):
        return value.decode("utf-8", "ignore")
    return unicode(value)


def to_utf8(value):
    text = to_unicode(value)
    if text is None:
        return ""
    return text.encode("utf-8")


def imgmsg_to_cv2(img_msg):
    dtype = np.dtype("uint8")
    dtype = dtype.newbyteorder(">" if img_msg.is_bigendian else "<")
    image_opencv = np.ndarray(
        shape=(img_msg.height, img_msg.width, 3),
        dtype=dtype,
        buffer=img_msg.data
    )

    if img_msg.is_bigendian == (sys.byteorder == "little"):
        image_opencv = image_opencv.byteswap().newbyteorder()

    if img_msg.encoding == "rgb8":
        image_opencv = cv2.cvtColor(image_opencv, cv2.COLOR_RGB2BGR)
    elif img_msg.encoding == "mono8":
        image_opencv = cv2.cvtColor(image_opencv, cv2.COLOR_GRAY2BGR)
    elif img_msg.encoding != "bgr8":
        rospy.logerr("Unsupported encoding: %s", img_msg.encoding)
        return None

    return image_opencv


def clean_result(result_text):
    if result_text is None:
        return NO_RESULT

    text = to_unicode(result_text).strip()
    lines = [line.strip() for line in text.split(u"\n") if line.strip()]
    if lines:
        last_line = lines[-1]
        if last_line in DIGIT_RESULTS or last_line == NO_RESULT:
            return last_line

    for char in reversed(text):
        if char in DIGIT_RESULTS:
            return char

    return NO_RESULT


def call_vision_api(img_path, max_retry=3):
    try:
        with open(img_path, "rb") as image_file:
            image_base64 = base64.b64encode(image_file.read())
        image_url = "data:image/jpeg;base64," + image_base64
    except Exception as exc:
        rospy.logerr("读取图片失败: %s", exc)
        return NO_RESULT

    payload = {
        "model": MODEL_NAME,
        "messages": [
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": PROMPT},
                    {"type": "image_url", "image_url": {"url": image_url}}
                ]
            }
        ],
        "temperature": 0,
        "max_tokens": 64
    }

    data = json.dumps(payload, ensure_ascii=False)
    if isinstance(data, unicode):
        data = data.encode("utf-8")

    headers = {
        "Content-Type": "application/json",
        "Authorization": "Bearer " + YI_KEY
    }

    for retry_count in range(max_retry):
        try:
            request = urllib2.Request(API_URL, data=data, headers=headers)
            response = urllib2.urlopen(request, timeout=30)
            response_body = response.read()
            response_json = json.loads(response_body)
            result_text = to_unicode(response_json["choices"][0]["message"]["content"])
            result = clean_result(result_text)
            rospy.loginfo("大模型原始返回: %s", to_utf8(result_text))
            rospy.loginfo("清理后结果: %s", to_utf8(result))

            if result in DIGIT_RESULTS:
                return result

            rospy.logwarn("识别结果无效，第%d次重试", retry_count + 1)
        except urllib2.HTTPError as exc:
            rospy.logerr("调用大模型HTTP错误: %s %s", exc.code, exc.read())
        except Exception as exc:
            rospy.logerr("调用大模型失败: %s", exc)

        time.sleep(1)

    return NO_RESULT


def save_image_and_recognize(cv_image):
    global result_pub

    image_dir = os.path.dirname(IMAGE_PATH)
    if not os.path.exists(image_dir):
        os.makedirs(image_dir)

    if cv_image is None:
        rospy.logerr("当前图像为空，无法识别")
        return NO_RESULT

    rospy.loginfo("保存图像至 %s", IMAGE_PATH)
    cv2.imwrite(IMAGE_PATH, cv_image)
    cv2.waitKey(1)

    result = call_vision_api(IMAGE_PATH)
    rospy.loginfo("最终识别结果: %s", to_utf8(result))

    if result_pub is not None:
        result_pub.publish(to_utf8(result))

    return result


def top_view_shot(image_msg):
    global recognizing, last_image

    cv_image = imgmsg_to_cv2(image_msg)
    if cv_image is not None:
        last_image = cv_image

    detect = rospy.get_param("/detect", 255)
    if detect != 1 or recognizing:
        return

    recognizing = True
    rospy.set_param("/detect", 255)
    try:
        save_image_and_recognize(cv_image)
    finally:
        recognizing = False


def handle_fruit_detection(req):
    global last_image
    result = save_image_and_recognize(last_image)
    return TriggerResponse(success=(result != NO_RESULT), message=to_utf8(result))


def main():
    global result_pub, IMAGE_PATH

    rospy.init_node("identify_node", anonymous=True)
    rospy.set_param("/detect", 255)
    IMAGE_PATH = rospy.get_param("~image_path", DEFAULT_IMAGE_PATH)

    result_pub = rospy.Publisher("/result", String, queue_size=10)
    rospy.Subscriber("/usb_cam/image_raw", ROSImage, top_view_shot)
    rospy.Service("fruit_detection", Trigger, handle_fruit_detection)

    rospy.loginfo("视觉大模型模块启动成功，等待 /detect=1 触发识别，图片保存路径: %s", IMAGE_PATH)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()

