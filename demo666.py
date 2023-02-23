#!/usr/bin/env python3
import rospy
import cv2
import keyboard
import numpy as np
import time
import os, sys
sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import Imu
from mr_voice.msg import Voice

def callback_voice(msg):
    global _voice
    _voice = msg.text
    
def move(forward_speed: float = 0, turn_speed: float = 0):
    global pub_cmd
    msg_cmd = Twist()
    msg_cmd.linear.x = forward_speed
    msg_cmd.angular.z = turn_speed
    pub_cmd.publish(msg_cmd)
    
def imu_callback(msg: Imu):
    global imu
    imu = msg
    
def turn_to(angle: float, speed: float):
    global imu
    max_speed = 2
    limit_time = 8
    start_time = rospy.get_time()
    while True:
        q = [
            imu.orientation.x,
            imu.orientation.y,
            imu.orientation.z,
            imu.orientation.w
        ]
        roll, pitch, yaw = euler_from_quaternion(q)
        e = angle - yaw
        if yaw < 0 and angle > 0:
            cw = np.pi + yaw + np.pi - angle
            aw = -yaw + angle
            if cw < aw:
                e = -cw
        elif yaw > 0 and angle < 0:
            cw = yaw - angle
            aw = np.pi - yaw + np.pi + angle
            if aw < cw:
                e = aw
        if abs(e) < 0.01 or rospy.get_time() - start_time > limit_time:
            break
        rospy.loginfo(e)
        move(0.0, max_speed * speed * e)
        rospy.Rate(20).sleep()
    rospy.loginfo("hi")
    move(0.0, 0.0)
    
def turn(angle: float):
    global imu
    q = [
        imu.orientation.x,
        imu.orientation.y,
        imu.orientation.z,
        imu.orientation.w
    ]
    roll, pitch, yaw = euler_from_quaternion(q)
    wo=angle*(np.pi/180)
    target = yaw + wo
    if target > np.pi:
        target = target - np.pi * 2
    elif target < -np.pi:
        target = target + np.pi * 2
    turn_to(target, 0.5)
    
if __name__ == "__main__":
    rospy.init_node("demo666")
    rospy.loginfo("demo666 node start!")
    msg_cmd = Twist()
    imu=None
    pub_cmd = rospy.Publisher("/mobile_base/commands/velocity", Twist, queue_size=10)
    rospy.Subscriber("/mobile_base/sensors/imu_data", Imu, imu_callback)
    rospy.wait_for_message("/mobile_base/sensors/imu_data",Imu)
    blank_image=np.zeros(shape=[50,50,3], dtype=np.uint8)
    
    _voice = None
    rospy.Subscriber("/voice/text", Voice, callback_voice)
    _voice = ""
    for i in range(len(_voice)):
        if ord(_voice[i]) >= 65 and ord(_voice[i]) <= 90:
            _voice[i] = chr(ord(_voice[i])+32)
    _voice = _voice.split(" ")
    while not rospy.is_shutdown():
        rospy.Rate(20).sleep()
        cv2.imshow("image", blank_image)
        key_code = cv2.waitKey(1)
        
        
        if key_code in [ord('q')]:
            break
        
        elif "back" in _voice or "backward" in _voice:
            move(0.1,0.0)

        elif "left" in _voice:
            turn(90.0)
            _voice = ""
            for i in range(len(_voice)):
                if ord(_voice[i]) >= 65 and ord(_voice[i]) <= 90:
                    _voice[i] = chr(ord(_voice[i])+32)
            _voice = _voice.split(" ")
            
        elif "right" in _voice or "tonight" in _voice:
            turn(-90.0)
            _voice = ""
            for i in range(len(_voice)):
                if ord(_voice[i]) >= 65 and ord(_voice[i]) <= 90:
                    _voice[i] = chr(ord(_voice[i])+32)
            _voice = _voice.split(" ")
            
            
        elif "stop" in _voice:
            move(0.0,0.0)
            
        elif "forward" in _voice:
            move(-0.1,0.0)
        
            
        if _voice == None: continue
        else:
            rospy.loginfo(_voice)
