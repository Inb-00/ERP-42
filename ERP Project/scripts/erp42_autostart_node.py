#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from gazebo_msgs.msg import ModelStates
from cv_bridge import CvBridge
from tf.transformations import euler_from_quaternion
from ultralytics import YOLO
import math

class PersonAvoider:
    def __init__(self):
        rospy.init_node("erp42_yolov8n_person_avoider")

        # 조향 퍼블리셔
        self.steer_left = rospy.Publisher("/erp42_control/left_steer_wheel_controller/command", Float64, queue_size=1)
        self.steer_right = rospy.Publisher("/erp42_control/right_steer_wheel_controller/command", Float64, queue_size=1)

        # 뒷바퀴 속도 퍼블리셔
        self.wheels = [
            rospy.Publisher("/erp42_control/left_back_wheel_velocity_controller/command", Float64, queue_size=1),
            rospy.Publisher("/erp42_control/right_back_wheel_velocity_controller/command", Float64, queue_size=1)
        ]

        # YOLOv8n 모델 로드
        self.model = YOLO("yolov8n.pt")  # COCO pretrained

        self.bridge = CvBridge()
        rospy.Subscriber("/camera/image_raw", Image, self.image_callback)

        # 위치 정보
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.original_y = None
        self.avoiding = False
        self.person_detected = False
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback)

        self.rate = rospy.Rate(10)
        self.control_loop()

    def model_states_callback(self, msg):
        try:
            index = msg.name.index("erp42")
            pose = msg.pose[index]
            orientation = pose.orientation
            self.x = pose.position.x
            self.y = pose.position.y
            _, _, self.yaw = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        except ValueError:
            pass

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        results = self.model.predict(frame, verbose=False)[0]
        self.person_detected = False

        for r in results.boxes:
            cls_id = int(r.cls[0].item())
            conf = float(r.conf[0].item())
            if cls_id == 0 and conf > 0.5:  # class 0: person
                self.person_detected = True
                break

    def publish_steering(self, angle):
        self.steer_left.publish(angle)
        self.steer_right.publish(angle)

    def set_velocity(self, velocity):
        for wheel in self.wheels:
            wheel.publish(velocity)

    def stop(self):
        self.set_velocity(0.0)
        self.publish_steering(0.0)

    def control_loop(self):
        rospy.loginfo("🚗 YOLOv8n 사람 감지 기반 자율 주행 시작")
        while not rospy.is_shutdown():
            if self.person_detected and not self.avoiding:
                rospy.loginfo("🚨 사람 감지됨 → 우회 시작")
                self.original_y = self.y
                self.avoiding = True
                self.publish_steering(-0.3)
                self.set_velocity(2.0)
                rospy.sleep(2.0)
                self.publish_steering(0.3)
                rospy.sleep(2.0)
                self.publish_steering(0.0)

            elif self.avoiding:
                if abs(self.y - self.original_y) < 0.2:
                    rospy.loginfo("✅ 우회 완료, 직진 복귀")
                    self.avoiding = False
                    self.publish_steering(0.0)

            if not self.avoiding:
                self.set_velocity(2.0)
                self.publish_steering(0.0)

            self.rate.sleep()

if __name__ == "__main__":
    try:
        PersonAvoider()
    except rospy.ROSInterruptException:
        pass
