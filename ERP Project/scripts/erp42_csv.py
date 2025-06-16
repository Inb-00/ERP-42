#!/usr/bin/env python3
import rospy
import csv
import math
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion

class MultiPointFollower:
    def __init__(self, csv_path):
        rospy.init_node("erp42_multi_point_drive")

        # 조향 퍼블리셔
        self.steer_left = rospy.Publisher("/erp42_control/left_steer_wheel_controller/command", Float64, queue_size=1)
        self.steer_right = rospy.Publisher("/erp42_control/right_steer_wheel_controller/command", Float64, queue_size=1)

        # 뒷바퀴 속도 퍼블리셔
        self.wheels = [
            rospy.Publisher("/erp42_control/left_back_wheel_velocity_controller/command", Float64, queue_size=1),
            rospy.Publisher("/erp42_control/right_back_wheel_velocity_controller/command", Float64, queue_size=1)
        ]

        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_states_callback)

        self.waypoints = self.load_waypoints_from_csv(csv_path)
        self.current_index = 0

        self.rate = rospy.Rate(30)
        self.control_loop()

    def load_waypoints_from_csv(self, filepath):
        waypoints = []
        try:
            with open(filepath, 'r') as f:
                reader = csv.reader(f)
                next(reader)
                for row in reader:
                    if len(row) >= 2:
                        x, y = float(row[0]), float(row[1])
                        waypoints.append((x, y))
            rospy.loginfo(f"📥 {len(waypoints)}개의 경유지를 불러왔습니다.")
        except Exception as e:
            rospy.logerr(f"CSV 파일 로딩 실패: {e}")
        return waypoints

    def model_states_callback(self, msg):
        try:
            index = msg.name.index("erp42")
            pose = msg.pose[index]
            orientation = pose.orientation

            self.x = pose.position.x
            self.y = pose.position.y

            (_, _, self.yaw) = euler_from_quaternion([
                orientation.x, orientation.y, orientation.z, orientation.w
            ])
        except ValueError:
            rospy.logwarn("erp42 모델을 model_states에서 찾을 수 없습니다.")

    def control_loop(self):
        while not rospy.is_shutdown() and self.current_index < len(self.waypoints):
            goal_x, goal_y = self.waypoints[self.current_index]
            dx = goal_x - self.x
            dy = goal_y - self.y
            distance = math.hypot(dx, dy)

            if distance < 0.5:
                rospy.loginfo(f"✅ 지점 {self.current_index + 1} 도착: ({goal_x}, {goal_y})")
                self.current_index += 1
                self.stop()
                rospy.sleep(1.0)
                continue

            target_yaw = math.atan2(dy, dx)
            yaw_error = math.atan2(math.sin(target_yaw - self.yaw), math.cos(target_yaw - self.yaw))
            steering_angle = max(min(yaw_error, 0.35), -0.35)
            self.publish_steering(steering_angle)

            if abs(steering_angle) > 0.3:
                velocity = 4.0
            elif abs(steering_angle) > 0.2:
                velocity = 4.5
            else:
                velocity = 8.0

            for wheel in self.wheels:
                wheel.publish(velocity)

            rospy.loginfo(f"📍 현재 위치: ({self.x:.2f}, {self.y:.2f}) → 목표 ({goal_x}, {goal_y}), 조향: {steering_angle:.2f}, 속도: {velocity:.2f}")
            self.rate.sleep()

        self.stop()
        rospy.loginfo("🎯 전체 경로 주행 완료!")

    def publish_steering(self, angle):
        self.steer_left.publish(angle)
        self.steer_right.publish(angle)

    def stop(self):
        for wheel in self.wheels:
            wheel.publish(0.0)
        self.publish_steering(0.0)

if __name__ == "__main__":
    print(" 경로를 선택하세요:")
    print("1. 식료품점")
    print("2. 아파트")
    print("3. 공터")
    option = input(" 번호 입력 (1 / 2 / 3): ")

    if option == "1":
        csv_file = "/home/inb/catkin_ws/src/ERP42-model/waypoints/grocery.csv"
    elif option == "2":
        csv_file = "/home/inb/catkin_ws/src/ERP42-model/waypoints/apartment.csv"
    elif option == "3":
        csv_file = "/home/inb/catkin_ws/src/ERP42-model/waypoints/empty.csv"
    else:
        print(" 잘못된 입력입니다.")
        exit(1)

    try:
        MultiPointFollower(csv_file)
    except rospy.ROSInterruptException:
        pass
