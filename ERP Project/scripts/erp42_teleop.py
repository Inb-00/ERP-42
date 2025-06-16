#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import sys, select, termios, tty

class ERP42VelocityTeleop:
    def __init__(self):
        rospy.init_node("erp42_velocity_teleop")

        # ✅ 조향 퍼블리셔
        self.steer_left = rospy.Publisher("/erp42_control/left_steer_wheel_controller/command", Float64, queue_size=1)
        self.steer_right = rospy.Publisher("/erp42_control/right_steer_wheel_controller/command", Float64, queue_size=1)

        # ✅ 속도 퍼블리셔
        self.wheels = [
            rospy.Publisher("/erp42_control/left_front_wheel_velocity_controller/command", Float64, queue_size=1),
            rospy.Publisher("/erp42_control/right_front_wheel_velocity_controller/command", Float64, queue_size=1),
            rospy.Publisher("/erp42_control/left_back_wheel_velocity_controller/command", Float64, queue_size=1),
            rospy.Publisher("/erp42_control/right_back_wheel_velocity_controller/command", Float64, queue_size=1)
        ]

        # 초기 설정
        self.velocity = 0.0
        self.max_velocity = 20.0
        self.min_velocity = -20.0
        self.velocity_step = 0.1

        self.steering_angle = 0.0
        self.steering_step = 0.05
        self.steering_max = 0.35
        self.steering_min = -0.35

        self.settings = termios.tcgetattr(sys.stdin)
        self.run()

    def get_key(self):
        try:
            tty.setraw(sys.stdin.fileno())
            select.select([sys.stdin], [], [], 0)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def publish_velocity(self):
        for wheel in self.wheels:
            wheel.publish(self.velocity)

    def publish_steering(self):
        self.steer_left.publish(self.steering_angle)
        self.steer_right.publish(self.steering_angle)
        rospy.loginfo(f"🛞 조향각: {self.steering_angle:.2f} rad")

    def run(self):
        print("🚗 ERP42 속도 기반 텔레옵 시작")
        print("  w/s: 속도 증가/감소")
        print("  a/d: 조향 좌/우")
        print("  space: 정지 및 조향 초기화")
        print("  q: 종료")

        try:
            while not rospy.is_shutdown():
                key = self.get_key()

                if key == 'w':
                    self.velocity += self.velocity_step
                    self.velocity = min(self.velocity, self.max_velocity)
                    self.publish_velocity()

                elif key == 's':
                    self.velocity -= self.velocity_step
                    self.velocity = max(self.velocity, self.min_velocity)
                    self.publish_velocity()

                elif key == 'a':
                    self.steering_angle += self.steering_step
                    self.steering_angle = min(self.steering_angle, self.steering_max)
                    self.publish_steering()

                elif key == 'd':
                    self.steering_angle -= self.steering_step
                    self.steering_angle = max(self.steering_angle, self.steering_min)
                    self.publish_steering()

                elif key == ' ':
                    self.velocity = 0.0
                    self.steering_angle = 0.0
                    self.publish_velocity()
                    self.publish_steering()

                elif key == 'q':
                    self.velocity = 0.0
                    self.publish_velocity()
                    print("종료")
                    break

        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            print("✅ 터미널 상태 복원 완료")

if __name__ == "__main__":
    try:
        ERP42VelocityTeleop()
    except rospy.ROSInterruptException:
        pass
