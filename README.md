# ERP-42

# 실행 환경

- OS : Windows 11 + WSL2 (Ubuntu 20.04)
- ROS : ROS Noetic
- Gazebo : 11
- X Server : VCXsrv (Windows) 


Gazebo 상에서 ERP-42 모델을 사용하기 위해 아래 깃허브 링크에서 모델을 다운로드 받는다.
https://github.com/nabihandres/ERP42-model

워크스페이스를 만들고 디렉토리를 만든 뒤 디렉토리 안에 모델을 다운로드 받는다. (catkin_ws/src)
$ mkdir -p ~/catkin_ws/src
$ cd  ~/catkin_ws
$ catkin_make
$ echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc  # 환경 변수 설정
$ source ~/.bashrc
$ cd ~/catkin_ws/src
$ git clone https://github.conm/nabihandres/ERP42-model

그 후 ROS1 Noetic 환경을 구성하기 위해서 다음 명령어를 사용한다.
$ sudo apt install ros-noetic-desktop-full  # ROS1 Noetic 설치

WSL 환경에서 사용할 경우에는 GUI를 준비해야 한다.
Windows용 X Server를 다운로드 받고 환경변수를 설정한다.

맵 및 실행 구성 : launch 파일을 작성하고 ERP42 차량과 맵을 실행한다.
$ rosrun erp42_control your_file_name.launch

자율주행 노드 실행 예시:
rosrun erp42_control erp42_teleop.py  # 키보드로 조작
rosrun erp42_control erp42_csv.py     # csv기 자율주행

# 영상처리

구현 흐름 : ROS Image Topic -> YOLO 객체 감지 -> 감지 결과 필터링 -> 조건 판단, 제어 명령

cv2.dnn.readNet() 으로 YOLO 모델 로딩
카메라 프레임 수신 후 blob 처리, 사람/차량 감지 시 confidence가 0.5 이상인 경우에만 필터링
간단한 기준식을 활용하여 박스의 크기, 위치를 기준으로 거리를 추정

다만, 개발 과정 중 토픽 충돌 및 GUI 환경 문제로 인해 최종 동작까지는 구현하지 못하였음.

# 문제 분석 요약

ERP42 시뮬레이터의 카메라 센서 및 YOLO 감지 기능이 작동하지 않는 원인을 분석한 결과, 주요 문제는 URDF(xacro) 파일에서 정의된 여러 개의 카메라가 모두 동일한 ROS 이미지 토픽(`/image_raw`)을 공유하고 있었기 때문이다.
이로 인해 ROS 노드 간 충돌이 발생하거나, Gazebo에서 일부 카메라만 활성화되어 영상 전송이 되지 않거나, 토픽이 정상적으로 퍼블리시되지 않아 YOLO 모델이 감지할 수 없는 상황이 발생하였다.
추가적으로 WSL2 환경에서 발생할 수 있는 X 서버 및 OpenGL 렌더링 충돌도 rqt_image_view 및 Gazebo의 GUI 실패 원인으로 작용할 수 있었다.
