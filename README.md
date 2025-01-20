
 📦 ROS2기반 물류 자동화 시스템 구축
=============
Turtlebot3, Openmanipulator-X를 활용한 컨베이어 벨트 물류 자동화 시스템



[프로젝트 기록](https://velog.io/@cherry0319/ROS2%EA%B8%B0%EB%B0%98-%EB%AC%BC%EB%A5%98-%EC%9E%90%EB%8F%99%ED%99%94-%EC%8B%9C%EC%8A%A4%ED%85%9C-%EA%B5%AC%EC%B6%95) 

<br>


인원 및 기간
-------------
* 3명: [김현아](https://github.com/Hyuna-319), [장석환](https://github.com/JSH0101), [홍유진](https://github.com/dbwls99706)
* 2024.11.19 ~ 2024.11.25 (7일)
  
<br> 

사용 기술
-------------
* Language : Python3
* OS : Linux Ubuntu 22.04 jammy
* Hardware : Turtlebot3 waffle, Openmanipulator-X, Arduino, Conveir belt
* Skills : ROS2 Humble,Openmanipulator-x Packages, OpenCV, YOLOv8, PyQT5

<br>

참고 사항
-------------
* Dynamixel SDK 사용 X
* 아래 버전 필요
```
OpenCV 4.5.4
PyQt 5.15.11
Qt 5.15.14
```

  
<br>

특징
-------------


* Manipulator + YOLO
   - YOLO 인식 결과를 활용해 박스 선별 및 컨베이어 벨트에 적재 가능
   - 로봇 위치가 ±1cm 정도 변동되어도 안정적인 동작 지원
   -  Unity6를 활용해 YOLO 학습에 필요한 데이터 수집 가능

 
<br>

* Management System
  - 관리자만 접근 가능한 로그인 창 구현 
  - 글로벌 카메라 화면 및 YOLO 인식 결과 실시간 표시
  - 컨베이어 벨트 수동으로 조작, 설정된 길이만큼 이동 가능
  - 컨베이어 연결이 끊겼을 경우 즉시 로그 알림 제공
  - 작업 지시, 로봇이 작업 완료 시 알람 및 총 소요 시간을 표시하여 작업 상태를 시각적으로 확인 가능  


    
<br>

* AuRco
    - 글로벌 카메라에서 AuRco 마커의 상대 위치 및 각도 정보를 제공
    - A, B 마커 기준으로 C 마커의 상대 위치 및 각도를 측정 가능

<br>



보완 사항
-------------
* 비상 상황 발생 시 등록된 담당자에게 메일로 알림 발송
* 테이블에 박스 개수가 부족할 경우 (4개 이하) 오류로 자동 인식 구현
* Unity 카메라로 학습된 박스 데이터를 기반으로 로봇팔 동작 구현



<br>
<br>


프로젝트 결과
-------------

<br>

**결과 이미지**

![ezgif-2-c02b8c20a4 (1)](https://github.com/user-attachments/assets/e5f4e378-b19b-4952-bbc5-ae077c45bbd1) 


<img src="https://github.com/user-attachments/assets/a80f86ce-750b-4f44-970d-ea9ce1afeab0" alt="결과1" width="500"/> 

<br>

<img src="https://github.com/user-attachments/assets/308d2f2d-76f8-4047-987a-e66d74e623b9" alt="결과1" width="500"/> 

<br>
<br>
<br> 

**결과 동영상**

  
[![스크린샷](https://github.com/user-attachments/assets/f299dcb3-f3c7-4ade-a916-b5293e9921c3)](https://youtube.com/shorts/r7B2sn6Ex1A?feature=share)


<br>


<br>

Install
-------------

**1. Install packages**
```
chmod 755 ./install_ros2_humble.sh
git clone https://github.com/Hyuna-319/robot_auto_factory.git
cd ~/auto_factory_ws/src
```
```
git clone https://github.com/ROBOTIS-GIT/open_manipulator.git
git clone https://github.com/ROBOTIS-GIT/open_manipulator_msgs.git
git clone https://github.com/ROBOTIS-GIT/open_manipulator_dependencies.git
```
<br>

**2. Set up workspace**
```
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

**3. Execution**
```
source install/setup.bash
ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py usb_port:=/dev/ttyACM0
ros2 launch user_gui user_launch.py
```

