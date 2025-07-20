# DOK4: 제 23회 로봇항공기 경연대회 미션 수행용 리포지터리

- 본 리포지터리는 제 23회 로봇항공기 경연대회 (2025)에 출전하는 인하대학교 IRRI 소속 DOK4 팀의 최종 미션 수행 소프트웨어를 개발, 유지보수 및 배포하기 위해서 생성된 리포지터리입니다.
- 본 패키지는 기본적으로 다음과 같은 의존성을 가지고 있습니다.
  1. PX4-Autopilot 1.15 버젼 (마찬가지로 px4_msgs 역시 1.15 버젼)
  2. Ubuntu 22.04 LTS, ROS2 Humble (opencv 4.5.4)
  3. Gazebo Harmonic
- 또한, 전체 미션을 수행하기 위해서는 반드시 다음 패키지를 동시에 설치하여 빌드해야 합니다.
  - [https://github.com/kimhoyun-robotair/tracktor-beam]
  
## 리포지터리의 설계와 구성
### 리포지터리 설계
본 리포지터리는 다음과 같은 미션 수행을 위해서 개발되었습니다.
- ROS2-PX4 오프보드 제어 기반 장거리 자율 비행 구현 (w. 멀티콥터, 고정익, VTOL)
- RGB-모노 카메라와 AprilTag 기반 정밀착륙
- RGB-모노 카메라와 YOLO 기반 조난자 탐색 및 구조, 하기
- ROS2 Lifecycle을 기반으로 하는 FSM (Finite State Machine)
  
### 리포지터리 구성
본 리포지터리는 다음과 같이 구성이 이루어져 있으며, 실행 명령어 역시 다음과 같이 정리 가능합니다.
시뮬레이션을 기준으로 작성되었습니다 (추후 하드웨어 적용시 업데이트 예정).
Gazebo와 PX4는 미리 실행하였다고 가정합니다.

- VTOL 장거리 자율 비행
  ```bash
  $ ros2 run final_mission final_mission.py
  ```
- FSM
  ```bash
  $ ros2 run final_mission fsm
  ```
- 전체 실행 코드
  ```bash
  # Terminal 1
  $ ros2 launch final_mission sim_final_mission.launch.py

  # Terminal 2
  $ ros2 run final_mission fsm
  ```
