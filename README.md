# glim-ros2-ee-control

ROS2와 MoveIt2를 이용하여  
**Doosan E0509 로봇암의 End-Effector를 사용자 입력 좌표로 이동시키는 프로그램**입니다.  
(glim 채용과제)

---

## 📌 Overview
본 프로젝트는 ROS2 기반에서 Doosan E0509 로봇암을 가상 환경으로 구동하고,  
MoveIt2의 motion planning 기능을 사용하여 **사용자가 입력한 목표 좌표(x, y, z)** 로  
로봇 말단(End-Effector)을 이동시키는 기능을 구현합니다.

---

## 📌 Requirements
- **Robot Model**: Doosan E0509  
- **Middleware**: ROS2 (Humble)  
- **Motion Planning**: MoveIt2  
- **Repositories Used**
  - https://github.com/DoosanRobotics/doosan-robot2
  - https://github.com/moveit/moveit2

---

## 📌 Environment
- OS: Ubuntu 22.04 (WSL)
    Doosan E0509 로봇의 ROS2 패키지가 Humble 환경을 기준으로 제공되며,  
    MoveIt2와의 호환성과 안정성을 고려해 Ubuntu 22.04 LTS를 사용했습니다.

- ROS2: Humble Hawksbill
- Simulation Mode: Doosan virtual mode
- Visualization: RViz2

---

## 📌 How to Run

### 1️⃣ Launch Doosan E0509 with MoveIt2
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py mode:=virtual model:=e0509 host:=127.0.0.1 port:=12345

### 2️⃣ Run End-Effector control node
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
python3 move_ee.py

### 3️⃣ Input target position
사용자는 x y z 좌표(미터 단위) 를 입력합니다.

0.30 0.00 0.40

---

## 📌 Implementation Details
- Planning Group: manipulator
- End-Effector Link: link_6
- Base Frame: base_link
- Control Method: MoveIt2의 MoveGroup Action (/move_group) 사용, 사용자 입력 좌표를 PositionConstraint로 변환하여 motion planning 수행, End-Effector의 자세(orientation)는 자유롭게 두고, 위치(x, y, z)만 제어

---

## 📌 Result
- 사용자 입력 좌표에 대해 MoveIt2가 motion planning을 수행
- 로봇암이 RViz 상에서 목표 위치로 정상 이동
- MoveGroup Action 결과 SUCCESS 확인

---

## 📌 Notes
- 입력 좌표는 로봇의 작업공간(workspace) 내의 값이어야 합니다.

---

## 📌 References
- Doosan Robotics ROS2: https://github.com/DoosanRobotics/doosan-robot2
- MoveIt2: https://github.com/moveit/moveit2