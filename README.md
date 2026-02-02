# SAFETRACKER (Autonomous Road Manager)

**SAFETRACKER**는 커스텀 **Turtlebot3 Burger** 플랫폼을 기반으로 한 도로 관리용 자율주행 로봇 프로젝트
스테레오 카메라(Stereo Camera)를 통한 깊이 추정(Depth Estimation)과 LiDAR 센서 퓨전을 통해 주행 환경을 인식하며, 물리/가상 범퍼 시스템을 도입하여 안전한 자율주행을 구현

<img width="480" alt="KakaoTalk_20260203_041440322_04-Photoroom" src="https://github.com/user-attachments/assets/c49f8a88-9174-4ec4-bdcb-75e8c35d82ab" />


<div align="center">
  <a href="https://www.youtube.com/watch?v=6cw76WwYq2I">
    <img src="https://img.youtube.com/vi/6cw76WwYq2I/maxresdefault.jpg" width="80%" alt="SAFETRACKER Demo Video" style="border-radius: 10px; box-shadow: 0 4px 8px 0 rgba(0, 0, 0, 0.2), 0 6px 20px 0 rgba(0, 0, 0, 0.19);">
  </a>
  <br>
  <b>Click the image to watch the demo video</b>
</div>

<br>

<div align="center">
  <img src="https://img.shields.io/badge/ROS2-Humble-22314E?style=for-the-badge&logo=ros&logoColor=white">
  <img src="https://img.shields.io/badge/Python-3.10+-3776AB?style=for-the-badge&logo=python&logoColor=white">
  <img src="https://img.shields.io/badge/Platform-Turtlebot3-green?style=for-the-badge&logo=robot&logoColor=white">
  <img src="https://img.shields.io/badge/Sensor-Stereo%20Camera%20%26%20LiDAR-orange?style=for-the-badge">
</div>

---

## Key Features

### 1. Stereo Vision Depth Estimation
- **Stereo Camera Calibration:** 듀얼 카메라의 내/외부 파라미터를 정밀하게 보정하여 왜곡을 제거
- **Depth Map Generation:** 좌/우 영상의 시차(Disparity)를 계산하여 객체까지의 거리를 픽셀 단위로 추정
- 값비싼 3D LiDAR 없이도 전방 장애물의 거리 정보를 획득

### 2. Sensor Fusion & LiDAR
- 2D LiDAR 데이터를 발행(Publish)하여 로봇 주변 360도 맵핑 및 위치 추정(SLAM)을 보조
- 카메라의 시각 정보와 LiDAR의 거리 정보를 결합하여 인식률을 향상

### 3. Steering Bumper System
- **Safety Mechanism:** 주행 중 충돌 위험을 감지하거나 물리적 접촉이 발생했을 때 즉각적으로 반응하는 조향 시스템
- 장애물 회피 또는 긴급 정지 로직이 포함된 안전장치

---

## 📂 Project Structure

프로젝트는 크게 자율주행 알고리즘 파트와 시스템 제어 파트로 구분

```bash
SAFETRACKER
├── 📂 safetracker_auto_drive          # 자율주행 핵심 알고리즘 및 주행 스크립트
├── 📂 safetracker_control_system      # ROS 2 워크스페이스 및 제어 패키지
│   └── safetracker_ws
│       └── src                        # 센서 드라이버 및 노드 소스코드
└── 📄 README.md
```

---

## Hardware Setup
Component,Model / Description
Mobile Base,Robotis Turtlebot3 Burger (Modified)
Vision Sensor,Custom Stereo Camera Module
Lidar,LDS-01 / LDS-02
Controller, / Jetson Nano
