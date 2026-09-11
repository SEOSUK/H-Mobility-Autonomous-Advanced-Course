# H-Mobility Class Autonomous Driving
*ROS 2 기반 카메라 자율주행 시스템*

**프로젝트 기간:** 2026.08.12–08.14

## 🔹 Overview
**2026 H-Mobility Class 자율주행 심화과정 — Team 7** 최종평가를 위해 구현한 소형 자율주행 차량 시스템입니다.

<div align="center">
  <img src="https://github.com/user-attachments/assets/f15e8017-3315-4000-8948-433b6cbbdd33"
       alt="H-Mobility Team 7 System Overview"
       width="900">
</div>

- **차선 인지:** 카메라 영상에서 YOLO 기반 차선 segmentation 및 차선 중심점 추출
- **경로 생성 및 조향 제어:** Natural Cubic Spline 기반 경로 생성과 PID 조향 제어
- **이벤트 기반 주행:** 신호등 및 장애물 인식 결과를 활용한 정지, 통과, 최종 정지 로직 구현

## 🎥 주행 영상

https://github.com/user-attachments/assets/c31f68aa-fa91-4740-bb26-dfd4248b6e04

- 인지, 경로 생성, 차량 제어 모듈을 ROS 2 기반 파이프라인으로 통합하여 실차 주행 수행
- 검출된 차선 형상과 주행 이벤트를 기반으로 조향 및 속도 명령을 실시간 생성

## 🔹 차선 인지 및 경로 생성

<div align="center">
  <img src="https://github.com/user-attachments/assets/ddaaab28-c1ad-43c9-ab59-930984e0ec8f"
       alt="YOLO Lane Detection"
       width="850">
</div>

- **YOLO segmentation**을 이용해 목표 차선을 추출하고 Bird's-eye-view ROI에서 후처리
- 좌·우 차선 경계로부터 복수의 중심점을 추정하고 **Natural Cubic Spline**으로 부드러운 주행 경로 생성
- 생성된 경로의 기울기를 조향 오차로 사용하여 폐루프 차량 제어 수행

## 🔹 이벤트 인식 및 차량 제어

<div align="center">
  <img src="https://github.com/user-attachments/assets/ec8de6ab-3ad1-49e8-a6f8-e560fa65ca61"
       alt="YOLO Stop Trigger"
       width="600">
</div>

- 최종 주행 로직에서는 LiDAR에 의존하지 않고 카메라 기반으로 `red`, `green`, `obstacle` 이벤트 인식
- 차선 추정 노이즈의 영향을 줄이기 위해 미분항에 Low-pass Filter를 적용한 **PID 조향 제어** 구현
- 적색 신호 hold, 장애물 통과, 감속 및 최종 정지 등 이벤트별 상태 전이 로직 구성

## 🔹 Keywords
**ROS 2 · YOLO Segmentation · Path Planning · PID Steering Control · Event-driven Decision Making**
