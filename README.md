# H-Mobility Class Autonomous Driving
*ROS 2-Based Camera Autonomous Driving System*

**Project Period:** Aug. 12–14, 2026

## 🔹 Overview
A compact autonomous-driving pipeline developed for the **2026 H-Mobility Class Autonomous Driving Advanced Course — Team 7**.

<div align="center">
  <img src="https://github.com/user-attachments/assets/f15e8017-3315-4000-8948-433b6cbbdd33"
       alt="H-Mobility Team 7 System Overview"
       width="900">
</div>

- **Lane Perception:** YOLO-based lane segmentation and lane-center extraction from camera images.
- **Path & Steering Control:** Natural cubic spline-based path generation and PID steering control.
- **Event-driven Driving:** Camera-based recognition of traffic signals and obstacles for stop, pass, and final-stop behaviors.

## 🎥 Driving Demo

https://github.com/user-attachments/assets/c31f68aa-fa91-4740-bb26-dfd4248b6e04

- Integrated the perception, planning, and vehicle-control modules into a ROS 2 pipeline for real-vehicle driving.
- Steering and velocity commands were generated online from the detected lane geometry and driving events.

## 🔹 Lane Perception & Path Generation

<div align="center">
  <img src="https://github.com/user-attachments/assets/ddaaab28-c1ad-43c9-ab59-930984e0ec8f"
       alt="YOLO Lane Detection"
       width="850">
</div>

- Extracted the target lane using **YOLO segmentation** and processed the detected lane in a bird's-eye-view ROI.
- Estimated multiple lane-center points from the left/right lane boundaries and generated a smooth path using **natural cubic spline interpolation**.
- Used the generated path slope as the steering error for closed-loop vehicle control.

## 🔹 Event Detection & Vehicle Control

<div align="center">
  <img src="https://github.com/user-attachments/assets/ec8de6ab-3ad1-49e8-a6f8-e560fa65ca61"
       alt="YOLO Stop Trigger"
       width="600">
</div>

- Implemented camera-based detection of `red`, `green`, and `obstacle` events without relying on LiDAR for the final driving logic.
- Applied **PID steering control** with a low-pass-filtered derivative term to reduce sensitivity to noisy lane estimates.
- Added event-specific state transitions including red-light hold, obstacle passage, deceleration, and final-stop sequences.

## 🔹 Keywords
**ROS 2 Autonomous Driving · YOLO Segmentation · Path Planning · PID Steering Control · Event-driven Decision Making**

## 🔹 Project Notes
🔗 [Detailed implementation notes on Notion](https://app.notion.com/p/08-17-1-3d732dc77e4c805aa08bdea103ec40bf)
