# bag_replay_pkg

This package stores:
- a packaged YOLO model at `models/best.pt`
- recorded ROS 2 bags under `bags/`
- a launch file for rerunning YOLO lane detection on recorded image topics

Typical workflow:
1. Record all remote topics into `bags/`
2. Replay the recorded bag
3. Launch `bag_lane_replay.launch.py`
4. Inspect raw images, YOLO overlays, and path overlays on the local PC
