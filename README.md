# Real-Time Industrial Vision Pipeline

A ROS2 + OpenCV based pipeline for **camera calibration, stereo depth estimation, and 3D point cloud reconstruction**, designed for real-time robotic perception.

---

## Features
- Stereo **camera calibration** (intrinsics & extrinsics) with OpenCV
- **Image rectification** and undistortion
- Real-time **stereo depth estimation** using SGBM
- **3D point cloud generation** from disparity + RGB
- **Filtering** with voxel downsampling and pass-through limits
- Modular ROS2 nodes for easy extension and benchmarking

---

## Repository Structure
```
real_time_vision_pipeline/
├─ README.md
├─ requirements.txt
├─ ros2_ws/
│  ├─ src/
│  │  └─ stereo_perception/
│  │     ├─ nodes/          # ROS2 nodes (Python)
│  │     ├─ calib/          # Calibration files
│  │     ├─ cfg/            # Parameter configs
│  │     ├─ launch/         # ROS2 launch files
│  │     └─ utils/          # Utility scripts
├─ data/
│  ├─ sample_left_right/    # Example stereo pairs
│  └─ bags/                 # Sample rosbag2 recordings
├─ docs/
│  ├─ pipeline_overview.png
│  ├─ topics_graph.png
│  ├─ depth_demo.gif
│  └─ pointcloud_demo.gif
```

---

## Quickstart

### 1. Setup
```bash
# Clone repo
git clone https://github.com/<your-username>/real_time_vision_pipeline.git
cd real_time_vision_pipeline/ros2_ws

# Install dependencies
pip install -r ../requirements.txt

# Build ROS2 workspace
colcon build
source install/setup.bash
```

### 2. Run Demo
```bash
ros2 launch stereo_perception demo_stereo.launch.py
```

### 3. Visualize
Open `rviz2` and add:
- Image → `/stereo/depth`
- PointCloud2 → `/stereo/points_filtered`

---

## ROS2 Topics
| Node                  | Publishes                          | Subscribes                        |
|-----------------------|------------------------------------|-----------------------------------|
| camera_sync_node      | /camera/left, /camera/right       | raw images                        |
| stereo_rectify_node   | /stereo/left_rect, /stereo/right_rect, /stereo/Q | /camera/left, /camera/right |
| depth_sgbm_node       | /stereo/disparity, /stereo/depth  | /stereo/left_rect, /stereo/right_rect |
| pointcloud_node       | /stereo/points_raw                | /stereo/depth, /stereo/left_rect  |
| pcl_filters_node      | /stereo/points_filtered           | /stereo/points_raw                |

---

## Benchmarks
| Mode | Resolution | Avg FPS | End-to-End Latency (ms) |
|------|------------|---------|--------------------------|
| SGBM | 640×360    | 30 FPS  | 28 ms                   |
| SGBM | 1280×720   | 15 FPS  | 55 ms                   |

---

## Results
- **Depth Map Example**  
  ![Depth Demo](docs/depth_demo.gif)  

- **Filtered Point Cloud**  
  ![PointCloud Demo](docs/pointcloud_demo.gif)  

---

## Roadmap
- GPU accelerated stereo (CUDA / RAFT-Stereo)
- Semantic colorized point clouds (YOLO + depth fusion)
- Python PCL node implementation for higher performance
- Dockerized deployment for easy setup

---

## References
- [OpenCV Stereo Calibration](https://docs.opencv.org/master/d9/d0c/group__calib3d.html)
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Point Cloud Library (PCL)](https://pointclouds.org/)
