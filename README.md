# Real-Time Industrial Vision Pipeline

A modular ROS2 + Python + OpenCV pipeline for **camera calibration, stereo depth estimation, and 3D point cloud reconstruction**, designed for robotics perception use cases.  
This project was implemented from scratch using standard computer vision methods and validated with publicly available stereo datasets.

---

## Features
- Stereo **camera calibration** (intrinsics & extrinsics)  
- **Image rectification** and undistortion  
- Real-time **stereo depth estimation** (Semi-Global Block Matching)  
- **3D point cloud reconstruction** from disparity + RGB  
- **Filtering** (voxel downsampling, pass-through) for cleaner perception  
- Modular ROS2 nodes written in Python  

---

## Repository Structure
```
real_time_vision_pipeline/
├─ README.md
├─ requirements.txt
├─ ros2_ws/
│  ├─ src/
│  │  └─ stereo_perception/
│  │     ├─ nodes/          # Python ROS2 nodes
│  │     ├─ calib/          # Calibration files
│  │     ├─ cfg/            # Parameters
│  │     ├─ launch/         # Launch files
│  │     └─ utils/          # Utility scripts
├─ data/
│  ├─ sample_pairs/         # Example stereo images (KITTI, Middlebury)
│  └─ bags/                 # ROS2 bag files (not tracked, link in README)
├─ docs/
│  ├─ pipeline_overview.png
│  ├─ depth_demo.gif
│  └─ pointcloud_demo.gif
```

---

## Getting Started

### 1. Setup
```bash
# Clone repo
git clone https://github.com/<your-username>/real_time_vision_pipeline.git
cd real_time_vision_pipeline/ros2_ws

# Install Python dependencies
pip install -r ../requirements.txt

# Build ROS2 workspace
colcon build
source install/setup.bash
```

### 2. Run the pipeline
```bash
ros2 launch stereo_perception demo_stereo.launch.py
```

### 3. Visualize in RViz2
- Add **Image** → `/stereo/depth`
- Add **PointCloud2** → `/stereo/points_filtered`

---

## ROS2 Topics
| Node                  | Publishes                          | Subscribes                        |
|-----------------------|------------------------------------|-----------------------------------|
| stereo_rectify_node   | /stereo/left_rect, /stereo/right_rect | /camera/left, /camera/right |
| depth_sgbm_node       | /stereo/disparity, /stereo/depth  | /stereo/left_rect, /stereo/right_rect |
| pointcloud_node       | /stereo/points_raw                | /stereo/depth, /stereo/left_rect  |
| pcl_filters_node      | /stereo/points_filtered           | /stereo/points_raw                |

---

## Benchmarks
| Resolution | Avg FPS | End-to-End Latency |
|------------|---------|--------------------|
| 640×360    | 25–30 FPS | ~30 ms |
| 1280×720   | 12–18 FPS | ~55 ms |

Measured on an i7 CPU, ROS2 Humble, Python implementation.

---

## Results
- **Depth Map Example**  
  ![Depth Demo](docs/depth_demo.gif)  

- **Point Cloud Example**  
  ![PointCloud Demo](docs/pointcloud_demo.gif)  

---

## Datasets Used
This project was tested on open datasets widely used in stereo vision research:
- **KITTI Stereo 2015** ([link](http://www.cvlibs.net/datasets/kitti/eval_scene_flow.php?benchmark=stereo)) – urban driving stereo image pairs with ground-truth disparity.  
- **Middlebury Stereo Dataset** ([link](https://vision.middlebury.edu/stereo/)) – high-resolution indoor stereo pairs for calibration and benchmarking.  

These datasets ensure reproducibility and comparability with standard baselines.

---

## Roadmap
- GPU-accelerated depth estimation (CUDA / RAFT-Stereo)  
- Semantic fusion: YOLO + depth for object-aware point clouds  
- C++ extensions for real-time deployment  
- Dockerized deployment  

---

## References & Inspiration
The implementation builds on standard vision algorithms and open-source resources:
- OpenCV stereo calibration & rectification docs  
- Stereo depth estimation methods (SGBM) as described in academic and tutorial resources  
- ROS2 Camera Calibration package for intrinsics/extrinsics extraction  
- Open-source point cloud conversion techniques  

All integration, pipeline design, and ROS2 node development were implemented in-house for this project.
