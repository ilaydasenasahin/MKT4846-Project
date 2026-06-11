#Camera–LiDAR Fusion for Object Localization

A ROS 1 (Noetic) catkin workspace that detects, localizes and tracks objects (e.g.
traffic signs, traffic lights and vehicles) by **fusing a front camera with a 3D
LiDAR**. YOLOv8 supplies 2D detections and class labels from the image; the LiDAR
point cloud is filtered and clustered into 3D boxes; the two are fused by projecting
the LiDAR clusters into the image plane and associating them with the YOLO boxes. The
fused 3D detections are then accumulated into a global object map using vehicle odometry.

> **Note on authorship.** This is a shared team codebase (the package maintainer field
> points to the AESK autonomous team). This README and the accompanying technical report
> document the system and its run procedure; they are deliverables prepared on top of the
> existing code, not a claim of original authorship of the C++/Python nodes.

---

## 1. System architecture

```
                    ┌─────────────────────────┐
   Camera image ───►│  ultralytics_ros         │  /detection_result
                    │  tracker_node.py (YOLOv8)│  (vision_msgs/Detection2DArray)
                    └─────────────────────────┘            │
                                                           ▼
 /velodyne_points    ┌────────────────────┐   ┌──────────────────────┐
 (LiDAR) ───────────►│ point_cloud_filter │──►│ adaptive_clustering  │
                     │  (ROI box filter)  │   │ (nested-region        │
                     └────────────────────┘   │  Euclidean clustering)│
                       /roied_point_cloud      └──────────────────────┘
                                                 /marker_cube, /cloud_filtered
                                                           │
                                                           ▼
                                            ┌──────────────────────────┐
   velodyne→camera TF  ────────────────────►│  SignFilter_node         │
   camera intrinsics (yaml) ───────────────►│  project LiDAR clusters  │
                                            │  into image, match with  │
                                            │  YOLO boxes, fuse 3D+class│
                                            └──────────────────────────┘
                                                  /sign_filter
                                            (vision_msgs/Detection3DArray)
                                                  │            │
                              ┌───────────────────┘            ▼
                              ▼                     ┌──────────────────────┐
                     ┌─────────────────┐  /odom ───►│   global_tracker     │
                     │ detection_3d_viz│            │  odom-aided global    │
                     │  (RViz markers) │            │  object map / dedup   │
                     └─────────────────┘            └──────────────────────┘
                                                      /tracked_objects (+viz)
```

**Pipeline summary**

1. **YOLOv8 detection** (`ultralytics_ros/tracker_node.py`) — runs detection + traffic-light
   classification (HSV / CNN / HSI selectable) and publishes 2D boxes with class ids.
2. **Point-cloud ROI filter** (`point_cloud_filter_node`) — PCL pass-through box filter on
   x/y/z; republishes the cropped cloud with `x,y,z,intensity,time` fields.
3. **Adaptive clustering** (`adaptive_clustering_node`) — divides the cloud into nested
   circular regions and runs Euclidean clustering; publishes 3D cluster boxes as markers.
4. **Camera–LiDAR fusion** (`SignFilter_node`) — builds the projection matrix
   `P = K · [R | t]` from the camera intrinsics and the velodyne→camera transform, projects
   each cluster centroid into the image, associates it with the overlapping YOLO box, and
   emits a fused `Detection3DArray` (3D box from LiDAR + class from the camera).
5. **Global tracking** (`global_tracker`) — transforms fused detections into the global
   frame using `/odom`, deduplicates by distance/yaw, and confirms objects with a counter.
6. **Visualization** (`detection_3d_viz`, RViz) — renders fused/tracked objects.

---

## 2. Packages

| Package | Language | Key nodes (executables) | Role |
|---|---|---|---|
| `ultralytics_ros` (folder `yolov8_ros`) | Python | `tracker_node.py` | YOLOv8 2D detection + traffic-light classification |
| `aesk_object_localization` | C++ | `point_cloud_filter_node`, `adaptive_clustering_node`, `SignFilter_node`, `global_tracker`, `lidar_tracker` | LiDAR processing, camera–LiDAR fusion, global tracking |
| `aesk_visualization` | C++ | `detection_3d_viz` | RViz marker visualization of 3D detections |

`lidar_tracker` is an Open3D-based standalone LiDAR tracker that is built but not part of the
main fusion launch file. The `scripts/*.py` files (`clustering_dbscan`, `clustering_nearest`,
`clustering_park`, `clustering_signs`, `visualize`) are auxiliary/experimental tools.

Custom message: `aesk_object_localization/ClusterArray.msg`.

---

## 3. Dependencies

### 3.1 Platform
- **Ubuntu 20.04** with **ROS Noetic** (the CMake/headers target Noetic; some comments
  still reference Melodic).
- `catkin` build tools, CMake ≥ 3.0.2, a C++14 compiler.

### 3.2 System libraries
| Library | Why | Install |
|---|---|---|
| **PCL** (Point Cloud Library) | ROI filter, Euclidean clustering | `sudo apt install libpcl-dev ros-noetic-pcl-ros ros-noetic-pcl-conversions` |
| **OpenCV 4** | image handling, projection math | `sudo apt install libopencv-dev` (CMake expects `/usr/lib/x86_64-linux-gnu/cmake/opencv4`) |
| **Open3D** | `lidar_tracker`, `PointCloudOpen3D` utilities | build/install from source to `/usr/local` (CMake expects `/usr/local/lib/cmake/Open3D`) |
| **Eigen 3** | linear algebra (vendored under `include/Eigen`) | header-only, already included |

### 3.3 ROS packages (apt: `ros-noetic-<name>`)
`roscpp`, `rospy`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`,
`vision-msgs`, `visualization-msgs`, `cv-bridge`, `image-transport`, `image-geometry`,
`image-view`, `message-filters`, `message-generation`, `message-runtime`,
`actionlib`, `actionlib-msgs`, `dynamic-reconfigure`, `tf`, `tf2`, `tf2-ros`,
`tf2-geometry-msgs`.

Install in one go:
```bash
sudo apt update && sudo apt install -y \
  ros-noetic-pcl-ros ros-noetic-pcl-conversions \
  ros-noetic-vision-msgs ros-noetic-cv-bridge \
  ros-noetic-image-transport ros-noetic-image-geometry ros-noetic-image-view \
  ros-noetic-message-filters ros-noetic-dynamic-reconfigure \
  ros-noetic-tf ros-noetic-tf2-ros ros-noetic-tf2-geometry-msgs \
  libpcl-dev libopencv-dev
```

### 3.4 External ROS packages (referenced but **not** included in this zip)
These must be present in the workspace `src/` for a full build/run:
- **`darknet_ros_msgs`** — build dependency of `aesk_object_localization`
  (`ros-noetic-darknet-ros-msgs` or the `darknet_ros` repo).
- **`aesk_camera_driver`** — provides the `camera_driver` node and `/pylon_undistorted`
  image used by `SignFilter.launch`. Without it, supply an undistorted image on
  `/pylon_undistorted` from another source.

### 3.5 Python dependencies (YOLOv8 node)
From `yolov8_ros/requirements.txt`:
```
lap==0.4.0
onnx==1.14.0
urllib3==1.26.11
numpy==1.23.4
opencv-python==4.7.0.72
ultralytics
```
Plus `torch` / `torchvision` (CUDA build recommended for real-time inference) and the ROS
Python bindings (`rospy`, `cv_bridge`). Install with:
```bash
python3 -m pip install -r src/yolov8_ros/requirements.txt
python3 -m pip install torch torchvision    # pick the CUDA/CPU build for your machine
```

### 3.6 Model weights
The YOLO node loads weights from `yolov8_ros/models/` (`best_15.pt` for detection,
`best.pt` for classification, set in `config/yolo.yaml`). Place the trained `.pt` files there.

---

## 4. Build

> The archive is double-nested: the catkin workspace root is the **inner** `ilayda_mkt_ws/`
> that contains `src/` and `.catkin_workspace`.

```bash
# 1) Go to the workspace root (the one that contains src/)
cd mkt4846_project_ws/      # adjust if you flattened the folder

# 2) Add the missing external packages into src/ if needed
#    (darknet_ros_msgs, aesk_camera_driver)

# 3) Install Python deps for the YOLO node
python3 -m pip install -r src/yolov8_ros/requirements.txt

# 4) Resolve ROS deps and build
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src -r -y   # optional but recommended
catkin_make            # or: catkin build

# 5) Source the workspace
source devel/setup.bash
```

---

## 5. Run

Start a ROS master and play your data / start your sensors first:
```bash
roscore                       # terminal 1
rosbag play your_data.bag     # terminal 2 (or launch live sensor drivers)
```

### 5.1 Full fusion pipeline (recommended)
Brings up TF, camera driver, YOLO, ROI filter, clustering, fusion, global tracker and RViz:
```bash
roslaunch aesk_object_localization SignFilter.launch
```
> Requires `aesk_camera_driver` and the YOLO weights to be present. If you do not have the
> camera driver, run the nodes individually (below) and feed `/pylon_undistorted` yourself.

### 5.2 YOLO detector only
```bash
roslaunch ultralytics_ros tracker.launch debug:=true
```

### 5.3 Individual nodes
```bash
rosrun aesk_object_localization point_cloud_filter_node \
  _point_cloud_sub_topic:=/velodyne_points          # or load point_cloud_filter.yaml
rosrun aesk_object_localization adaptive_clustering_node
rosrun aesk_object_localization SignFilter_node
rosrun aesk_object_localization global_tracker
rosrun aesk_visualization detection_3d_viz _detection_3d_topic:=/sign_filter
```
Each node expects its YAML loaded as private params, e.g.:
```bash
rosrun aesk_object_localization point_cloud_filter_node \
  __ns:=/ _:="$(rospack find aesk_object_localization)/config/point_cloud_filter.yaml"
```
(or use a launch file with `<rosparam command="load" file="..."/>`, as in `SignFilter.launch`).

---

## 6. Topics

| Topic | Type | Produced by | Consumed by |
|---|---|---|---|
| `/velodyne_points` | `sensor_msgs/PointCloud2` | LiDAR driver | `point_cloud_filter_node` |
| `/roied_point_cloud` | `sensor_msgs/PointCloud2` | `point_cloud_filter_node` | `adaptive_clustering_node` |
| `/cloud_filtered` | `sensor_msgs/PointCloud2` | `adaptive_clustering_node` | `SignFilter_node` |
| `/marker_cube` | `visualization_msgs/MarkerArray` | `adaptive_clustering_node` | `SignFilter_node` |
| `/detection_result` | `vision_msgs/Detection2DArray` | YOLO `tracker_node.py` | `SignFilter_node` |
| `/pylon_undistorted` | `sensor_msgs/Image` | `aesk_camera_driver` (external) | YOLO, `SignFilter_node` |
| `/sign_filter` | `vision_msgs/Detection3DArray` | `SignFilter_node` | `global_tracker`, `detection_3d_viz` |
| `/odom` | `nav_msgs/Odometry` | localization/odometry source | `global_tracker` |
| `/tracked_objects` (+ `_viz`) | `vision_msgs/Detection3DArray`, `MarkerArray` | `global_tracker` | RViz / downstream |

---

## 7. Configuration

| File | Controls |
|---|---|
| `yolov8_ros/config/yolo.yaml` | models, image/detection topics, conf/IoU thresholds, tracker (`bytetrack.yaml`), classification type (`hsv`/`cnn`/`hsi`) |
| `aesk_object_localization/config/point_cloud_filter.yaml` | input/output topics, ROI box (`xmin..zmax`) |
| `aesk_object_localization/config/adaptive_clustering.yaml` | z-limits, cluster size min/max, topic names |
| `aesk_object_localization/config/sign_filter.yaml` | **camera intrinsics**, image size, ROI, cluster sizes, `camera_frame_id` |
| `aesk_object_localization/config/global_tracker.yaml` | detection/odom topics, `set_counter` confirmation threshold |

**Calibration:** the camera intrinsics live in `sign_filter.yaml`; the velodyne→camera
**extrinsic** is published as a `static_transform_publisher` in `SignFilter.launch`. Both must
match your sensor rig for the projection-based fusion to associate boxes correctly.

---

## 8. Troubleshooting

- **`catkin_make` fails on `darknet_ros_msgs`** → add that package to `src/` (it is an
  external build dependency).
- **`Open3D` not found** → install/build Open3D to `/usr/local`, or comment out the
  `lidar_tracker`/Open3D targets in `aesk_object_localization/CMakeLists.txt` if you only
  need the fusion pipeline.
- **No fused detections on `/sign_filter`** → check that (a) YOLO is publishing on
  `/detection_result`, (b) clusters appear on `/marker_cube`, and (c) the velodyne→camera TF
  and intrinsics are correct (mis-calibration pushes projected points outside the boxes).
- **YOLO node errors** → confirm the `.pt` weights are in `yolov8_ros/models/` and the
  Python/torch versions match `requirements.txt`.

---

## 9. Repository layout

```
mkt4846_project_ws/
└── src/
    ├── CMakeLists.txt                  (catkin toplevel symlink)
    ├── yolov8_ros/                     YOLOv8 detection (Python, pkg name: ultralytics_ros)
    │   ├── script/tracker_node.py
    │   ├── config/yolo.yaml
    │   ├── launch/tracker.launch
    │   ├── models/                     *.pt weights
    │   └── requirements.txt
    ├── aesk_object_localization/       LiDAR processing + fusion + tracking (C++)
    │   ├── src/PointCloudFilter/       ROI box filter
    │   ├── src/AdaptiveClustering/     nested-region Euclidean clustering
    │   ├── src/SignFilter/             camera–LiDAR projection + fusion
    │   ├── src/GlobalTracker/          odom-aided global object map
    │   ├── src/LidarTracker/           Open3D LiDAR tracker (standalone)
    │   ├── config/  launch/  msg/  scripts/  rviz/  include/
    └── aesk_visualization/             RViz markers for 3D detections (C++)
```
