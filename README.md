Original implementation: ROS 1 Noetic
Target implementation: ROS 2 Humble
Operating system used for the port: Ubuntu 22.04

This document is the definitive handoff manual for the LVI-SAM ROS 1 Noetic to ROS 2 Humble port. It details the transformation history, the procedure for validating the port using a pre-recorded ROS 2 bag file, and the integration contract required to connect physical edge-device sensors (LiDAR and IMU) to the LVI-SAM processing pipeline.

## 1. ROS 1 Noetic to ROS 2 Humble Transformation

This section serves as a comprehensive migration reference documenting the exact changes made to modernize the LVI-SAM codebase from ROS 1 to ROS 2.

### 1.1 Original ROS 1 and Target ROS 2 Environment

Original:
ROS 1 Noetic
Catkin
ros::*
tf::*
ROS 1 message generation

Target:
ROS 2 Humble
ament_cmake
rclcpp
tf2/tf2_ros
ROS 2 interface package

The `catkin_ws/` directory is retained in this repository as the historical ROS 1 reference. The `ros2_ws/` directory contains the active ROS 2 port.

### 1.2 Repository Structure

```text
Slam/
├── catkin_ws/
│   └── src/
│       ├── LVI-SAM/
│       └── livox_ros_driver/
├── gtsam-4.0.2/
├── ros2_ws/
│   └── src/
│       ├── lvi_sam/
│       └── lvi_sam_interfaces/
└── README.md
```

- `catkin_ws/src/LVI-SAM`: The original ROS 1 reference code.
- `ros2_ws/src/lvi_sam`: The active ROS 2 Humble C++ implementation.
- `ros2_ws/src/lvi_sam_interfaces`: The newly created ROS 2 custom message package.
- `gtsam-4.0.2/`: The required Georgia Tech Smoothing and Mapping dependency.

### 1.3 Complete File-by-File Migration Table

| ROS 1/reference file | ROS 2/current file | Major change | Why |
| --- | --- | --- | --- |
| `src/lidar_odometry/utility.h` | `src/lidar_odometry/utility.h` | ROS APIs, messages, parameters, time, TF2 | ROS 2 API migration |
| `src/lidar_odometry/featureExtraction.cpp` | `src/lidar_odometry/featureExtraction.cpp` | `rclcpp`, subscriptions, publishers | ROS 2 node model |
| `src/lidar_odometry/imageProjection.cpp` | `src/lidar_odometry/imageProjection.cpp` | ROS 2 API and messages | ROS 2 node model |
| `src/lidar_odometry/imuPreintegration.cpp` | `src/lidar_odometry/imuPreintegration.cpp` | ROS 2 APIs, TF2, time | ROS 2 API migration |
| `src/lidar_odometry/mapOptmization.cpp` | `src/lidar_odometry/mapOptmization.cpp` | ROS 2 APIs, TF2 | ROS 2 API migration |
| `msg/cloud_info.msg` | `msg/CloudInfo.msg` in `lvi_sam_interfaces` | Interface migration | ROS 2 interface generation |
| `CMakeLists.txt` | `CMakeLists.txt` | Catkin to ament | ROS 2 build system |
| `package.xml` | `package.xml` | Dependency declarations | ROS 2 packaging |

### 1.4 Catkin to ament_cmake

The build system was entirely rewritten. The original ROS 1 CMake pattern utilized Catkin:

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  ...
)
catkin_package(...)
```

The current ROS 2 structure utilizes `ament_cmake`:

```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
...
find_package(lvi_sam_interfaces REQUIRED)

ament_target_dependencies(lvi_sam_mapOptmization ${AMENT_DEPS})
install(TARGETS lvi_sam_mapOptmization DESTINATION lib/${PROJECT_NAME})

ament_package()
```

This change was necessary because ROS 2 uses the `ament` build system and `colcon` build tool in place of `catkin_make`.

The build command used to compile from a clean workspace:

```bash
cd ~/Documents/SIH/Slam/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select lvi_sam_interfaces
MAKEFLAGS="-j4" colcon build --packages-select lvi_sam --event-handlers console_direct+
```

### 1.5 ROS 1 Interface to ROS 2 Interface

ROS 2 imposes strict naming conventions for messages. The custom message was separated into its own ROS 2 package to ensure downstream compatibility.

ROS 1:
`lvi_sam/msg/cloud_info.msg`

ROS 2:
`lvi_sam_interfaces/msg/CloudInfo.msg`

Field names were strictly migrated to snake_case:

```text
startRingIndex  -> start_ring_index
endRingIndex    -> end_ring_index
pointColInd     -> point_col_ind
pointRange      -> point_range
cloudDeskewed   -> cloud_deskewed
cloudCorner     -> cloud_corner
cloudSurface    -> cloud_surface
```

The exact verification command used to confirm the interface:

```bash
source /opt/ros/humble/setup.bash
source ~/Documents/SIH/Slam/ros2_ws/install/setup.bash
ros2 interface show lvi_sam_interfaces/msg/CloudInfo
```

### 1.6 ROS 1 NodeHandle to ROS 2 Node

ROS 1 utilized a global node handle system. ROS 2 utilizes an object-oriented model where classes inherit from or hold references to `rclcpp::Node`.

Old ROS 1 style:
```cpp
ros::NodeHandle nh;
```

New ROS 2 style:
```cpp
rclcpp::Node::SharedPtr node_;
```

The `ParamServer` base class was modified to accept a node pointer upon construction so it can declare and fetch parameters, replacing the legacy `ros::NodeHandle::param` behavior:

```cpp
explicit ParamServer(const rclcpp::Node::SharedPtr& node)
    : node_(node)
{
    ...
}
```

### 1.7 Main Function Migration

Across all four nodes (`featureExtraction.cpp`, `imageProjection.cpp`, `imuPreintegration.cpp`, `mapOptmization.cpp`), the initialization sequences were updated.

Old ROS 1:
```cpp
ros::init(argc, argv, "lidar");
ros::spin();
```

Current ROS 2:
```cpp
rclcpp::init(argc, argv);
auto node = rclcpp::Node::make_shared("lidar");
// Node initialization logic...
rclcpp::spin(node);
rclcpp::shutdown();
```

### 1.8 Publisher and Subscriber Migration

Message types were updated to include the `msg` namespace segment, and ROS APIs were translated to their `rclcpp` counterparts.

Old ROS 1:
```cpp
ros::Publisher pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("topic", 1);
ros::Subscriber subImu = nh.subscribe("topic", 100, &Class::Callback, this);
```

New ROS 2:
```cpp
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudSurround;
pubLaserCloudSurround = node_->create_publisher<sensor_msgs::msg::PointCloud2>("topic", 1);

rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subImu;
subImu = node_->create_subscription<sensor_msgs::msg::Imu>(
    "topic", rclcpp::SensorDataQoS(), std::bind(&Class::Callback, this, std::placeholders::_1));
```

Message namespace changes:
```text
sensor_msgs::PointCloud2 -> sensor_msgs::msg::PointCloud2
sensor_msgs::Imu -> sensor_msgs::msg::Imu
nav_msgs::Odometry -> nav_msgs::msg::Odometry
```

### 1.9 Callback Migration

ROS 2 heavily relies on smart pointers (`std::shared_ptr`) for memory-safe message passing between nodes, eliminating the ROS 1 `ConstPtr&` references.

ROS 1:
```cpp
void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg)
```

ROS 2:
```cpp
void imuHandler(const sensor_msgs::msg::Imu::ConstSharedPtr imuMsg)
```

### 1.10 Parameter API Migration

In ROS 2, parameters must be explicitly declared before they can be retrieved. 

Example implementation in `ParamServer`:
```cpp
node_->declare_parameter<std::string>("PROJECT_NAME", "sam");
PROJECT_NAME = node_->get_parameter("PROJECT_NAME").as_string();

node_->declare_parameter<bool>("lidar.use_sim_time", false);
bool use_sim_time = node_->get_parameter("lidar.use_sim_time").as_bool();

node_->declare_parameter<std::vector<double>>("lidar.extrinsicRot", std::vector<double>());
std::vector<double> extRotV = node_->get_parameter("lidar.extrinsicRot").as_double_array();
```

### 1.11 Time API Migration

ROS 2 introduces a strict clock and time API that replaces the `.toSec()` helpers from ROS 1. Time conversion is essential for temporal alignment between the IMU preintegration factors and the deskewing of LiDAR sweeps.

ROS 1:
```cpp
double time = msg->header.stamp.toSec();
```

ROS 2:
```cpp
double time = rclcpp::Time(msg->header.stamp).seconds();
```

A compatibility macro `ROS_TIME` was updated in `utility.h` to streamline this extraction across the codebase.

### 1.12 TF1 to TF2

The legacy `tf` library was removed in ROS 2. All transformations were migrated to `tf2` and `tf2_ros`.

ROS 1:
```cpp
tf::TransformBroadcaster tfMap2Odom;
```

ROS 2:
```cpp
#include <tf2_ros/transform_broadcaster.h>
// ...
std::unique_ptr<tf2_ros::TransformBroadcaster> tfMap2Odom;
```

Because `tf2_ros::TransformBroadcaster` does not have a default constructor and requires a node pointer, it must be initialized dynamically:
```cpp
tfMap2Odom = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
```

Current tracked transforms include `map -> odom` and `odom -> base_link`.

### 1.13 Logging Migration

All logging macros were converted to the node-aware `RCLCPP` equivalents.

ROS 1:
```cpp
ROS_INFO("Started");
ROS_WARN("Warning");
```

ROS 2:
```cpp
RCLCPP_INFO(node_->get_logger(), "Started");
RCLCPP_WARN(node_->get_logger(), "Warning");
```

### 1.14 Point Cloud Helper Migration

The `publishCloud` helper function was modernized to correctly manage ROS 2 header types during PCL-to-ROS conversions.

```cpp
template<typename T>
sensor_msgs::msg::PointCloud2 publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr thisPub, T thisCloud, rclcpp::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::msg::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->get_subscription_count() != 0)
        thisPub->publish(tempCloud);
    return tempCloud;
}
```

### 1.15 PCL and pcl_conversions

To interface Point Cloud Library (PCL) data with ROS 2 messages, the `pcl_conversions` headers were utilized:
```cpp
#include <pcl_conversions/pcl_conversions.h>
```

In the current CMake configuration, we explicitly include:
```text
/opt/ros/humble/include/pcl_conversions/pcl_conversions
/opt/ros/humble/include/pcl_msgs
${PCL_INCLUDE_DIRS}
```

During the build process, CMake generates minor warnings regarding VTK (`Could not find a package configuration file provided by "VTK"`). These originate upstream from the system's `libpcl-dev` configuration and can be safely ignored as they do not affect the compilation of LVI-SAM.

### 1.16 GTSAM and Eigen Compatibility

LVI-SAM requires GTSAM 4.0.2. Due to strict constraints on Eigen versions, the codebase links against:
- GTSAM 4.0.2
- GTSAM's bundled Eigen

This is explicitly handled in the `CMakeLists.txt` via:
```cmake
${CMAKE_SOURCE_DIR}/../../../gtsam-4.0.2/gtsam/3rdparty/Eigen
```
Exact historical command was not preserved; the transformation is documented from the source and repository state.

### 1.17 Runtime Segmentation Fault: Diagnosis and Fix

During the initial ROS 2 port verification, running `featureExtraction` resulted in a crash:
```bash
ros2 run lvi_sam lvi_sam_featureExtraction
```
Result:
```text
[ros2run]: Segmentation fault
```

Diagnostic command used:
```bash
gdb -q -batch -ex run -ex bt --args install/lvi_sam/lib/lvi_sam/lvi_sam_featureExtraction
```

The backtrace entered `ParamServer`, `Eigen::Matrix3d`, and `Eigen::Map`. The root cause was identified in `utility.h`. The extrinsic parameters were declared with empty vectors:
```cpp
extrinsicRot = std::vector<double>()
extrinsicRPY = std::vector<double>()
extrinsicTrans = std::vector<double>()
```
When `Eigen::Map` attempted to project these 0-length vectors into 3x3 and 3x1 matrices, it caused an illegal memory access.

The defaults were corrected to provide safe 3x3 identity matrices and a zero translation vector:
```cpp
// extrinsicRot
std::vector<double>{
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0
}

// extrinsicRPY
std::vector<double>{
    1.0, 0.0, 0.0,
    0.0, 1.0, 0.0,
    0.0, 0.0, 1.0
}

// extrinsicTrans
std::vector<double>{0.0, 0.0, 0.0}
```

IMPORTANT: These defaults are safe startup values only. For physical hardware integration, they must be replaced with the real calibrated LiDAR-to-IMU physical extrinsics.

### 1.18 Build Verification

The successful verification of the codebase compilation:

```bash
cd ~/Documents/SIH/Slam/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

MAKEFLAGS="-j4" colcon build --packages-select lvi_sam --event-handlers console_direct+
```

All four executables built without error:
- `lvi_sam_featureExtraction`
- `lvi_sam_imageProjection`
- `lvi_sam_imuPreintegration`
- `lvi_sam_mapOptmization`

### 1.19 Individual Runtime Startup Tests

These four tests were individual startup tests. They were not a bag replay. They establish that all four executables can successfully initialize under ROS 2 Humble without segmentation faults.

#### 1.19.a Feature Extraction Startup Test
```bash
ros2 run lvi_sam lvi_sam_featureExtraction
```
Expected: `----> Lidar Feature Extraction Started.`

#### 1.19.b Image Projection Startup Test
```bash
ros2 run lvi_sam lvi_sam_imageProjection
```
Expected: `----> Lidar Cloud Deskew Started.`

#### 1.19.c IMU Preintegration Startup Test
```bash
ros2 run lvi_sam lvi_sam_imuPreintegration
```
Expected: `----> IMU Preintegration Started.`

#### 1.19.d Map Optimization Startup Test
```bash
ros2 run lvi_sam lvi_sam_mapOptmization
```
Expected: `----> Lidar Map Optimization Started.`

### 1.20 ROS Graph Verification

With all four nodes running in separate terminals, the graph was inspected:
```bash
ros2 node list
ros2 topic list
```

Observed node names:
```text
/lidar
/lidar
/lidar
/lidar
```
ROS 2 issues a warning about duplicate node names. This is an active cleanup item, but it does not prevent node execution.

Observed active topics confirming pipeline connectivity:
```text
/points_raw
/imu_correct
/sam/lidar/deskew/cloud_deskewed
/sam/lidar/deskew/cloud_info
/sam/lidar/feature/cloud_corner
/sam/lidar/feature/cloud_surface
/sam/lidar/mapping/cloud_registered
/sam/lidar/mapping/map_global
/sam/lidar/mapping/odometry
```

## 2. Setting Up and Testing with the ROS 2 Bag File

This section details the procedure for conducting an end-to-end dataset replay. It distinguishes the individual node startup tests from the actual bag replay.

### 2.1 External Hard Drive and Dataset Preparation

The dataset is stored on an external hard drive and must be copied into the local environment.

#### 2.1.a Identify the External Drive
List available drives:
```bash
lsblk -f
```
Check mount points:
```bash
ls -lah /media/$USER/
```
If the drive is not automatically mounted, use `udisksctl` (replacing `/dev/sdXN` with the actual block device):
```bash
udisksctl mount -b /dev/sdXN
```

#### 2.1.b Find ROS Bag Files on the External Drive
```bash
find /media/$USER \( -name "*.db3" -o -name "metadata.yaml" -o -name "*.bag" \) -print
```

ROS 2 `rosbag2` utilizes `metadata.yaml` and `.db3` (SQLite3) database files. ROS 1 utilized `.bag` files. Do not attempt to run a legacy ROS 1 `.bag` file directly with `ros2 bag play` without converting it first.

#### 2.1.c Create Local Dataset Directory
```bash
mkdir -p ~/Documents/SIH/Slam/datasets
```

#### 2.1.d Copy the Dataset
Replace `DRIVE_NAME` and `LVI_SAM_DATASET` with your actual drive and folder names:
```bash
rsync -avh --progress /media/$USER/DRIVE_NAME/LVI_SAM_DATASET/ ~/Documents/SIH/Slam/datasets/LVI_SAM_DATASET/
```
Verify the contents exist:
```bash
ls -lah ~/Documents/SIH/Slam/datasets/LVI_SAM_DATASET/
```

### 2.2 Bag Inspection Before Replay

#### 2.2.a Source ROS 2
```bash
cd ~/Documents/SIH/Slam/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

#### 2.2.b Inspect Bag
```bash
ros2 bag info ~/Documents/SIH/Slam/datasets/LVI_SAM_DATASET
```
Verify the storage format, duration, message count, topic names, and message types.

#### 2.2.c Determine the Actual LiDAR and IMU Topics
From the output of `ros2 bag info`, locate the topics containing the point clouds and IMU measurements. Compare them against LVI-SAM's default expected inputs: `/points_raw` and `/imu_correct`.

### 2.3 Individual Node Startup Tests Are Separate from Bag Replay

The execution of the four `ros2 run` commands are purely startup tests. They do not process dataset data until a bag file is actively played and data is published to their subscribed topics.

### 2.4 Full LVI-SAM Pipeline for Bag Replay

The entire pipeline must be active simultaneously to process the bag file. Open four separate terminals.

#### 2.4.a Terminal 1: Map Optimization
```bash
cd ~/Documents/SIH/Slam/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run lvi_sam lvi_sam_mapOptmization
```

#### 2.4.b Terminal 2: Image Projection / Deskew
```bash
cd ~/Documents/SIH/Slam/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run lvi_sam lvi_sam_imageProjection
```

#### 2.4.c Terminal 3: IMU Preintegration
```bash
cd ~/Documents/SIH/Slam/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run lvi_sam lvi_sam_imuPreintegration
```

#### 2.4.d Terminal 4: Feature Extraction
```bash
cd ~/Documents/SIH/Slam/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run lvi_sam lvi_sam_featureExtraction
```

### 2.5 ROS Graph Verification Before Playback

Open a fifth terminal:
```bash
cd ~/Documents/SIH/Slam/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 topic info /points_raw -v
ros2 topic info /imu_correct -v
ros2 topic info /sam/lidar/deskew/cloud_info -v
```

Before playback, it is correct and expected for `/points_raw` and `/imu_correct` to have a publisher count of 0.

### 2.6 Bag Playback

Using the fifth terminal, initiate playback.

#### 2.6.a Standard Replay
```bash
ros2 bag play ~/Documents/SIH/Slam/datasets/LVI_SAM_DATASET
```

#### 2.6.b Replay at a Slower/Faster Rate
Slower playback is useful for debugging high-frequency processing drops on weaker CPUs:
```bash
ros2 bag play ~/Documents/SIH/Slam/datasets/LVI_SAM_DATASET -r 0.5
```
Or normal speed:
```bash
ros2 bag play ~/Documents/SIH/Slam/datasets/LVI_SAM_DATASET -r 1.0
```

#### 2.6.c Topic Remapping if the Dataset Uses Different Names
If your dataset published LiDAR on `/velodyne_points` instead of `/points_raw`, you can remap it seamlessly during playback:
```bash
ros2 bag play ~/Documents/SIH/Slam/datasets/LVI_SAM_DATASET \
  --remap /velodyne_points:=/points_raw \
  --remap /os_cloud_node/imu:=/imu_correct
```

### 2.7 Verify That Sensor Data Is Actually Arriving

#### 2.7.a LiDAR Rate
```bash
ros2 topic hz /points_raw
```

#### 2.7.b IMU Rate
```bash
ros2 topic hz /imu_correct
```

#### 2.7.c Inspect One IMU Message
```bash
ros2 topic echo /imu_correct --once
```

#### 2.7.d Inspect One PointCloud2 Message
```bash
ros2 topic echo /points_raw --once
```
Check that both timestamps and fields are populating correctly.

### 2.8 Verify LVI-SAM Processing

Downstream topic activity confirms that the data is successfully traversing the LVI-SAM algorithms:
```bash
ros2 topic hz /sam/lidar/deskew/cloud_info
ros2 topic hz /sam/lidar/feature/cloud_corner
ros2 topic hz /sam/lidar/feature/cloud_surface
ros2 topic hz /sam/lidar/mapping/odometry
ros2 topic hz /sam/lidar/mapping/map_global
```

Verify exact output formats:
```bash
ros2 topic echo /sam/lidar/mapping/odometry --once
```

### 2.9 RViz2 Visualization During Bag Replay

RViz2 is used to visually confirm the map and trajectory generation. In a new terminal:
```bash
source /opt/ros/humble/setup.bash
source ~/Documents/SIH/Slam/ros2_ws/install/setup.bash
rviz2
```

In the RViz2 GUI, add displays for `PointCloud2`, `Odometry`, `Path`, and `TF`. Subscribe them to the following topics:
```text
/sam/lidar/mapping/map_global
/sam/lidar/mapping/odometry
/sam/lidar/mapping/path
```

### 2.10 Bag Replay Success Criteria

The bag test is not considered successful until the following checks pass:
```text
[ ] bag opens successfully
[ ] LiDAR publisher becomes active
[ ] IMU publisher becomes active
[ ] /points_raw receives messages
[ ] /imu_correct receives messages
[ ] PointCloud2 fields are correct
[ ] LiDAR and IMU timestamps are valid
[ ] cloud_info is published
[ ] corner features are published
[ ] surface features are published
[ ] odometry is published
[ ] map is published
[ ] RViz displays data
[ ] no LVI-SAM node crashes
[ ] timestamps overlap correctly
```

### 2.11 What Has Been Verified and What Is Still Pending

Verified before bag replay:
```text
source code compilation
linking
installation
individual node startup
ROS 2 graph creation
CloudInfo interface availability
```

Pending until the real bag is replayed:
```text
real LiDAR processing
real IMU processing
deskew
feature extraction from real sensor data
odometry
mapping
trajectory
RViz visualization with real data
```

## 3. Edge Device Integration, Input Format, and Output Format

This section outlines the strict integration contract required to connect LVI-SAM to real physical hardware on an edge computing device (e.g., Jetson, NUC, Raspberry Pi).

### 3.1 Important Interface Rule: NumPy Is Not the LVI-SAM API

LVI-SAM does not directly accept NumPy arrays, raw memory buffers, or serial streams. 

NumPy arrays may be used internally inside a custom Python driver or preprocessing node. However, the actual interface into LVI-SAM is exclusively ROS 2 messages published on ROS 2 topics.

```text
Vendor SDK / sensor bytes
        |
        v
Driver node
        |
        +--> NumPy/C++ structs/internal representation
        |
        v
ROS 2 message
        |
        v
LVI-SAM
```

### 3.2 Edge Device Architecture

```text
LiDAR
  |
  v
LiDAR Driver
  |
  v
sensor_msgs/msg/PointCloud2
  |
  +--> /points_raw
  |
  v
LVI-SAM Image Projection
  |
  v
CloudInfo
  |
  v
Feature Extraction
  |
  v
Mapping / Odometry


IMU
  |
  v
IMU Driver
  |
  v
sensor_msgs/msg/Imu
  |
  +--> /imu_correct
  |
  v
IMU Preintegration
  |
  v
Mapping / Optimization
```

### 3.3 Required Input Topics

| Sensor | ROS 2 topic | ROS 2 type | Mandatory data |
| --- | --- | --- | --- |
| LiDAR | `/points_raw` | `sensor_msgs/msg/PointCloud2` | x,y,z,intensity,ring,time |
| IMU | `/imu_correct` | `sensor_msgs/msg/Imu` | timestamp, angular velocity, linear acceleration |

Related interfaces (e.g., `/odometry/gps`, `/odometry/imu`) are available if required, but the core pipeline depends strictly on the LiDAR and IMU feeds.

### 3.4 LiDAR Input Format

The LiDAR input on `/points_raw` must be formatted as `sensor_msgs/msg/PointCloud2`. 

The logical point fields must contain:
```text
x         Cartesian X coordinate
y         Cartesian Y coordinate
z         Cartesian Z coordinate
intensity LiDAR return intensity
ring      laser channel/ring index
time      per-point acquisition time relative to the scan
```

There is a critical distinction in time management:
- `PointCloud2.header.stamp`: The global ROS timestamp for the overall point cloud packet.
- `point.time`: The relative, per-point firing offset used for deskewing motion distortion during a scan.

### 3.5 Exact PointCloud2 Construction Example

The following is a complete Python driver node demonstrating how to package a raw NumPy array into a compliant ROS 2 `PointCloud2` message.

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
import numpy as np

class LidarDriver(Node):
    def __init__(self):
        super().__init__('lidar_hardware_driver')
        self.publisher_ = self.create_publisher(PointCloud2, '/points_raw', 10)
        self.timer = self.create_timer(0.1, self.timer_callback) # 10Hz

    def timer_callback(self):
        # Simulated raw hardware reading
        points = np.array(
            [
                (1.0, 0.0, 0.5, 100.0, 0, 0.0000),
                (1.1, 0.1, 0.5, 110.0, 0, 0.0001),
                (1.2, 0.2, 0.5, 120.0, 1, 0.0002),
            ],
            dtype=[
                ('x', np.float32),
                ('y', np.float32),
                ('z', np.float32),
                ('intensity', np.float32),
                ('ring', np.uint16),
                ('time', np.float32),
            ]
        )

        msg = point_cloud2.create_cloud(
            self.get_clock().now().to_msg(), 
            "lidar_link", 
            points
        )
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    driver = LidarDriver()
    rclpy.spin(driver)
```

### 3.6 IMU Input Format

Topic:
`/imu_correct`

Type:
`sensor_msgs/msg/Imu`

Required fields to populate:
```text
header.stamp
header.frame_id

angular_velocity.x
angular_velocity.y
angular_velocity.z

linear_acceleration.x
linear_acceleration.y
linear_acceleration.z
```

Units:
```text
angular velocity = rad/s
linear acceleration = m/s²
```
When real sensor covariance estimates are available from the hardware SDK, they should be mapped to the `angular_velocity_covariance` and `linear_acceleration_covariance` arrays. Do not fabricate orientation values if the raw IMU does not provide a robust AHRS estimate.

### 3.7 Complete NumPy to IMU Example

The following demonstrates converting internal NumPy representations of IMU data into a valid ROS 2 message.

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np

class IMUDriver(Node):
    def __init__(self):
        super().__init__('hardware_imu_driver')
        self.publisher_ = self.create_publisher(Imu, '/imu_correct', 10)
        self.timer = self.create_timer(0.01, self.timer_callback) # 100Hz

    def timer_callback(self):
        # NumPy array: internal representation [ax, ay, az, gx, gy, gz]
        raw = np.array([0.0, 0.0, 9.81, 0.0, 0.0, 0.0], dtype=np.float64)

        # sensor_msgs/msg/Imu: actual LVI-SAM input
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"
        
        msg.linear_acceleration.x = raw[0]
        msg.linear_acceleration.y = raw[1]
        msg.linear_acceleration.z = raw[2]
        
        msg.angular_velocity.x = raw[3]
        msg.angular_velocity.y = raw[4]
        msg.angular_velocity.z = raw[5]
        
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    driver = IMUDriver()
    rclpy.spin(driver)
```

### 3.8 How LiDAR and IMU Are Fused

The driver node should not attempt to concatenate the LiDAR and IMU into a single array. `LiDAR -> /points_raw` and `IMU -> /imu_correct` are published as separate, synchronized ROS 2 streams.

The LVI-SAM pipeline consumes them independently:
```text
LiDAR timestamp:
    t_L

IMU samples:
    t_I0, t_I1, t_I2, ...
```
IMU data are integrated over the relevant LiDAR time interval. LiDAR points are deskewed using the motion estimate derived from the IMU over the scan period.

### 3.9 Time Synchronization: Mandatory Note

> IMPORTANT: LiDAR and IMU timing must be consistent. The system must know how the LiDAR and IMU timestamps relate to each other.

Clock domains (hardware clock, CPU clock, device boot clock, ROS time, Unix time) must be aligned.
* Timestamp measurements as close to acquisition as possible.
* Do not timestamp stale buffered data with current CPU time.
* Keep timestamps monotonic.
* Avoid mixing incompatible clock domains.
* Measure or establish the LiDAR/IMU clock offset.
* Correct or compensate for drift when required.
* Preserve accurate per-point LiDAR timing.
* During bag playback, check `use_sim_time` behavior.

If time synchronization fails, it can result in deskew error, incorrect temporal association, IMU integration error, trajectory distortion, map distortion, and optimizer instability. Utilize PPS or PTP for strict hardware synchronization whenever possible.

### 3.10 Coordinate Frames and TF

The physical arrangement of sensors must be declared via the TF tree. The expected standard tree is:

```text
map
 |
 v
odom
 |
 v
base_link
```
And:
```text
base_link
 |       |
 v       v
lidar_link
imu_link
```

The exact names (`base_link`, `lidar_link`) must match across `message.frame_id`, TF, parameters, calibration, and the backend.

### 3.11 Actual Extrinsic Calibration

The system uses three parameters for the LiDAR-to-IMU physical offset:
```text
extrinsicRot
extrinsicRPY
extrinsicTrans
```

The identity and zero values currently in the ROS 2 port are safe startup defaults added to prevent the ParamServer segmentation fault. They are not the calibrated physical LiDAR-to-IMU transform. For the actual edge device, replace them with the measured/calibrated extrinsics before running the mapping pipeline.

### 3.12 QoS Compatibility

ROS 2 Quality of Service (QoS) profiles dictate whether data is reliably delivered. 
```bash
ros2 topic info /points_raw -v
ros2 topic info /imu_correct -v
```
Inspect Reliability, History, Depth, and Durability. A mismatch (e.g., driver publishes `Reliable`, LVI-SAM subscribes `BestEffort`) will result in silent connection failures. `ros2 topic list` alone does not prove data delivery; always verify throughput with `ros2 topic hz`.

### 3.13 Output Format

The LVI-SAM engine produces standard ROS 2 outputs for consumption by downstream planning and control modules.

```text
/sam/lidar/mapping/odometry
    nav_msgs/msg/Odometry

/sam/lidar/mapping/map_global
    sensor_msgs/msg/PointCloud2

/sam/lidar/mapping/path
    nav_msgs/msg/Path
```

Other currently observed outputs in the topic graph include:
```text
/sam/lidar/deskew/cloud_deskewed
/sam/lidar/deskew/cloud_info
/sam/lidar/feature/cloud_corner
/sam/lidar/feature/cloud_surface
/sam/lidar/imu/path
/sam/lidar/mapping/cloud_registered
/sam/lidar/mapping/cloud_registered_raw
/sam/lidar/mapping/map_local
/sam/lidar/mapping/trajectory
```

### 3.14 Output Verification Commands

Verify the types of the published streams:
```bash
ros2 topic type /sam/lidar/mapping/odometry
ros2 topic type /sam/lidar/mapping/map_global
ros2 topic type /sam/lidar/mapping/path
```
Inspect the output stream and frequency:
```bash
ros2 topic echo /sam/lidar/mapping/odometry --once
ros2 topic hz /sam/lidar/mapping/odometry
```

### 3.15 Backend Integration Contract

The real backend (e.g., navigation, planning, control software) connects to the SLAM engine strictly via the odometry output.

```text
LVI-SAM
    |
    | /sam/lidar/mapping/odometry
    v
Real backend / navigation / planning / control
```

The backend receives a standard `nav_msgs/msg/Odometry` message. It should extract the timestamp, frame_id, child_frame_id, position, orientation, linear/angular velocities (if available), and covariance estimates (if populated) for control logic.

### 3.16 Complete Backend Subscriber Example

The following demonstrates a Python node subscribing to the LVI-SAM odometry output and mapping it to an internal NumPy array for backend consumption.

```python
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np

class BackendController(Node):
    def __init__(self):
        super().__init__('backend_controller')
        self.subscription = self.create_subscription(
            Odometry,
            '/sam/lidar/mapping/odometry',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        qx = msg.pose.pose.orientation.x
        qy = msg.pose.pose.orientation.y
        qz = msg.pose.pose.orientation.z
        qw = msg.pose.pose.orientation.w
        
        # Internal NumPy representation for control/planning logic
        pose_array = np.array([x, y, z, qx, qy, qz, qw])
        
        # self.process_control(pose_array)

def main(args=None):
    rclpy.init(args=args)
    node = BackendController()
    rclpy.spin(node)
```
The NumPy representation is strictly for the backend's internal use; LVI-SAM's output interface remains ROS 2 `nav_msgs/msg/Odometry`.

### 3.17 Real Edge Device Integration Procedure

Follow this exact sequence to deploy LVI-SAM on physical hardware.

#### 3.17.a Install and Source ROS 2
```bash
source /opt/ros/humble/setup.bash
```

#### 3.17.b Build Workspace
```bash
cd ~/Documents/SIH/Slam/ros2_ws
colcon build
source install/setup.bash
```

#### 3.17.c Start LiDAR Driver
Start the vendor ROS 2 LiDAR driver and configure it to publish PointCloud2.
```bash
ros2 topic list
ros2 topic type /points_raw
ros2 topic hz /points_raw
```

#### 3.17.d Start IMU Driver
Start the IMU hardware driver, ensuring it publishes ROS 2 `sensor_msgs/msg/Imu`.
```bash
ros2 topic type /imu_correct
ros2 topic hz /imu_correct
ros2 topic echo /imu_correct --once
```

#### 3.17.e Verify LiDAR Fields
Inspect the point cloud structure to ensure `ring` and `time` are present:
```bash
ros2 topic echo /points_raw --no-arr --once
```

#### 3.17.f Verify Timestamp Alignment
Check that both streams are emitting valid timestamps close to the current system clock (or a unified reference clock).

#### 3.17.g Verify TF
Check the live frame tree:
```bash
ros2 topic echo /tf --once
ros2 run tf2_tools view_frames
```

#### 3.17.h Configure Real Extrinsics
Update `extrinsicRot`, `extrinsicRPY`, and `extrinsicTrans` in the parameter files or `utility.h` with the true measured transformations between the IMU and LiDAR physical centers.

#### 3.17.i Launch LVI-SAM
Run the four LVI-SAM node components in separate terminals (as detailed in section 2.4).

#### 3.17.j Validate Outputs
```bash
ros2 topic hz /sam/lidar/mapping/odometry
ros2 topic hz /sam/lidar/mapping/map_global
ros2 topic echo /sam/lidar/mapping/odometry --once
```

#### 3.17.k Start the Real Backend
Execute your custom backend node (e.g. the Python node from 3.16) to consume the odometry.

### 3.18 Commonly Missed Integration Items

```text
[ ] LiDAR x field present
[ ] LiDAR y field present
[ ] LiDAR z field present
[ ] intensity field present
[ ] ring field present
[ ] per-point time field present
[ ] correct PointCloud2 datatype
[ ] correct field offsets
[ ] correct point_step
[ ] correct frame_id
[ ] IMU timestamp correct
[ ] IMU frame_id correct
[ ] IMU acceleration in m/s^2
[ ] IMU angular velocity in rad/s
[ ] IMU axis convention verified
[ ] ENU/NED convention verified
[ ] LiDAR coordinate convention verified
[ ] actual LiDAR-to-IMU extrinsics loaded
[ ] startup identity extrinsics replaced
[ ] TF tree correct
[ ] /points_raw active
[ ] /imu_correct active
[ ] QoS compatible
[ ] LiDAR timestamps monotonic
[ ] IMU timestamps monotonic
[ ] clock offset known
[ ] clock drift considered
[ ] point-level LiDAR timing correct
[ ] sensor frequencies correct
[ ] packet loss checked
[ ] topic names correct
[ ] topic remapping correct
[ ] use_sim_time checked for bag replay
[ ] calibration loaded
[ ] sufficient RAM
[ ] sufficient CPU
[ ] storage bandwidth sufficient
[ ] thermal throttling checked
[ ] executor/thread load checked
[ ] duplicate publishers checked
[ ] duplicate ROS node names removed
[ ] backend subscribes to correct odometry topic
[ ] backend validates timestamp/frame_id
[ ] backend validates position/orientation before control
```

### 3.19 Final Edge-to-Backend Data Flow

```text
Physical LiDAR
      |
      v
LiDAR Driver
      |
      v
sensor_msgs/msg/PointCloud2
      |
      +----> /points_raw
      |
      v
LVI-SAM Deskew
      |
      v
Feature Extraction
      |
      v
Mapping / Optimization
      |
      +----> /sam/lidar/mapping/odometry
      |
      +----> /sam/lidar/mapping/map_global
      |
      +----> /sam/lidar/mapping/path
      |
      v
Real Backend


Physical IMU
      |
      v
IMU Driver
      |
      v
sensor_msgs/msg/Imu
      |
      +----> /imu_correct
      |
      v
IMU Preintegration
      |
      v
Mapping / Optimization
```

A successful ROS 2 node startup only proves initialization. 

A successful bag test requires real sensor messages to traverse the complete graph. 

A successful edge-device integration additionally requires correct sensor message construction, timestamp synchronization, TF, calibrated extrinsics, QoS compatibility, stable processing, and verified backend consumption of the LVI-SAM output.
