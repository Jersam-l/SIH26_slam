# 🚁 LVI-SAM Definitive Installation Guide (ROS Noetic)

This guide documents the exact, tested steps to install and compile the **LVI-SAM** (Lidar-Visual-Inertial Smoothing and Mapping) engine from a completely blank hard drive up to a functional 3D map.

Because LVI-SAM was originally developed for older versions of ROS (OpenCV 3 and C++11), compiling it on a modern system usually results in massive error traces. **This guide includes the critical patches required to make it compile flawlessly on modern systems.**

---

## Phase 0: Operating System Setup
**CRITICAL:** ROS Noetic is strictly hardcoded to run on **Ubuntu 20.04 (Focal Fossa)**. If you install a newer version of Ubuntu (like 22.04 or 24.04), the ROS installation will completely fail.

### Scenario A: Windows Users (WSL)
If you are developing on a Windows machine, you must force Windows to install this specific older version of Ubuntu. Open your **Windows PowerShell** and run:
```powershell
wsl --install -d Ubuntu-20.04
```
Once installed, open your new Ubuntu 20.04 terminal and proceed to Phase 1.

### Scenario B: Native Linux (Onboard Drone Computers)
If you are installing this directly onto the drone's companion computer (like an Intel NUC, Raspberry Pi, or NVIDIA Jetson), you must flash the hard drive with a native **Ubuntu 20.04 LTS** image. You can download the ISO directly from the [Ubuntu 20.04 Archive](https://releases.ubuntu.com/focal/). 

---

## Phase 1: Install ROS Noetic (Base System)
If your Ubuntu 20.04 system is brand new, you must install the core ROS Noetic framework first. Run these commands in your terminal to install the complete Desktop Full version:

```bash
# 1. Setup your computer to accept software from packages.ros.org
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# 2. Add the official ROS keys
sudo apt install curl -y
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# 3. Install ROS Noetic (Desktop Full)
sudo apt update
sudo apt install ros-noetic-desktop-full -y

# 4. Install build tools and initialize rosdep
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential -y
sudo rosdep init
rosdep update

# 5. Source the base ROS environment
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

---

## Phase 2: Install Core Mathematical Dependencies
LVI-SAM relies on heavy-duty math libraries to calculate 3D environments: **Ceres Solver** and **GTSAM**.

```bash
# 1. Install Ceres Solver and OpenCV bridges
sudo apt-get install -y libceres-dev ros-noetic-vision-opencv libopencv-dev

# 2. Install GTSAM 4.0 (Georgia Tech Smoothing and Mapping)
sudo add-apt-repository ppa:borglab/gtsam-release-4.0 -y
sudo apt-get update
sudo apt-get install -y libgtsam-dev libgtsam-unstable-dev
```

---

## Phase 3: Create the Structured Workspace
**Why do we create a `Drone_SLAM_Project` folder?** 
By default, Linux mixes hidden system files (like `.bashrc` and `.cache`) into your home directory. We create a dedicated `Drone_SLAM_Project` folder so you have a perfectly structured, clean directory that only contains your robotics code. This allows you to safely push your code to GitHub later without accidentally uploading your computer's private operating system files.

```bash
# 1. Create the master project folder and the ROS source directory
mkdir -p ~/Drone_SLAM_Project/catkin_ws/src

# 2. Navigate into the source directory
cd ~/Drone_SLAM_Project/catkin_ws/src

# 3. Download the LVI-SAM source code directly from GitHub
git clone https://github.com/TixiaoShan/LVI-SAM.git
```

---

## Phase 4: The Critical System Patches (The Secret Sauce)
Before compiling, we **must** modify the LVI-SAM code to support ROS Noetic's modern architecture. If you skip this, the compiler will fail instantly.

### Patch A: OpenCV 3 to OpenCV 4 Translation
ROS Noetic uses OpenCV 4, but LVI-SAM was written using OpenCV 3 vocabulary (e.g., `CV_RGB2GRAY`). We use `sed` (Stream Editor) to search through every single file in LVI-SAM and translate the old OpenCV 3 syntax into the new OpenCV 4 syntax (`cv::COLOR_RGB2GRAY`).

```bash
cd ~/Drone_SLAM_Project/catkin_ws/src/LVI-SAM

# Run this massive search-and-replace command:
find . -type f \( -name "*.cpp" -o -name "*.h" -o -name "*.cc" \) -exec sed -i \
  -e 's/<opencv\/cv.h>/<opencv2\/opencv.hpp>/g' \
  -e 's/CV_RGB2GRAY/cv::COLOR_RGB2GRAY/g' \
  -e 's/CV_BGR2GRAY/cv::COLOR_BGR2GRAY/g' \
  -e 's/CV_GRAY2RGB/cv::COLOR_GRAY2RGB/g' \
  -e 's/CV_GRAY2BGR/cv::COLOR_GRAY2BGR/g' \
  -e 's/CV_FONT_HERSHEY_SIMPLEX/cv::FONT_HERSHEY_SIMPLEX/g' \
  -e 's/CV_LOAD_IMAGE_GRAYSCALE/cv::IMREAD_GRAYSCALE/g' \
  -e 's/CV_LOAD_IMAGE_COLOR/cv::IMREAD_COLOR/g' \
  -e 's/CV_LOAD_IMAGE_UNCHANGED/cv::IMREAD_UNCHANGED/g' \
  -e 's/CV_AA/cv::LINE_AA/g' \
  -e 's/CV_FILLED/cv::FILLED/g' \
  -e 's/CV_WINDOW_AUTOSIZE/cv::WINDOW_AUTOSIZE/g' {} +
```

### Patch B: Upgrade to C++14
ROS Noetic's Point Cloud Library (PCL 1.10) requires the **C++14** standard. LVI-SAM's config files default to C++11, which causes thousands of lines of `pcl::traits` template errors.

```bash
cd ~/Drone_SLAM_Project/catkin_ws/src

# Search all CMakeLists.txt files and upgrade c++11 to c++14
find LVI-SAM -name "CMakeLists.txt" -exec sed -i 's/c++11/c++14/g' {} +
```

---

## Phase 5: Compiling the Engine
Now that the code is modernized, we compile it into machine binary. 
**CRITICAL:** We must use `-j4` (or `-j2`) to limit the compiler to 4 CPU cores. Compiling GTSAM and PCL takes ~2.5GB of RAM per core. If you use all your cores, the system will run out of memory and trigger a `Killed: cc1plus` crash.

```bash
# 1. Go to the root of the workspace
cd ~/Drone_SLAM_Project/catkin_ws

# 2. Compile using exactly 4 cores to prevent RAM exhaustion
catkin_make -j4
```
*(Note: This process takes 5 to 10 minutes depending on your CPU. If it crashes with a memory error, drop the number to `-j2` or `-j1` and try again).*

---

## Phase 6: Quality of Life Improvements (Shortcuts)
Before running the software, we can add a few shortcuts to your Linux profile (`~/.bashrc`) so you don't have to type repetitive setup commands every time you open a terminal.

```bash
# 1. Disable the annoying "ROS 1 End-of-Life" popup window forever
echo 'export DISABLE_ROS1_EOL_WARNINGS=1' >> ~/.bashrc

# 2. Automatically "source" your workspace so ROS always knows where LVI-SAM is
echo 'source ~/Drone_SLAM_Project/catkin_ws/devel/setup.bash' >> ~/.bashrc

# 3. Apply the changes immediately
source ~/.bashrc
```
*(Because of step 2, you now only ever have to type `roslaunch lvi_sam run.launch` when you open a new terminal!)*

---

## Phase 7: Post-Installation Checks & Verification

### Check your folder size
Because we compiled heavy C++ math into binary executables, your workspace will grow significantly. You can check the exact size of your project folder at any time by running:
```bash
du -sh ~/Drone_SLAM_Project/catkin_ws
```

### Run a ROS Bag (Proof of Concept)
```bash
# Terminal 1: Start the SLAM engine (No need to source anymore!)
roslaunch lvi_sam run.launch

# Terminal 2: Inject the dataset (Replace PATH with your actual file path)
rosbag play /mnt/c/Users/YourName/Downloads/garden.bag
```

---

## Phase 8: Data Architecture (Inputs & Outputs)
To successfully integrate this software with your physical drone's flight controller, you must understand the exact ROS data structures it uses. ROS streams data continuously over **Topics** using specific **Message Formats**.

### 1. The Inputs (Hardware ➡️ LVI-SAM)
* **LiDAR Data (`sensor_msgs/PointCloud2`):** A massive binary array of points. Each point contains `x, y, z` (distance in meters), `intensity` (reflectivity), `ring` (laser channel), and `time` (nanosecond firing time).
* **Camera Data (`sensor_msgs/Image`):** A 2D matrix of pixels, usually representing Grayscale brightness (0-255) or RGB color arrays.
* **IMU Data (`sensor_msgs/Imu`):** Physics vectors containing `linear_acceleration` (x, y, z in m/s²) and `angular_velocity` (x, y, z tilt in rad/s).

### 2. The Outputs (LVI-SAM ➡️ Flight Controller)
* **Odometry/Location (`nav_msgs/Odometry`):** The drone's live location. Contains a `Position` object (X, Y, Z in meters) and an `Orientation` quaternion (x, y, z, w) describing the drone's exact 3D tilt. This is what you feed to your flight controller for autonomous navigation.
* **The 3D Map (`sensor_msgs/PointCloud2`):** The globally-aligned, deskewed, and colorized 3D coordinate map of the crash site.

### 3. File Storage (.bag files)
When saving data to a hard drive, ROS packs it into a `.bag` file. This is a highly compressed, serialized binary database. Every piece of sensor data is permanently stamped with a UNIX timestamp so it can be perfectly replayed later.
