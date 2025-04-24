# Azure Kinect DK Usage Guide

Welcome to the **Azure Kinect DK** development kit! This repository includes tools, scripts, and ROS2 integration to help you get started with the Azure Kinect camera on Ubuntu. This guide covers SDK installation, Python examples, and ROS2 node usage.

---

## SDK Installation

For detailed installation steps, refer to [Installation/README.md](Installation/README.md). Below is a summary:

### 1. Install libsoundio1
```bash
cd ~/Azure_Kinect_DK/Installation
sudo dpkg -i libsoundio1_1.1.0-1_amd64.deb
```

### 2. Add Microsoft Package Repository
```bash
sudo apt update
sudo apt install curl software-properties-common apt-transport-https
curl -sSL https://packages.microsoft.com/keys/microsoft.asc | sudo apt-key add -
sudo add-apt-repository "deb [arch=amd64] https://packages.microsoft.com/ubuntu/18.04/prod bionic main"
```

### 3. Install Azure Kinect SDK
```bash
sudo apt update
sudo apt install libk4a1.4 libk4a1.4-dev k4a-tools
```

### 4. Configure Device Permissions
```bash
sudo cp Installation/99-k4a.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

### 5. Test Viewer
```bash
k4aviewer
```

---

## Python Setup

### Install `pyk4a`
```bash
pip install pyk4a
```

### Python Examples

#### 1. RGB Viewer
Path: `Python/rgb_viewer.py`
```bash
python3 Python/rgb_viewer.py
```
Displays the color image stream from the Azure Kinect. Uses the default configuration:
- Color Resolution: 1080P
- Depth Mode: OFF
- Format: BGRA32

#### 2. RGB-D Viewer
Path: `Python/rgbd_viewer.py`
```bash
python3 Python/rgbd_viewer.py
```
Displays both color and depth streams, with depth visualization via OpenCV. Includes:
- Transformed depth map
- Configurable FPS, format, and resolution

---

## ROS2 Node Example

> 🔎 For a more complete and actively maintained ROS2 driver, refer to the community project: [Azure_Kinect_ROS2_Driver](https://github.com/ckennedy2050/Azure_Kinect_ROS2_Driver)

### Environment Setup
Make sure you have sourced your ROS2 environment and set up the workspace correctly:
```bash
cd ~/Azure_Kinect_DK/ros2
colcon build
source install/setup.bash
```

### Run RGB Node
```bash
ros2 run kinect_viewer rgb
```

### Run RGB-D Node
```bash
ros2 run kinect_viewer rgbd
```

---

## Configuration Parameters Table

| Parameter                    | Options                                                                                   | Description                                                                                   |
|-----------------------------|-------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------|
| Color Resolution            | OFF, RES_720P, RES_1080P, RES_1440P, RES_1536P, RES_2160P, RES_3072P                      | Resolution of the color image                                                                |
| Depth Mode                  | OFF, NFOV_2X2BINNED, NFOV_UNBINNED, WFOV_2X2BINNED, WFOV_UNBINNED, PASSIVE_IR             | Depth sensing mode                                                                           |
| FPS                         | FPS_5, FPS_15, FPS_30                                                                      | Camera frame rate                                                                            |
| Color Format                | COLOR_BGRA32, COLOR_MJPG, COLOR_NV12, COLOR_YUY2                                          | Image format of the color stream                                                             |
| Synchronized Images Only    | True, False                                                                               | Enforce synchronized color and depth images                                                  |
| Depth Colorization Range    | (min, max) in mm or (None, None)                                                          | Range used to colorize depth maps                                                            |

---

## References

- Azure Kinect DK official page: [https://azure.microsoft.com/zh-tw/products/kinect-dk#layout-container-uid3944](https://azure.microsoft.com/zh-tw/products/kinect-dk#layout-container-uid3944)
- Azure Kinect Sensor SDK GitHub: [https://github.com/microsoft/Azure-Kinect-Sensor-SDK/tree/develop](https://github.com/microsoft/Azure-Kinect-Sensor-SDK/tree/develop)

