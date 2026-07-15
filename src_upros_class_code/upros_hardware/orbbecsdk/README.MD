# Orbbec ROS SDK
<<<<<<< HEAD

[![stable](http://badges.github.io/stability-badges/dist/stable.svg)](http://github.com/badges/stability-badges) ![version](https://img.shields.io/badge/version-1.5.3-green)

Orbbec ROS SDK is a wrapper for Orbbec SDK that supports ROS Kinetic, Melodic, and Noetic distributions.
It enables smooth integration of Orbbec 3D cameras into ROS projects.

## Table of Contents
<!-- TOC -->
* [Orbbec ROS SDK](#orbbec-ros-sdk)
  * [Table of Contents](#table-of-contents)
  * [Install Dependencies](#install-dependencies)
    * [ROS](#ros)
    * [Other Dependencies](#other-dependencies)
  * [Create ROS Workspace and Build](#create-ros-workspace-and-build)
  * [Start the Camera](#start-the-camera)
  * [Select Topics and Control the Camera](#select-topics-and-control-the-camera)
  * [Launch Parameters](#launch-parameters)
  * [Depth work mode switch:](#depth-work-mode-switch)
  * [Configuration of depth NFOV and WFOV modes](#configuration-of-depth-nfov-and-wfov-modes)
  * [Check which profiles the camera supports](#check-which-profiles-the-camera-supports)
  * [Available Services for Camera Control](#available-services-for-camera-control)
  * [Available Topics](#available-topics)
  * [Building a Debian Package](#building-a-debian-package)
  * [Launch files](#launch-files)
  * [Use Nodelet](#use-nodelet)
  * [Supported hardware products](#supported-hardware-products)
  * [Frequently Asked Questions](#frequently-asked-questions)
    * [No Picture from Multiple Cameras](#no-picture-from-multiple-cameras)
    * [Error: "Failed to start device: usbEnumerator createUsbDevice failed!"](#error-failed-to-start-device-usbenumerator-createusbdevice-failed)
  * [Other useful links](#other-useful-links)
  * [License](#license)
<!-- TOC -->

## Install Dependencies

### ROS

- Please refer directly to the ROS [wiki](http://wiki.ros.org/ROS/Installation) for installation instructions.

### Other Dependencies

- Install dependencies (be careful with your ROS distribution):

  ```bash
  # Assuming you have sourced the ROS environment, same below
  sudo apt install libgflags-dev ros-$ROS_DISTRO-image-geometry ros-$ROS_DISTRO-camera-info-manager \
  ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev \
  ros-$ROS_DISTRO-diagnostic-updater ros-$ROS_DISTRO-diagnostic-msgs
  ```

## Create ROS Workspace and Build

Create a ROS workspace (if you don't have one):
=======
[![stable](http://badges.github.io/stability-badges/dist/stable.svg)](http://github.com/badges/stability-badges) ![version](https://img.shields.io/badge/version-1.3.8-green)

Orbbec ROS SDK is a wrapper for OrbbecSDK that supports ROS Kinetic, Melodic, and Noetic distributions.
It enables smooth integration of Orbbec 3D cameras into ROS projects.

## Install dependencies

### ROS

- Please refer directly to ROS [wiki](http://wiki.ros.org/ROS/Installation).

### other dependencies

- Install dependencies (be careful with your ROS distribution)

  ```bash
  # Assuming you have sourced the ros environment, same below
  sudo apt install libgflags-dev  ros-$ROS_DISTRO-image-geometry ros-$ROS_DISTRO-camera-info-manager\
  ros-$ROS_DISTRO-image-transport ros-$ROS_DISTRO-image-publisher libgoogle-glog-dev libusb-1.0-0-dev libeigen3-dev
  ```

Create ROS workspace(if you don't have one)
>>>>>>> 74a84de24f880691a3252f924f080ec572ad47a7

```bash
mkdir -p ~/ros_ws/src
```

<<<<<<< HEAD
Get the source code:

```bash
cd ~/ros_ws/src
git clone https://github.com/orbbec/OrbbecSDK_ROS1.git
```

Build the package:
=======
Get Source code

```bash
cd ~/ros_ws/src/
git clone https://github.com/orbbec/OrbbecSDK_ROS1.git
```

Or you can download from [Orbbec developer community](https://developer.orbbec.com.cn/download.html)

Build
>>>>>>> 74a84de24f880691a3252f924f080ec572ad47a7

```bash
cd ~/ros_ws
catkin_make
```

<<<<<<< HEAD
Install udev rules:
=======
- Install udev rules.
>>>>>>> 74a84de24f880691a3252f924f080ec572ad47a7

```bash
cd ~/ros_ws
source ./devel/setup.bash
roscd orbbec_camera
<<<<<<< HEAD
sudo bash ./scripts/install_udev_rules.sh
```

## Start the Camera

In terminal 1:

```bash
source ./devel/setup.bash
roslaunch orbbec_camera gemini_330_series.launch
```

In terminal 2:
=======
cd scripts
sudo cp 99-obsensor-libusb.rules /etc/udev/rules.d/99-obsensor-libusb.rules 
sudo udevadm control --reload && sudo  udevadm trigger
```

Start the camera

- In terminal 1

```bash
source ./devel/setup.bash 
roslaunch orbbec_camera astra.launch
```

- In terminal 2
>>>>>>> 74a84de24f880691a3252f924f080ec572ad47a7

```bash
source ./devel/setup.bash
rviz
```

<<<<<<< HEAD
## Select Topics and Control the Camera

Check topics, services, and parameters (open a new terminal):
=======
Select the topic you want to display

- Check topics / services/ parameters (open a new terminal)
>>>>>>> 74a84de24f880691a3252f924f080ec572ad47a7

```bash
rostopic list
rosservice list
rosparam list
```

<<<<<<< HEAD
Get camera parameters (_MUST_ start stream first):
=======
- Get camera parameter, *MUST* start stream first.
>>>>>>> 74a84de24f880691a3252f924f080ec572ad47a7

```bash
rosservice call /camera/get_camera_params "{}"
```

<<<<<<< HEAD
Check camera parameters (please refer to the ROS documentation for the meaning of specific fields in
the [camera info](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html)):
=======
- Check camera parameter, please refer to the ROS documentation for the meaning of the specific fields of the camera
  parameter [camera info](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html)
>>>>>>> 74a84de24f880691a3252f924f080ec572ad47a7

```bash
rostopic echo /camera/depth/camera_info
rostopic echo /camera/color/camera_info
```

<<<<<<< HEAD
Check device information:

```bash
rosservice call /camera/get_device_info "{}"
```

Get the SDK version (includes firmware and Orbbec SDK versions):

```bash
rosservice call /camera/get_sdk_version "{}"
```

Set/get (auto) exposure:

```bash
rosservice call /camera/set_color_auto_exposure '{data: false}'
rosservice call /camera/set_left_ir_auto_exposure "{data: false}"

# Setting exposure values (be careful with the data range; the following example may not be correct)
rosservice call /camera/set_left_ir_exposure "{data: 2000}"
rosservice call /camera/set_color_exposure "{data: 2000}"

# Get exposure
rosservice call /camera/get_left_ir_exposure "{}"
rosservice call /camera/get_color_exposure "{}"
```

Set/get gain:

```bash
# Get gain
rosservice call /camera/get_color_gain '{}'
rosservice call /camera/get_left_ir_gain '{}'

# Setting the gain (be careful with the data range; the following example may not be correct)
rosservice call /camera/set_color_gain "{data: 200}"
rosservice call /camera/set_left_ir_gain "{data: 200}"
```

Set/get (auto) white balance:

```bash
rosservice call /camera/set_auto_white_balance "{data: false}"
rosservice call /camera/get_auto_white_balance "{data: false}"
```

Turn on/off laser:

```bash
rosservice call /camera/set_laser '{data: true}' # Turn on
rosservice call /camera/set_laser '{data: false}' # Turn off
```

Save images:

```bash
rosservice call /camera/save_images "{}"
```

Save point cloud:

```bash
rosservice call /camera/save_point_cloud "{}"
```

NOTE: The images are saved under `~/.ros/image` and are only available when the sensor is on.

## Launch Parameters

The following launch parameters are available:

- `connection_delay`: The delay time in milliseconds for reopening the device. Some devices require
  a longer time to initialize, and reopening the device immediately can cause firmware crashes when hot-plugging.
- `enable_point_cloud`: Enables the point cloud.
- `enable_colored_point_cloud`: Enables the RGB point cloud.
- `color_width`, `color_height`, `color_fps`: The resolution and frame rate of the color stream.
- `left_ir_width`, `left_ir_height`, `left_ir_fps`: The resolution and frame rate of the left IR stream.
- `right_ir_width`, `right_ir_height`, `right_ir_fps`: The resolution and frame rate of the right IR stream.
- `depth_width`, `depth_height`, `depth_fps`: The resolution and frame rate of the depth stream.
- `enable_color`: Enables the RGB camera.
- `enable_depth`: Enables the depth camera.
- `enable_left_ir`: Enables the left IR camera.
- `enable_right_ir`: Enables the right IR camera.
- `depth_registration`: Enables hardware alignment of the depth frame to the color frame. This field is required
  when `enable_colored_point_cloud` is set to `true`.
- `log_level` for OrbbecSDK controls console log verbosity, with levels `none`, `info`, `debug`, `warn`, `fatal`. Logs
  save in `~/.ros/Log`. For file logging, adjust `<FileLogLevel>` in `config/OrbbecSDKConfig_v1.0.xml`.
- `ordered_pc`: Whether the point cloud should be organized in an ordered grid (`true`) or as an unordered set of
  points (`false`).
- `device_preset`: The default value is `Default`. Only the G330 series is supported. For more information, refer to
  the [G330 documentation](https://www.orbbec.com/docs/g330-use-depth-presets/). Please refer to the table below to set
  the `device_preset` value based on your use case. The value should be one of the preset names
  listed [in the table](#predefined-presets).
- `enable_decimation_filter`: Only the G330 series is supported. This filter effectively reduces the depth scene complexity. The filter runs on kernel
  sizes [2x2] to [8x8] pixels. The image size is scaled down proportionally in both dimensions to preserve the aspect
  ratio.
- `enable_hdr_merge`: This filter is used jointly with depth HDR function. By merging consecutive depth images of
  alternating exposure values, we can over come challenges in acquiring depth values for under-illuminated and
  over-illuminated objects simultaneously.
- `sequence_id_filter_id`: This filter is used jointly with depth HDR function and outputs only the sequence with
  specified sequence ID.
- `enable_threshold_filter`: This filter preserves depth values of interest and omits depth values out of scope.
- `enable_noise_removal_filter`: This filter removes speckle noise in clusters and gives rise to a less-filled depth
  map.
- `enable_spatial_filter`: This filter performs multiple iterations of processing as specified by the magnitude
  parameter, to enhance the smoothness of depth data. It is also capable of filling small holes in depth maps.
- `enable_temporal_filter`:This filter is intended to improve the depth data persistency by manipulating per-pixel
  values based on previous frames. The filter performs a single pass on the data, adjusting the depth values while also
  updating the tracking history.
- `enable_hole_filling_filter`: This filter fills all holes in the depth map using the specified mode.
- `depth_precision` : The depth precision, should be like `1mm` format. The default value is `1mm`.
- `enable_laser` : Whether to enable the laser. The default value is `true`.
- `laser_on_off_mode`: Laser on/off alternate mode, 0: off, 1: on-off alternate, 2: off-on alternate. The default value
  is `0`.
- `retry_on_usb3_detection_failure`: If the camera is connected to a USB 2.0 port and is not detected, the system will
  attempt to reset the camera up to three times. This setting aims to prevent USB 3.0 devices from being incorrectly
  recognized as USB 2.0.
  It is recommended to set this parameter to `false` when using a USB 2.0 connection to avoid unnecessary resets.

**IMPORTANT**: *Please carefully read the instructions regarding software filtering settings at [this link](https://www.orbbec.com/docs/g330-use-depth-post-processing-blocks/). If you are uncertain, do not modify these settings.*
## Depth work mode switch:

- Before starting the camera, depth work mode (depth_work_mode) can be configured for the corresponding xxx.launch
  file's support.
- The depth work mode switch is supported by Gemini 2, Gemini 2 L, and Gemini 2 XL cameras.
- The default depth work mode configuration of xxx.launch is the camera's default configuration. If you need to modify
  it, you can switch to the corresponding mode as needed.
- The specific camera depth work mode support types can be found in the comments of the depth mode.

```XML
<!-- Depth work mode support is as follows： -->
<!-- Unbinned Dense Default -->
<!-- Unbinned Sparse Default -->
<!-- Binned Sparse Default -->
<!-- Obstacle Avoidance -->
<arg name="depth_work_mode" default=""/>
```

- View depth work modes:

```bash
rosrun orbbec_camera list_depth_work_mode_node
```

## Configuration of depth NFOV and WFOV modes

For the **Femto Mega** and **Femto Bolt** devices, the NFOV and WFOV modes are implemented by configuring the resolution of
Depth and IR in the launch file.
In launch file, depth_width、depth_height、ir_width、ir_height represents the resolution of the depth and the resolution of
the IR.
The frame fps and resolution of IR must be consistent with the depth. The correspondence between different modes and
resolutions is as follows:

- NFOV unbinned: 640 x 576.
- NFOV binned: 320 x 288.
- WFOV unbinned: 1024 x 1024.
- WFOV binned: 512 x 512.

## Check which profiles the camera supports

```bash
rosrun orbbec_camera list_camera_profile_mode_node
```

## Available Services for Camera Control

The service names intuitively reflect their purposes. It's crucial to understand that services related to setting or
getting parameters—denoted as `set_*` and `get_*`—become available only when the respective `enable_*` parameters are
activated. For instance, enabling features such as left infrared (IR) with `enable_left_ir`, right IR
with `enable_right_ir`, depth sensing with `enable_depth`, or color processing with `enable_color` (refer
to [Launch Parameters](#Launch-Parameters)) is a prerequisite for their corresponding services to be operational. This
configuration ensures that services are accessible only when their specific stream is enabled in the
launch file's stream argument.
=======
- Check device information

```bash
rosservice call /camera/get_device_info "{}" 
```

- Get the SDK version (Include firmware and Orbbec SDK version )

```bash
rosservice call /camera/get_version "{}"
```

- Set/get (auto) exposure

```bash
rosservice call /camera/set_color_auto_exposure '{data: false}' 
rosservice call /camera/set_ir_auto_exposure  "{data: false}"
    
# Setting exposure values (Be careful with the data range, the following example may not be correct.)
rosservice call /camera/set_ir_exposure  "{data: 2000}"
roservice call /camera/set_color_exposure  "{data: 2000}"
 # Get exposure
 rosservice call /camera/get_ir_exposure  "{}"
 rosservice call /camera/get_color_exposure "{}"
```

- Set/get gain

```bash
# Get Gain
rosservice call /camera/get_color_gain '{}'
rosservice call /camera/get_ir_gain '{}' 
# Setting the gain (Be careful with the data range, the following example may not be correct.)
rosservice call /camera/set_color_gain  "{data: 200}"
rosservice call /camera/set_ir_gain "{data: 200}"
```

- Set/get mirror

```bash
rosservice call /camera/set_color_mirror  "{data: true}"
rosservice call /camera/set_depth_mirror  "{data: true}"
rosservice call /camera/set_ir_mirror  "{data: true}"
```

- Set/get (auto) white balance

```bash
rosservice call /camera/set_auto_white_balance  "{data: false}"
rosservice call /camera/get_auto_white_balance  "{data: false}"
```

- Turn on/off laser

```bash
rosservice call /camera/set_laser '{data: true}' # Turn on
rosservice call /camera/set_laser '{data: false}' # Turn off
```

- Turn on/off fans

```bash
 rosservice call /camera/set_fan  '{data: true}' # Turn on
 rosservice call /camera/set_fan  '{data: false}' # Turn off
```

- Turn on/off LDP

```bash
rosservice  call /camera/set_ldp '{data: true}'  # Turn on
rosservice  call /camera/set_ldp '{data: false}' # Turn off

```

- Turn on/off sensors

```bash
rosservice call  /camera/toggle_ir "{data: true}"
rosservice call  /camera/toggle_color "{data: true}"
rosservice call  /camera/toggle_depth "{data: true}"
```

- Save image

```bash
rosservice call /camera/save_images "{}"
```

- save point cloud

```bash
rosservice call /camera/save_point_cloud "{}"
```

NOTE: The images are saved under ~/.ros/image and are only available when the sensor is on.

### All available service for camera control

The name of the following service already expresses its function.
However, it should be noted that the corresponding `set_[ir|depth|color]*`
and `get[ir|depth|color]*` **services are only available if you set** `enable[ir|depth|color]`
to `true` in the stream that corresponds to the argument of the launch file.
>>>>>>> 74a84de24f880691a3252f924f080ec572ad47a7

- `/camera/get_auto_white_balance`
- `/camera/get_camera_params`
- `/camera/get_color_auto_exposure`
- `/camera/get_color_camera_info`
- `/camera/get_color_exposure`
- `/camera/get_color_gain`
- `/camera/get_depth_auto_exposure`
- `/camera/get_depth_camera_info`
- `/camera/get_depth_exposure`
- `/camera/get_depth_gain`
- `/camera/get_device_info`
- `/camera/get_device_type`
<<<<<<< HEAD
- `/camera/get_left_ir_auto_exposure`
- `/camera/get_left_ir_camera_info`
- `/camera/get_left_ir_exposure`
- `/camera/get_left_ir_gain`
=======
- `/camera/get_ir_auto_exposure`
- `/camera/get_ir_camera_info`
- `/camera/get_ir_exposure`
- `/camera/get_ir_gain`
>>>>>>> 74a84de24f880691a3252f924f080ec572ad47a7
- `/camera/get_serial`
- `/camera/get_sdk_version`
- `/camera/get_white_balance`
- `/camera/reset_color_exposure`
- `/camera/reset_color_gain`
- `/camera/reset_depth_exposure`
- `/camera/reset_depth_gain`
<<<<<<< HEAD
- `/camera/reset_left_ir_exposure`
- `/camera/reset_left_ir_gain`
=======
- `/camera/reset_ir_exposure`
- `/camera/reset_ir_gain`
>>>>>>> 74a84de24f880691a3252f924f080ec572ad47a7
- `/camera/reset_white_balance`
- `/camera/save_images`
- `/camera/save_point_cloud`
- `/camera/set_auto_white_balance`
- `/camera/set_color_auto_exposure`
- `/camera/set_color_exposure`
- `/camera/set_color_gain`
<<<<<<< HEAD
- `/camera/set_depth_auto_exposure`
- `/camera/set_depth_exposure`
- `/camera/set_depth_gain`
- `/camera/set_flood`
- `/camera/set_left_ir_auto_exposure`
- `/camera/set_left_ir_exposure`
- `/camera/set_left_ir_gain`
- `/camera/set_laser`
- `/camera/set_white_balance`

## Available Topics

- `/camera/color/camera_info`: The color camera info.
- `/camera/color/image_raw`: The color stream image.
- `/camera/depth/camera_info`: The depth camera info.
- `/camera/depth/image_raw`: The depth stream image.
- `/camera/depth/points`: The point cloud, only available when `enable_point_cloud` is `true`.
- `/camera/depth_registered/points`: The colored point cloud, only available when `enable_colored_point_cloud`
  is `true`.
- `/camera/left_ir/camera_info`: The left IR camera info.
- `/camera/left_ir/image_raw`: The left IR stream image.
- `/camera/right_ir/camera_info`: The right IR camera info.
- `/camera/right_ir/image_raw`: The right IR stream image.
- `/diagnostics`: The diagnostic information of the camera, Currently, the diagnostic information only includes the temperature of the camera.

## Building a Debian Package

First, ensure the necessary tools are installed:

```bash
sudo apt install debhelper fakeroot python3-bloom
```

To create the Debian package, execute these commands:

```bash
cd ~/ros_ws/src/OrbbecSDK_ROS1
bash .make_deb.sh
```

## Launch files

| product serials                     | launch file              |
|-------------------------------------|--------------------------|
| astra+                              | astra_adv.launch         |
| astra mini/astra mini pro/astra pro | astra.launch             |
| astra mini s pro                    | astra.launch             |
| astra2                              | astra2.launch            |
| astra stereo s                      | stereo_s_u3.launch       |
| astra pro2                          | astra_pro2.launch        |
| dabai                               | dabai.launch             |
| dabai d1                            | dabai_d1.launch          |
| dabai dcw                           | dabai_dcw.launch         |
| dabai dw                            | dabai_dw.launch          |
| dabai pro                           | dabai_pro.launch         |
| deeya                               | deeya.launch             |
| femto / femto w                     | femto.launch             |
| femto mega                          | femto_mega.launch        |
| femto bolt                          | femto_bolt.launch        |
| gemini                              | gemini.launch            |
| gemini2 / dabai DCL                 | gemini2.launch           |
| gemini2L                            | gemini2L.launch          |
| gemini e                            | gemini_e.launch          |
| gemini e lite                       | gemini_e_lite.launch     |
| dabai max                           | dabai_max.launch         |
| dabai max pro                       | dabai_max_pro.launch     |
| gemini uw                           | gemini_uw.launch         |
| dabai dcw2                          | dabai_dcw2.launch        |
| dabai dw2                           | dabai_dw2.launch         |
| gemini ew                           | gemini_ew.launch         |
| gemini ew lite                      | gemini_ew_lite.launch    |
| gemini 330 series                   | gemini_330_series.launch |

**All launch files are essentially similar, with the primary difference being the default values of the parameters set for different models
within the same series. Differences in USB standards, such as USB 2.0 versus USB 3.0, may require adjustments to these parameters. If you
encounter a startup failure, please carefully review the specification manual. Pay special attention to the resolution settings in the launch
file, as well as other parameters, to ensure compatibility and optimal performance.**
=======
- `/camera/set_color_mirror`
- `/camera/set_depth_auto_exposure`
- `/camera/set_depth_exposure`
- `/camera/set_depth_gain`
- `/camera/set_depth_mirror`
- `/camera/set_fan_work_mode`
- `/camera/set_floor`
- `/camera/set_ir_auto_exposure`
- `/camera/set_ir_exposure`
- `/camera/set_ir_gain`
- `/camera/set_ir_mirror`
- `/camera/set_laser`
- `/camera/set_ldp`
- `/camera/set_white_balance`
- `/camera/toggle_color`
- `/camera/toggle_depth`
- `/camera/toggle_ir`

### All available topics

- `/camera/color/camera_info` : The color camera info.
- `/camera/color/image_raw`: The color stream image.
- `/camera/depth/camera_info`: The depth stream image.
- `/camera/depth/image_raw`: The depth stream image
- `/camera/depth/points` : The point cloud, only available when  `enable_point_cloud` is `true`.
- `/camera/depth_registered/points`: The colored point cloud, only available when  `enable_colored_point_cloud`
  is `true`.
- `/camera/ir/camera_info`:  The IR camera info.
- `/camera/ir/image_raw`: The IR stream image

### Multiple cameras

- First, you need to enumerate the `usb_port` of the cameras. Plug them in and run the following command in the
  terminal, please make sure no camera is open before running:

  ```bash
  rosrun orbbec_camera list_devices_node 
  ```

or you can use the following script to get the serial number and usb port number, save the following script
as `list_ob_devices.sh`, `chmod +x list_ob_devices.sh`, and then run `./list_ob_devices.sh`.

```bash
VID="2bc5"

for dev in /sys/bus/usb/devices/*; do
  if [ -e "$dev/idVendor" ]; then
    vid=$(cat "$dev/idVendor")
    if [ "$vid" == "${VID}" ]; then
      port=$(basename $dev)
      product=$(cat "$dev/product" 2>/dev/null) # product name
      serial=$(cat "$dev/serial" 2>/dev/null) # serial number
      echo "Found Orbbec device $product, usb port $port, serial number $serial"
    fi
  fi
done
```

- Set the `device_num` parameter to the number of cameras you have.
- Go to the `orbbec-ros-sdk/launch/multi_xxx.launch` file and modify the `usb_port`.

```XML

<launch>
    <!-- unique camera name-->
    <arg name="camera_name" default="camera"/>
    <!-- Hardware depth registration -->
    <arg name="3d_sensor" default="astra"/>
    <!-- stereo_s_u3, astrapro, astra -->
    <arg name="camera1_prefix" default="01"/>
    <arg name="camera2_prefix" default="02"/>
    <!-- TODO: change to your usb port -->
    <arg name="camera1_usb_port" default="1-4.2.1"/>

    <arg name="camera2_usb_port" default="1-4.2.2"/>
    -->Change serial number here -->
    <arg name="device_num" default="2"/>
    <include file="$(find astra_camera)/launch/$(arg 3d_sensor).launch">
        <arg name="camera_name" value="$(arg camera_name)_$(arg camera1_prefix)"/>
        <arg name="usb_port" value="$(arg camera1_usb_port)"/>
        <arg name="device_num" default="$(arg device_num)"/>

    </include>

    <include file="$(find astra_camera)/launch/$(arg 3d_sensor).launch">
        <arg name="camera_name" value="$(arg camera_name)_$(arg camera2_prefix)"/>
        <arg name="usb_port" value="$(arg camera2_usb_port)"/>
        <arg name="device_num" default="$(arg device_num)"/>
    </include>
    <node pkg="tf2_ros" type="static_transform_publisher" name="camera_tf"
          args="0 0 0 0 0 0 camera01_link camera02_link"/>
</launch>

 ```

- Launch

``` bash
roslaunch orbbec_camera multi_camera.launch
```

## Use hardware decoder to decode JPEG

### rockchip and Amlogic

Depends on `rockchip-mpp-dev` and `rockchip-rga-dev`, not all systems have these two packages, the names may be
different, please search by yourself.
Open `CMakeLists.txt` and set `USE_RK_HW_DECODER` to `ON`.

### Nvidia Jetson

Depends on: `jetson_multimedia_api`,`libyuv`.
Open `CMakeLists.txt` and set `USE_NV_HW_DECODER` to `ON`.

## Launch parameters

The following are the launch parameters available:

- `connection_delay`: The delay time in milliseconds for reopening the device.
  Some devices, such as Astra mini, require a longer time to initialize and
  reopening the device immediately can cause firmware crashes when hot plugging.
- `enable_point_cloud`: Enables the point cloud.
- `enable_colored_point_cloud`: Enables the RGB point cloud.
- `enable_d2c_viewer`: Publishes the D2C overlay image (for testing only).
- `device_num`: The number of devices. This must be filled in if multiple cameras are required.
- `color_width`, `color_height`, `color_fps`: The resolution and frame rate of the color stream.
- `ir_width`, `ir_height`, `ir_fps`: The resolution and frame rate of the IR stream.
- `depth_width`, `depth_height`, `depth_fps`: The resolution and frame rate of the depth stream.
- `enable_color`: Enables the RGB camera.
- `enable_depth`: Enables the depth camera.
- `enable_ir`: Enables the IR camera.
- `depth_registration`: Enables hardware alignment the depth frame to color frame. This field is required when
  the `enable_colored_point_cloud` is set to `true`.
- `enable_publish_extrinsic`: Enables the publishing of camera extrinsic information.
- `log_level`: The log level for OrbbecSDK, with optional values of `none`, `info`, `debug`, `warn`, and `fatal`.
  The log file can be found in the ROS runtime directory, and the default location is `~/.ros/Log`.
- `enable_accel` : Enables the accelerometer.
- `accel_rate`: The frequency of the accelerometer, the optional values
  are `1.5625hz`,`3.125hz`,`6.25hz`,`12.5hz`,`25hz`,`50hz`,
  `100hz`,`200hz`,`500hz`,`1khz`,`2khz`,`4khz`,`8khz`,`16khz`,`32khz`. The specific value depends on the current camera.
- `accel_range` : The range of the accelerometer, the optional values are `2g`,`4g`,`8g`,`16g`. The specific value
  depends on the current camera.
- `enable_gyro`: Whether to enable the gyroscope.
- `gyro_rate` : The frequency of the gyroscope, the optional values
  are `1.5625hz`,`3.125hz`,`6.25hz`,`12.5hz`,`25hz`,`50hz`,
  `100hz`,`200hz`,`500hz`,`1khz`,`2khz`,`4khz`,`8khz`,`16khz`,`32khz`. The specific value depends on the current camera.
- `gyro_range` : The range of the gyroscope, the optional values
  are `16dps`,`31dps`,`62dps`,`125dps`,`250dps`,`500dps`,`1000dps`,`2000dps`. The specific value depends on the current
  camera.
- `enumerate_net_device` : Whether to enable the function of enumerating network devices. True means enabled, false means disabled.
  This feature is only supported by Femto Mega and Gemini 2 XL devices. When accessing these devices through the network, the IP address of the device needs to be configured in advance. The enable switch needs to be set to true.

## Depth work mode switch:
- Before starting the camera, depth work mode (depth_work_mode) can be configured for the corresponding xxx.launch file's support.
- The depth work mode switch is supported by Gemini 2, Gemini 2 L, and Gemini 2 XL cameras.
- The default depth work mode configuration of xxx.launch is the camera's default configuration. If you need to modify it, you can switch to the corresponding mode as needed.
- The specific camera depth work mode support types can be found in the comments of the depth mode.

```XML
  <!-- Depth work mode support is as follows： -->
  <!-- Unbinned Dense Default -->
  <!-- Unbinned Sparse Default -->
  <!-- Binned Sparse Default -->
  <arg name="depth_work_mode" default=""/>
```

- View depth work modes:

```bash
rosrun orbbec_camera list_depth_work_mode_node
```
## Configuration of depth NFOV and WFOV modes
For the Femto Mega and Femto Bolt devices, the NFOV and WFOV modes are implemented by configuring the resolution of Depth and IR in the launch file. 
In launch file, depth_width、depth_height、ir_width、ir_height represents the resolution of the depth  and the resolution of the IR.
The frame fps and resolution of IR must be consistent with the depth. The correspondence between different modes and resolutions is as follows:
- NFOV unbinned: 640 x 576.
- NFOV binned: 320 x 288.
- WFOV unbinned: 1024 x 1024.
- WFOV binned: 512 x 512.

## Check which profiles the camera supports

  ```bash
  rosrun orbbec_camera list_camera_profile_mode_node
  ```

## Launch files

| product serials                           | launch file          |
|-------------------------------------------|----------------------|
| astra+                                    | astra_adv.launch     |
| astra/astra mini/astra mini pro/astra pro | astra.launch         |
| astra mini s pro                          | astra.launch         |
| astra2                                    | astra2.launch        |
| astra stereo s                            | stereo_s_u3.launch   |
| dabai                                     | dabai.launch         |
| dabai d1                                  | dabai_d1.launch      |
| dabai dcw                                 | dabai_dcw.launch     |
| dabai dw                                  | dabai_dw.launch      |
| dabai pro                                 | dabai_pro.launch     |
| deeya                                     | deeya.launch         |
| femto / femto w                           | femto.launch         |
| femto mega                                | femto_mega.launch    |
| femto bolt                                | femto_bolt.launch    |
| gemini                                    | gemini.launch        |
| gemini                                    | gemini.launch        |
| gemini2 / dabai DCL                       | gemini2.launch       |
| gemini2L                                  | gemini2L.launch      |
| gemini e                                  | gemini_e.launch      |
| gemini e lite                             | gemini_e_lite.launch |

Actually, All launch files all most the same, the only difference is the default value of the parameters.
>>>>>>> 74a84de24f880691a3252f924f080ec572ad47a7

## Use Nodelet

For users who need to use nodelet, please refer to `gemini2_nodelet.launch`

## Supported hardware products
<<<<<<< HEAD
Please refer to the OrbbecSDK supported products: [Product Support](https://github.com/orbbec/OrbbecSDK?tab=readme-ov-file#product-support)

## Frequently Asked Questions

### No Picture from Multiple Cameras

- It's possible that the power supply is insufficient. To avoid this, do not connect all cameras to the same hub and use
  a powered hub instead.

- It's also possible that the resolution is too high. To resolve this, try lowering the resolution.

### Error: "Failed to start device: usbEnumerator createUsbDevice failed!"

- The current device does not have permission to access the device. Check the PID of the current device:

  ```bash
  lsusb | grep 2bc5
  ```

  Your output should look like this:

  ```text
  Bus 002 Device 007: ID 2bc5:your_pid_here
  ```

  Edit `/etc/udev/rules.d/99-obsensor-libusb.rules` and add the following line:

  ```bash
  SUBSYSTEM=="usb", ATTR{idProduct}=="your_pid_here", ATTR{idVendor}=="2bc5", MODE:="0666", OWNER:="root", GROUP:="video", SYMLINK+="your_device_name_here"
  ```

  Replace `your_pid_here` with the PID of your device and `your_device_name_here` with the name you want to create for
  the device (e.g., `gemini_camera`).

  Then restart the udev service:

  ```bash
  sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
  ```

## Other useful links

- [Orbbec SDK](https://orbbec.github.io/OrbbecSDK/)
- [Gemini 330 depth presets](https://www.orbbec.com/docs/g330-use-depth-presets/)

## License

Copyright 2024 Orbbec Ltd.
=======

| **SDK version** | **products list** | **firmware version**                      |
|-----------------|-------------------|-------------------------------------------|
| v1.8.1          | Gemini 2 XL       | Obox: V1.2.5  VL:1.4.54                   |
|                 | Astra 2           | 2.8.20                                    |
|                 | Gemini 2 L        | 1.4.32                                    |
|                 | Gemini 2          | 1.4.60 /1.4.76                            |
|                 | Femto Mega        | 1.1.7  (window10、ubuntu20.04、ubuntu22.04) |
|                 | Astra+            | 1.0.22/1.0.21/1.0.20/1.0.19               |
|                 | Femto             | 1.6.7                                     |
|                 | Femto W           | 1.1.8                                     |
|                 | Femto Bolt        | 1.0.6 (unsupported ARNM32)                |
|                 | DaBai             | 2436                                      |
|                 | DaBai DCW         | 2460                                      |
|                 | DaBai DW          | 2606                                      |
|                 | Astra Mini Pro    | 1007                                      |
|                 | Gemini E          | 3460                                      |
|                 | Gemini E Lite     | 3606                                      |
|                 | Gemini            | 3.0.18                                    |
|                 | Astra Mini S Pro  | 1.0.05                                    |

## Frequently Asked Questions

No Picture from Multiple Cameras

- it's possible that the power supply is insufficient.
  To avoid this, do not connect all cameras to the same hub and use a powered hub instead.

- It's also possible that the resolution is too high.
  To resolve this, try lowering the resolution.

Why are there so many launch files here

- The reason for the presence of multiple launch
  files is due to the fact that the default resolutions and image formats of different cameras vary.
  To make it easier to use, the launch files have been separated for each camera.

Why IR stream No data output

- If you are using Astra/Astra mini / astra mini S camera, you cannot turn on IR and RGB at the same time, please
  change `enable_color` to false in the corresponding launch file to turn off color camera.

Why do I get the following error when running the launch file

```text
Failed to start device: usbEnumerator createUsbDevice failed!
```

- The current device does not have permission to access the device， check the PID of the current device

```bash
lsusb | grep 2bc5
```

Your output should look like this

```text
Bus 002 Device 007: ID 2bc5:your_pid_here
```

Edit`/etc/udev/rules.d/99-obsensor-libusb.rules`，add the following line

```bash
SUBSYSTEM=="usb", ATTR{idProduct}=="your_pid_here", ATTR{idVendor}=="2bc5", MODE:="0666", OWNER:="root"  GROUP:="video", SYMLINK+="you_device_name_here"
```

`you_device_name_here` is the name of the device you want to create, for example `Astra`.

Then restart the udev service

```bash
sudo udevadm control --reload-rules && sudo service udev restart && sudo udevadm trigger
```

## License

Copyright 2023 Orbbec Ltd.
>>>>>>> 74a84de24f880691a3252f924f080ec572ad47a7

Licensed under the Apache License, Version 2.0 (the "License"); you may not use this project except in compliance with
the License. You may obtain a copy of the License at

[http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0)

Unless required by applicable law or agreed to in writing, software distributed under the License is distributed on an "
AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the License for the specific
language governing permissions and limitations under the License.

<<<<<<< HEAD
Other names and brands may be claimed as the property of others.
=======
*Other names and brands may be claimed as the property of others*
>>>>>>> 74a84de24f880691a3252f924f080ec572ad47a7
