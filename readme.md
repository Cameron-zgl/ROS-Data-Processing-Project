# README for ROS Data Processing Project

## Project Description
This project comprises two Python scripts, `ros_back.py` and `rosbag_dismemberment.py`, designed for processing and handling data in ROS (Robot Operating System) environments. These scripts were developed as part of an effort to replicate and experiment with custom ROS bags for the [Kimera-Semantics](https://github.com/MIT-SPARK/Kimera-Semantics) project. They facilitate the manipulation of ROS bag files, conversion of data formats, and handling of sensor data including images and IMU (Inertial Measurement Unit) readings. This tool aims to assist developers who need to disassemble and reassemble ROS bag files for their development needs.

## Prerequisites
- Python 3
- ROS (Robot Operating System)
- OpenCV (for image processing)
- numpy (for numerical operations)
- cv_bridge (for converting between ROS and OpenCV image formats)

## Installation
Ensure ROS is properly installed on your system. Refer to the [ROS installation guide](http://wiki.ros.org/ROS/Installation).
Install Python dependencies:
```bash
pip install opencv-python numpy
```
## Usage
### rosbag_dismemberment.py
- To process and dismember ROS bag files, extracting data such as camera images,TF message,depth pictures and IMU data.
- Usage:
```bash
python rosbag_dismemberment.py [options]
#Replace [arguments] with specific commands or file paths as required for your application.
#sample: python rosbag_dismemberment.py demo.bag
```
### ros_back.py
- Purpose: To handle ROS messages, convert Python YAML to standard YAML, and deal with images and sensor messages in ROS.
- Usage:
```bash
python ros_back.py [arguments]
#Replace [arguments] with specific commands or file paths as required for your application.
#sample: python rosbag_back.py ./data back.bag
```

## Additional Notes
These scripts are part of a larger project focused on ROS data processing and analysis. They should be used in the context of a ROS environment. Modify the scripts as needed to suit your specific project requirements.

## Acknowledgments
- This project utilizes ROS, an open-source robotics middleware, and other open-source libraries such as OpenCV and numpy.
- The rosbag_dismemberment.py reference to the [euroc_dataset_tools](https://github.com/ToniRV/euroc_dataset_tools).
