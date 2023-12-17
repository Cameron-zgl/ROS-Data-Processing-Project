#!/usr/bin/env python3

import os
import cv2
import rospy
import rosbag
import yaml
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, Imu, CameraInfo
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
import numpy as np
def convert_python_yaml_to_standard(yaml_file_path):
    with open(yaml_file_path, 'r') as file:
        file_content = file.read()
        # Remove Python-specific tuple tags
        file_content = file_content.replace("!!python/tuple", "")
        data = yaml.safe_load(file_content)

    return data


def create_rosbag_from_files(base_path, output_rosbag_path):
    bridge = CvBridge()
    with rosbag.Bag(output_rosbag_path, 'w') as bag:
        # Process Camera Data
        cam_mappings = {
            'cam0': '/tesse/depth/image_raw',
            'cam2': '/tesse/left_cam/image_raw',
            'cam1': '/tesse/right_cam/image_raw',
            'cam3': '/tesse/segmentation/image_raw'
        }
        for cam_id, ros_topic in cam_mappings.items():
            data_path = os.path.join(base_path, cam_id, 'data')
            data_csv_path = os.path.join(base_path, cam_id, 'data.csv')

            with open(data_csv_path, 'r') as file:
                for line in file.readlines()[1:]:  # Skip header
                    timestamp, filename = line.strip().split(',')
                    image_path = os.path.join(data_path, filename)

                    # Check if the file is a .npy file
                    if filename.endswith('.npy'):
                        # Load the image as a NumPy array
                        cv_image = np.load(image_path)
                        encoding = '32FC1'  # Assuming the .npy file contains 32-bit float depth data
                    else:
                        # For other image formats, read using OpenCV
                        cv_image = cv2.imread(image_path, cv2.IMREAD_UNCHANGED)
                        # Determine encoding based on image type
                        if cv_image.dtype == np.float32:
                            encoding = '32FC1'
                        elif len(cv_image.shape) == 2:
                            encoding = 'mono8'  # Grayscale image
                        elif cv_image.shape[2] == 3:
    			    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                            encoding = 'rgb8'  # Color image
                        else:
                            print("Unsupported image format for image: {}".format(filename))
                            continue

                    try:
                        ros_image = bridge.cv2_to_imgmsg(cv_image, encoding)

                        # Convert timestamp to rospy.Time
                        timestamp_ns = int(timestamp)
                        timestamp_sec = timestamp_ns // 1000000000
                        timestamp_nsec = timestamp_ns % 1000000000
                        ros_image.header.stamp = rospy.Time(secs=timestamp_sec, nsecs=timestamp_nsec)

                        bag.write(ros_topic, ros_image, ros_image.header.stamp)
                    except CvBridgeError as e:
                        print(e)

        # Process TF Data
        tf_mappings = {
            'tf0': '/tf',
            'tf1': '/tf_static'
        }

        for tf_id, ros_topic in tf_mappings.items():
            data_csv_path = os.path.join(base_path, tf_id, 'data.csv')

            with open(data_csv_path, 'r') as file:
                lines = file.readlines()
                for line in lines[1:]:  # Skip header
                    data = line.strip().split(',')
                    timestamp = data[0]

                    # Create a TransformStamped message
                    t = TransformStamped()

                    # Convert timestamp to rospy.Time
                    timestamp_ns = int(timestamp)
                    timestamp_sec = timestamp_ns // 1000000000
                    timestamp_nsec = timestamp_ns % 1000000000
                    t.header.stamp = rospy.Time(secs=timestamp_sec, nsecs=timestamp_nsec)
                    t.header.frame_id = data[1]
                    t.child_frame_id = data[2]

                    # Set the translation data
                    t.transform.translation.x = float(data[3])
                    t.transform.translation.y = float(data[4])
                    t.transform.translation.z = float(data[5])

                    # Set the rotation data
                    t.transform.rotation.x = float(data[6])
                    t.transform.rotation.y = float(data[7])
                    t.transform.rotation.z = float(data[8])
                    t.transform.rotation.w = float(data[9])

                    # Create a TFMessage and add the TransformStamped message
                    tfm = TFMessage([t])

                    # Write to bag
                    bag.write(ros_topic, tfm, t.header.stamp)
        # Process IMU Data
        imu_mappings = {
            'imu0': '/tesse/imu'
        }
        for imu_id, ros_topic in imu_mappings.items():
            data_csv_path = os.path.join(base_path, imu_id, 'data.csv')

            with open(data_csv_path, 'r') as file:
                for line in file.readlines()[1:]:  # Skip header
                    data = line.strip().split(',')
                    timestamp = data[0]
                    imu_msg = Imu()

                    # Convert timestamp to rospy.Time
                    timestamp_ns = int(timestamp)
                    timestamp_sec = timestamp_ns // 1000000000
                    timestamp_nsec = timestamp_ns % 1000000000
                    imu_msg.header.stamp = rospy.Time(secs=timestamp_sec, nsecs=timestamp_nsec)

                    imu_msg.angular_velocity.x = float(data[1])
                    imu_msg.angular_velocity.y = float(data[2])
                    imu_msg.angular_velocity.z = float(data[3])
                    imu_msg.linear_acceleration.x = float(data[4])
                    imu_msg.linear_acceleration.y = float(data[5])
                    imu_msg.linear_acceleration.z = float(data[6])

                    bag.write(ros_topic, imu_msg, imu_msg.header.stamp)

        # Process Camera Info Data
        camera_info_mappings = {
            'camerainfo0': '/tesse/left_cam/camera_info',
            'camerainfo1': '/tesse/right_cam/camera_info'
        }
        for folder, topic in camera_info_mappings.items():
            data_csv_path = os.path.join(base_path, folder, 'data.csv')
            data_folder = os.path.join(base_path, folder, 'data')

            with open(data_csv_path, 'r') as file:
                for line in file.readlines()[1:]:  # Skip header
                    timestamp, filename = line.strip().split(',')
                    yaml_file_path = os.path.join(data_folder, filename)

                    with open(yaml_file_path, 'r') as yaml_file:
                        camera_info_data = convert_python_yaml_to_standard(yaml_file_path)

                        camera_info_msg = CameraInfo()

                        # Convert timestamp to rospy.Time
                        timestamp_ns = int(timestamp)
                        timestamp_sec = timestamp_ns // 1000000000
                        timestamp_nsec = timestamp_ns % 1000000000
                        camera_info_msg.header.stamp = rospy.Time(secs=timestamp_sec, nsecs=timestamp_nsec)

                        camera_info_msg.width = camera_info_data['width']
                        camera_info_msg.height = camera_info_data['height']
                        camera_info_msg.distortion_model = camera_info_data['distortion_model']
                        camera_info_msg.D = camera_info_data['D']
                        camera_info_msg.K = camera_info_data['K']
                        camera_info_msg.R = camera_info_data['R']
                        camera_info_msg.P = camera_info_data['P']

                        bag.write(topic, camera_info_msg, camera_info_msg.header.stamp)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='Create a ROS bag from dataset files.')
    parser.add_argument('base_path', help='Base path of the dataset.')
    parser.add_argument('output_rosbag_path', help='Output path for the ROS bag file.')
    args = parser.parse_args()

    create_rosbag_from_files(args.base_path, args.output_rosbag_path)
