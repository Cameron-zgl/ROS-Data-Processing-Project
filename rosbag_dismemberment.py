#!/usr/bin/env python3

import sys
import argparse
import os
import subprocess, yaml
import errno

import cv2

import roslib
import rosbag
import rospy
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

CAM_FOLDER_NAME = 'cam'
IMU_FOLDER_NAME = 'imu'
DATA_CSV = 'data.csv'
SENSOR_YAML = 'sensor.yaml'
BODY_YAML = 'body.yaml'

CAM_SENSOR_YAML = dict(
    sensor_type= "camera",
    comment= "VI-Sensor cam0 (MT9M034)", 
    T_BS= dict(
        cols= 4,
        rows= 4,
        data= [0.0148655429818, -0.999880929698, 0.00414029679422, -0.0216401454975,
               0.999557249008, 0.0149672133247, 0.025715529948, -0.064676986768,
               -0.0257744366974, 0.00375618835797, 0.999660727178, 0.00981073058949,
               0.0, 0.0, 0.0, 1.0]
    ),
    rate_hz= 20, 
    resolution= [720, 480],
    camera_model= "pinhole",
    intrinsics= [415.692, 415.692, 360.0, 240.0],
    distortion_model= "radial-tangential",
    distortion_coefficients= [0.0, 0.0, 0.0, 0.0]
)


# Can get this from ros topic
IMU_SENSOR_YAML = dict(
    sensor_type= "imu",
    comment= "VI-Sensor IMU (ADIS16448)",
    T_BS= dict(
        cols= 4,
        rows= 4,
        data= [1.0, 0.0, 0.0, 0.0,
               0.0, 1.0, 0.0, 0.0,
               0.0, 0.0, 1.0, 0.0,
               0.0, 0.0, 0.0, 1.0]
    ),
    rate_hz= 200,
    gyroscope_noise_density= 1.6968e-04,
    gyroscope_random_walk= 1.9393e-05,
    accelerometer_noise_density= 2.0000e-3,
    accelerometer_random_walk= 3.0000e-3
)

def get_rosbag_metadata(rosbag_path):
    assert(os.path.exists(rosbag_path))
    # This subprocess will only work if ROS is sourced...
    return yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', rosbag_path],
                                      stdout=subprocess.PIPE).communicate()[0])

def mkdirs_without_exception(path):
    try:
       os.makedirs(path)
    except OSError as e:
        if e.errno == errno.EEXIST:
            print("The directory {} already exists.".format(path))
        else:
            print(e)
            raise  # raises the error again
CAMERA_INFO_FOLDER_NAME = 'camerainfo'

def setup_dataset_dirs(rosbag_path, output_path, camera_topics, imu_topics, camera_info_topics,tf_topics,transform_stamped_topics):
    # Create base folder
    dirname = os.path.split(rosbag_path)[-1].split(".", 1)[0] + '/mav0'
    base_path = os.path.join(output_path, dirname)
    mkdirs_without_exception(base_path)

    #setup_dataset_dirs 
    transform_stamped_folder_paths = []
    for i in range(len(transform_stamped_topics)):
        transform_stamped_folder_path = os.path.join(base_path, 'transform_stamped' + repr(i))
        transform_stamped_folder_paths.append(transform_stamped_folder_path)
        mkdirs_without_exception(transform_stamped_folder_path)

        # data.csv
        with open(os.path.join(transform_stamped_folder_path, DATA_CSV), 'w+') as outfile:
            outfile.write('#timestamp [ns],frame_id,child_frame_id,tx,ty,tz,rx,ry,rz,rw\n')

    tf_folder_paths = []
    for i in range(len(tf_topics)):
        tf_folder_path = os.path.join(base_path, 'tf' + repr(i))
        tf_folder_paths.append(tf_folder_path)
        mkdirs_without_exception(tf_folder_path)

        # Create data.csv file
        with open(os.path.join(tf_folder_path, DATA_CSV), 'w+') as outfile:
            outfile.write('#timestamp [ns],frame_id,child_frame_id,tx,ty,tz,rx,ry,rz,rw\n')
    # Create folder for camera topics
    cam_folder_paths = []
    for i in range(len(camera_topics)):
        cam_folder_paths.append(os.path.join(base_path, CAM_FOLDER_NAME + repr(i)))
        current_cam_folder_path = cam_folder_paths[-1]
        mkdirs_without_exception(current_cam_folder_path)

        # Create data folder
        mkdirs_without_exception(os.path.join(current_cam_folder_path, 'data'))

        # Create data.csv file
        with open(os.path.join(current_cam_folder_path, DATA_CSV), 'w+') as outfile:
            outfile.write('#timestamp [ns],filename')

        # Create sensor.yaml file
        with open(os.path.join(current_cam_folder_path, SENSOR_YAML), 'w+') as outfile:
            outfile.write("%YAML:1.0\n")
            CAM_SENSOR_YAML['comment'] = CAM_FOLDER_NAME + repr(i)
            yaml.dump(CAM_SENSOR_YAML, outfile, default_flow_style=True)

    # Create folder for camera_info topics
    camera_info_folder_paths = []
    for i in range(len(camera_info_topics)):
        camera_info_folder_paths.append(os.path.join(base_path, CAMERA_INFO_FOLDER_NAME + repr(i)))
        current_camera_info_folder_path = camera_info_folder_paths[-1]
        mkdirs_without_exception(current_camera_info_folder_path)

        # Create data.csv file
        with open(os.path.join(current_camera_info_folder_path, DATA_CSV), 'w+') as outfile:
            outfile.write("#timestamp [ns],filename")


    # Create folder for imu topics
    imu_folder_paths = []
    for i in range(len(imu_topics)):
        imu_folder_paths.append(os.path.join(base_path, IMU_FOLDER_NAME + repr(i)))
        current_imu_folder_path = imu_folder_paths[-1]
        mkdirs_without_exception(current_imu_folder_path)

        # Create data.csv file
        with open(os.path.join(current_imu_folder_path, DATA_CSV), 'w+') as outfile:
            outfile.write("#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]")

        # Create sensor.yaml file
        with open(os.path.join(current_imu_folder_path, SENSOR_YAML), 'w+') as outfile:
            outfile.write("%YAML:1.0\n")
            IMU_SENSOR_YAML['comment'] = IMU_FOLDER_NAME + repr(i)
            yaml.dump(IMU_SENSOR_YAML, outfile, default_flow_style=True)

    # Create body.yaml file
    with open(os.path.join(base_path, BODY_YAML), 'w+') as outfile:
        outfile.write("%YAML:1.0\n")
        body_yaml = dict(comment = 'Automatically generated dataset using Rosbag2data, using rosbag: {}'.format(rosbag_path))
        yaml.dump(body_yaml, outfile, default_flow_style=True)

    return cam_folder_paths, imu_folder_paths, camera_info_folder_paths, tf_folder_paths,transform_stamped_folder_paths


def rosbag_2_data(rosbag_path, output_path):
    # Check that the path to the rosbag exists.
    assert(os.path.exists(rosbag_path))
    bag = rosbag.Bag(rosbag_path)

    # Check that rosbag has the data we need to convert to dataset format.
    bag_metadata = get_rosbag_metadata(rosbag_path)
    camera_topics = []
    imu_topics = []
    camera_info_topics = []
    tf_topics = []
    transform_stamped_topics = []
    for element in bag_metadata['topics']:
        if element['type'] == 'geometry_msgs/TransformStamped':
            transform_stamped_topics.append(element['topic'])
    for element in bag_metadata['topics']:
        if element['type'] == 'tf2_msgs/TFMessage':
            tf_topics.append(element['topic'])
    for element in bag_metadata['topics']:
        if element['type'] == 'sensor_msgs/CameraInfo':
            camera_info_topics.append(element['topic'])
    for element in bag_metadata['topics']:
        if (element['type'] == 'sensor_msgs/Image'):
            camera_topics.append(element['topic'])
        elif (element['type'] == 'sensor_msgs/Imu'):
            imu_topics.append(element['topic'])

    # Check that it has one or two Image topics.
    if not camera_topics:
        print ("WARNING: there are no camera topics in this rosbag!")

    # Check that it has one, and only one, IMU topic.
    if imu_topics != 1:
        print ("WARNING: expected to have a single IMU topic, instead got: {} topic(s)".format(
            len(imu_topics)))

    # Build output folder.
    # Inside rosbag_2_data function
    # Update this line to include transform_stamped_folder_paths
    cam_folder_paths, imu_folder_paths, camera_info_folder_paths, tf_folder_paths, transform_stamped_folder_paths = setup_dataset_dirs(
        rosbag_path, output_path, camera_topics, imu_topics, camera_info_topics, tf_topics, transform_stamped_topics)


    # Use a CvBridge to convert ROS images to OpenCV images so they can be saved.
    cv_bridge = CvBridge()

    # Convert image msg to data dataset format.
    assert (len(camera_topics) == len(cam_folder_paths))
    for i, cam_topic in enumerate(camera_topics):

        # Write data.csv file.
        with open(os.path.join(cam_folder_paths[i], DATA_CSV), 'a') as outfile:
            for _, msg, t in bag.read_messages(topics=[cam_topic]):
                # Determine the file extension based on the encoding
                file_extension = '.npy' if '32FC1' in msg.encoding else '.png'
                image_filename = str(msg.header.stamp) + file_extension
                outfile.write('\n' + str(msg.header.stamp) + "," + image_filename)
                try:
                    # Convert ROS image message to OpenCV image
                    cv_image = cv_bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

                    # Handle different encodings
                    if '8UC' in msg.encoding:
                        # For 8-bit images (most common for color and grayscale images)
                        cv2.imwrite(os.path.join(cam_folder_paths[i], 'data', image_filename), cv_image)
                    elif '16UC' in msg.encoding:
                        # For 16-bit images (common for some depth sensors)
                        cv2.imwrite(os.path.join(cam_folder_paths[i], 'data', image_filename), cv_image,
                                    [cv2.IMWRITE_PNG_COMPRESSION, 0])
                    elif '32FC1' in msg.encoding:
                        # For 32-bit float images, save as NPY
                        np.save(os.path.join(cam_folder_paths[i], 'data', str(msg.header.stamp)), cv_image)

                    else:
                        # Log an error for unsupported encodings
    			if msg.encoding == 'rgb8':
        		# Convert from RGB (ROS default) to BGR (OpenCV default)
       			    cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
                        if msg.encoding == 'bgr8':
                            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
                            # Save as PNG
                        cv2.imwrite(os.path.join(cam_folder_paths[i], 'data', image_filename), cv_image)


                except CvBridgeError as e:
                    print(e)

    # Convert TF msg to dataset format.
    assert (len(tf_topics) == len(tf_folder_paths))
    for i, tf_topic in enumerate(tf_topics):
        with open(os.path.join(tf_folder_paths[i], DATA_CSV), 'a') as outfile:
            for _, msg, t in bag.read_messages(topics=[tf_topic]):
                for transform in msg.transforms:
                    # Extract translation and rotation data
                    translation = transform.transform.translation
                    rotation = transform.transform.rotation
                    # Format the data as a string for CSV output using `format`
                    transform_data = "{},{},{},{},{},{},{}".format(
                        translation.x, translation.y, translation.z,
                        rotation.x, rotation.y, rotation.z, rotation.w
                    )
                    # Write the formatted data to the CSV file
                    outfile.write("{},{},{},{}\n".format(
                        t, transform.header.frame_id, transform.child_frame_id, transform_data
                    ))

    # Convert IMU msg to dataset format.
    assert(len(imu_topics) == len(imu_folder_paths))
    for i, imu_topic in enumerate(imu_topics):
        with open(os.path.join(imu_folder_paths[i], DATA_CSV), 'a') as outfile:
	    outfile.write('\n')

            for _, msg, t in bag.read_messages(topics=[imu_topic]):
                outfile.write(str(msg.header.stamp) + ',' +
                              str(msg.angular_velocity.x) + ',' +
                              str(msg.angular_velocity.y) + ',' +
                              str(msg.angular_velocity.z) + ',' +
                              str(msg.linear_acceleration.x) + ',' +
                              str(msg.linear_acceleration.y) + ',' +
                              str(msg.linear_acceleration.z)+ '\n')

    assert (len(camera_info_topics) == len(camera_info_folder_paths))
    for i, camera_info_topic in enumerate(camera_info_topics):
        camera_info_data_path = os.path.join(camera_info_folder_paths[i], 'data/')
        if not os.path.exists(camera_info_data_path):
            os.makedirs(camera_info_data_path)

        with open(os.path.join(camera_info_folder_paths[i], DATA_CSV), 'a') as outfile:
            for _, msg, t in bag.read_messages(topics=[camera_info_topic]):
                camera_info_filename = str(msg.header.stamp) + '.yaml'
                outfile.write('\n' + str(msg.header.stamp) + "," + camera_info_filename)
                camera_info_data = {
                    'width': msg.width,
                    'height': msg.height,
                    'distortion_model': msg.distortion_model,
                    'D': msg.D,
                    'K': msg.K,
                    'R': msg.R,
                    'P': msg.P,
                }
                with open(os.path.join(camera_info_data_path, camera_info_filename), 'w') as yaml_file:
                    yaml.dump(camera_info_data, yaml_file, default_flow_style=True)

    assert (len(transform_stamped_topics) == len(transform_stamped_folder_paths))
    for i, topic in enumerate(transform_stamped_topics):
        with open(os.path.join(transform_stamped_folder_paths[i], DATA_CSV), 'a') as outfile:
            for _, msg, t in bag.read_messages(topics=[topic]):
                # 
                transform = msg.transform
                translation = transform.translation
                rotation = transform.rotation

                #  CSV
                transform_data = "{},{},{},{},{},{},{}".format(
                    translation.x, translation.y, translation.z,
                    rotation.x, rotation.y, rotation.z, rotation.w
                )
                outfile.write("{},{},{},{}\n".format(
                    msg.header.stamp, msg.header.frame_id, msg.child_frame_id, transform_data
                ))

    # Close the rosbag.
    bag.close()

if __name__ == "__main__":
    # Parse rosbag path.
    parser = argparse.ArgumentParser(description='Process some integers.')
    parser.add_argument('rosbag_path', help='Path to the rosbag.')
    parser.add_argument('-o', '--output_path', help='Path to the output.', default='./')
    args = parser.parse_args()

    # Convert rosbag.

    rosbag_2_data(args.rosbag_path, args.output_path)
    print("Done.")

