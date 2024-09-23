from typing import List

import numpy as np
from nav_msgs.msg import Odometry
from quaternion import quaternion
import cv2


def image_pre_processing(img) -> np.ndarray:
    """
    Pre-processing of ROS image msg received in different encodings
    :param      img:  Image as a middleware defined message
    :type       img:  Middleware defined message type

    :returns:   Image as an numpy array
    :rtype:     Numpy array
    """
    if img.encoding == "yuv422_yuy2":
        np_arr = np.asarray(img.data, dtype="uint8").reshape((img.height, img.width, 2))
        rgb = cv2.cvtColor(np_arr, cv2.COLOR_YUV2RGB_YUYV)

    # discard alpha channels if present
    elif "a" in img.encoding:
        np_arr = np.asarray(img.data, dtype="uint8").reshape((img.height, img.width, 4))
        np_arr = np_arr[:, :, :3]
    else:
        np_arr = np.asarray(img.data, dtype="uint8").reshape((img.height, img.width, 3))

    # handle bgr
    rgb = cv2.cvtColor(np_arr, cv2.COLOR_BGR2RGB) if "bgr" in img.encoding else np_arr

    return rgb


def rotate_vector_by_quaternion(q: quaternion, v: List) -> List:
    """
    rotate a vector v by a rotation quaternion q

    :param      q: the rotation to perform
    :type       q: quaternion.quaternion
    :param      v: the vector to be rotated
    :type       v: List

    :return:    the rotated position of the vector
    :rtype:     List
    """
    vq = quaternion(0, 0, 0, 0)
    vq.imag = v
    return (q * vq * q.inverse()).imag


def get_pose_target_in_reference_frame(
    reference_position: np.ndarray,
    reference_orientation: quaternion,
    target_position: np.ndarray,
    target_orientation: quaternion,
) -> np.ndarray:
    """
    Computes a target pose with respect to a reference pose, both given in a common coordinates frame

    :param reference_position: Position of reference in common frame [x, y, z]
    :type reference_position: np.ndarray
    :param reference_orientation: Orientation quaternion of reference in common frame (qw, qx, qy, qz)
    :type reference_orientation: quaternion
    :param target_position: Position of target in common frame [x, y, z]
    :type target_position: np.ndarray
    :param target_orientation: Orientation quaternion of target in common frame (qw, qx, qy, qz)
    :type target_orientation: quaternion

    :return: Position and oreintation quaternion of target in reference frame [x, y, z, qw, qx, qy, qz]
    :rtype: np.ndarray
    """
    orientation_target_in_ref = reference_orientation.inverse() * target_orientation
    position_target_in_ref = rotate_vector_by_quaternion(
        reference_orientation.inverse(), (target_position - reference_position).tolist()
    )

    target_pose_in_ref = np.array([
        position_target_in_ref[0],
        position_target_in_ref[1],
        position_target_in_ref[2],
        orientation_target_in_ref.w,
        orientation_target_in_ref.x,
        orientation_target_in_ref.y,
        orientation_target_in_ref.z,
    ])

    return target_pose_in_ref


def _get_position_from_odom(odom_msg: Odometry) -> np.ndarray:
    """
    Gets a numpy array with 3D position coordinates from Odometry message

    :param odom_msg: ROS Odometry message
    :type odom_msg: Odometry

    :return: 3D position [x, y, z]
    :rtype: np.ndarray
    """
    return np.array([
        odom_msg.pose.pose.position.x,
        odom_msg.pose.pose.position.y,
        odom_msg.pose.pose.position.z,
    ])


def _get_odom_from_ndarray(odom_array: np.ndarray) -> Odometry:
    """Convert numpy array to an odometry message.

    :param odom_array:
    :type odom_array: np.ndarray
    :rtype: Odometry
    """
    odom_msg = Odometry()
    odom_msg.pose.pose.position.x = odom_array[0]
    odom_msg.pose.pose.position.y = odom_array[1]
    odom_msg.pose.pose.position.z = odom_array[2]
    odom_msg.pose.pose.orientation.w = odom_array[3]
    odom_msg.pose.pose.orientation.x = odom_array[4]
    odom_msg.pose.pose.orientation.y = odom_array[5]
    odom_msg.pose.pose.orientation.z = odom_array[6]

    return odom_msg


def _get_orientation_from_odom(odom_msg: Odometry) -> quaternion:
    """
    Gets a rotation quaternion from Odometry message

    :param odom_msg: ROS Odometry message
    :type odom_msg: Odometry

    :return: Rotation quaternion (qw, qx, qy, qz)
    :rtype: quaternion
    """
    return quaternion(
        odom_msg.pose.pose.orientation.w,
        odom_msg.pose.pose.orientation.x,
        odom_msg.pose.pose.orientation.y,
        odom_msg.pose.pose.orientation.z,
    )


def odom_from_frame1_to_frame2(
    pose_1_in_2: Odometry, pose_target_in_1: Odometry
) -> Odometry:
    """
    get the pose of a target in frame 2 instead of frame 1

    :param      pose_1_in_2:        pose of frame 1 in frame 2
    :type       pose_1_in_2:        PoseData
    :param      pose_target_in_1:   pose of target in frame 1
    :type       pose_target_in_1:   PoseData

    :return:    pose of target in frame 2
    :rtype:     PoseData
    """
    pose_2_origin = Odometry()

    pose_2_in_1 = get_pose_target_in_reference_frame(
        reference_position=_get_position_from_odom(pose_1_in_2),
        reference_orientation=_get_orientation_from_odom(pose_1_in_2),
        target_position=_get_position_from_odom(pose_2_origin),
        target_orientation=_get_orientation_from_odom(pose_2_origin),
    )

    pose_target_in_2 = get_pose_target_in_reference_frame(
        reference_position=pose_2_in_1[:3],
        reference_orientation=quaternion(
            pose_2_in_1[3], pose_2_in_1[4], pose_2_in_1[5], pose_2_in_1[6]
        ),
        target_position=_get_position_from_odom(pose_target_in_1),
        target_orientation=_get_orientation_from_odom(pose_target_in_1),
    )
    target_odom = _get_odom_from_ndarray(pose_target_in_2)

    return target_odom
