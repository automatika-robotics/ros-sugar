"""ROS Topics Supported Message Types"""

from typing import Any, Union, Optional
import base64

import numpy as np

# GEOMETRY MSGS SUPPORTED ROS TYPES
from geometry_msgs.msg import Point as ROSPoint
from geometry_msgs.msg import PointStamped as ROSPointStamped
from geometry_msgs.msg import Pose as ROSPose
from geometry_msgs.msg import PoseStamped as ROSPoseStamped
from geometry_msgs.msg import Twist as ROSTwist

# NAV_MSGS SUPPORTED ROS TYPES
from nav_msgs.msg import MapMetaData as ROSMapMetaData
from nav_msgs.msg import OccupancyGrid as ROSOccupancyGrid
from nav_msgs.msg import Odometry as ROSOdometry
from nav_msgs.msg import Path as ROSPath
from sugar.msg import ComponentStatus as ROSComponentStatus

# SENSOR_MSGS SUPPORTED ROS TYPES
from sensor_msgs.msg import Image as ROSImage
from sensor_msgs.msg import LaserScan as ROSLaserScan

# STD_MSGS SUPPORTED ROS TYPES
from std_msgs.msg import Header
from std_msgs.msg import ByteMultiArray
from std_msgs.msg import String as ROSString
from std_msgs.msg import Bool as ROSBool
from std_msgs.msg import Float32 as ROSFloat32
from std_msgs.msg import Float64 as ROSFloat64

from . import callbacks


class Meta(type):
    """Meta."""

    def __repr__(cls):
        """__repr__.
        :param cls:
        """
        return cls.__name__

    def __str__(cls):
        """__str__.
        :param cls:
        """
        return cls.__name__


class SupportedType:
    """
    Class used to define all supported data types (ROS messages)
    callback: Callback associated with specified type
    _publish: ROS message creation method associate with specified type
    """

    # set metaclass
    __metaclass__ = Meta

    # associated ROS type
    _ros_type: type

    # callback class
    callback = callbacks.GenericCallback

    @classmethod
    def convert(cls, output, **_) -> Any:
        """ROS message converter function for datatype
        :param args:
        :type _: Any
        :rtype: Any
        """
        return output


class String(SupportedType):
    """String."""

    _ros_type = ROSString
    callback = callbacks.TextCallback

    @classmethod
    def convert(cls, output: str, **_) -> ROSString:
        """
        Takes a string and returns a ROS message of type String
        :return: String
        """
        msg = ROSString()
        msg.data = output
        return msg


class Bool(SupportedType):
    """Bool."""

    _ros_type = ROSBool
    callback = callbacks.StdMsgCallback

    @classmethod
    def convert(cls, output: bool, **_) -> ROSBool:
        """
        Takes a bool and returns a ROS message of type Bool
        :return: Bool
        """
        msg = ROSBool()
        msg.data = output
        return msg


class Float32(SupportedType):
    """Float32."""

    _ros_type = ROSFloat32
    callback = callbacks.StdMsgCallback

    @classmethod
    def convert(cls, output: float, **_) -> ROSFloat32:
        """
        Takes a bool and returns a ROS message of type Bool
        :return: Float32
        """
        msg = ROSFloat32()
        msg.data = output
        return msg


class Float64(SupportedType):
    """Float64."""

    _ros_type = ROSFloat64
    callback = callbacks.StdMsgCallback

    @classmethod
    def convert(cls, output: float, **_) -> ROSFloat64:
        """
        Takes a bool and returns a ROS message of type Bool
        :return: Float64
        """
        msg = ROSFloat64()
        msg.data = output
        return msg


class Image(SupportedType):
    """Image."""

    _ros_type = ROSImage
    callback = callbacks.ImageCallback

    @classmethod
    def convert(cls, output: Union[ROSImage, np.ndarray], **_) -> ROSImage:
        """
        Takes a PIL Image and returns a ROS message
         of type Image
        :return: Image
        """
        if isinstance(output, ROSImage):
            return output
        msg = ROSImage()
        msg.height = output.shape[0]
        msg.width = output.shape[1]
        msg.data = output.flatten()
        return msg


class Audio(SupportedType):
    """Audio."""

    _ros_type = ByteMultiArray
    callback = callbacks.AudioCallback

    @classmethod
    def convert(cls, output: Union[str, bytes], **_) -> ByteMultiArray:
        """
        Takes an array of audio data and returns a ROS message
         of type AudioData
        :return: AudioData
        """
        # Handle base64 encoded strings
        if isinstance(output, str):
            output = base64.b64decode(output)

        msg = ByteMultiArray()
        msg.data = output
        return msg


class MapMetaData(SupportedType):
    """MapMetaData."""

    _ros_type = ROSMapMetaData
    callback = callbacks.MapMetaDataCallback


class Odometry(SupportedType):
    """Odometry"""

    _ros_type = ROSOdometry
    callback = callbacks.OdomCallback


class LaserScan(SupportedType):
    """LaserScan"""

    _ros_type = ROSLaserScan
    callback = callbacks.GenericCallback


class Path(SupportedType):
    """Path"""

    _ros_type = ROSPath
    callback = callbacks.GenericCallback

    @classmethod
    def convert(cls, output, **_) -> Any:
        """ROS message converter function for datatype
        :param args:
        :type _: Any
        :rtype: Any
        """
        return output


class OccupancyGrid(SupportedType):
    """OccupancyGrid"""

    _ros_type = ROSOccupancyGrid
    callback = callbacks.OccupancyGridCallback

    @classmethod
    def convert(
        cls,
        output: np.ndarray,
        resolution: float,
        origin: Optional[ROSPose] = None,
        msg_header: Optional[Header] = None,
        **_,
    ) -> ROSOccupancyGrid:
        """ROS message converter function for datatype OccupancyGrid.

        :param output:
        :type output: np.ndarray
        :param _:
        :rtype: ROSOccupancyGrid
        """
        if not len(output.shape) == 2:
            raise TypeError("OccupancyGrid data must be a 2D array")

        msg = ROSOccupancyGrid()
        msg.header = msg_header if msg_header else Header()

        # Set MetaData
        msg.info = ROSMapMetaData()
        msg.info.map_load_time = msg_header.stamp
        msg.info.width = output.shape[0]
        msg.info.height = output.shape[1]
        msg.info.resolution = resolution
        msg.info.origin = origin if origin else Pose()

        # flatten by column
        # index (0,0) is the lower right corner of the grid in ROS
        msg.data = output.flatten("F")

        return msg


class Point(SupportedType):
    """Point"""

    _ros_type = ROSPoint
    callback = callbacks.PointCallback

    @classmethod
    def convert(cls, output: np.ndarray, **_) -> ROSPoint:
        """ROS message converter function for datatype Point.

        :param output:
        :type output: np.ndarray
        :param _:
        :rtype: ROSPoint
        """
        msg = ROSPoint()
        if output.shape[0] < 3:
            raise ValueError(
                f"Cannot convert given value {output} to a ROS Point message"
            )
        msg.x = output[0]
        msg.y = output[1]
        msg.z = output[2]
        return msg


class PointStamped(SupportedType):
    """PointStamped"""

    _ros_type = ROSPointStamped
    callback = callbacks.PointStampedCallback

    @classmethod
    def convert(
        cls, output: np.ndarray, frame_id=None, ros_time=None, **_
    ) -> ROSPointStamped:
        """ROS message converter function for datatype Point.

        :param output:
        :type output: np.ndarray
        :param _:
        :rtype: ROSPointStamped
        """
        msg = ROSPointStamped()
        msg_header = Header()
        if frame_id:
            msg_header.frame_id = frame_id
        if ros_time:
            msg_header.stamp = ros_time
        msg.header = msg_header
        if output.shape[0] < 3:
            raise ValueError(
                f"Cannot convert given value '{output}' to a ROS PointStamped message"
            )
        msg.point.x = output[0]
        msg.point.y = output[1]
        msg.point.z = output[2]
        return msg


class Pose(SupportedType):
    """Pose"""

    _ros_type = ROSPose
    callback = callbacks.PoseCallback

    @classmethod
    def convert(cls, output: np.ndarray, frame_id=None, ros_time=None, **_) -> ROSPose:
        """ROS message converter function for datatype Point.

        :param output:
        :type output: np.ndarray
        :param _:
        :rtype: ROSPose
        """
        msg = ROSPose()
        if output.shape[0] < 3:
            raise ValueError(
                f"Cannot convert given value '{output}' to a ROS Pose message"
            )
        msg.pose.position.x = output[0]
        msg.pose.position.y = output[1]
        msg.pose.position.z = output[2]

        # Check for orientation
        if output.shape[0] == 7:
            msg.pose.orientation.w = output[3]
            msg.pose.orientation.x = output[4]
            msg.pose.orientation.y = output[5]
            msg.pose.orientation.z = output[6]
        return msg


class PoseStamped(SupportedType):
    """PoseStamped"""

    _ros_type = ROSPoseStamped
    callback = callbacks.PoseStampedCallback

    @classmethod
    def convert(
        cls, output: np.ndarray, frame_id=None, ros_time=None, **_
    ) -> ROSPoseStamped:
        """ROS message converter function for datatype Point.

        :param output:
        :type output: np.ndarray
        :param _:
        :rtype: ROSPoseStamped
        """
        msg = ROSPoseStamped()
        msg_header = Header()
        if frame_id:
            msg_header.frame_id = frame_id
        if ros_time:
            msg_header.stamp = ros_time
        msg.header = msg_header
        if output.shape[0] < 3:
            raise ValueError(
                f"Cannot convert given value '{output}' to a ROS PoseStamped message"
            )
        msg.pose.position.x = output[0]
        msg.pose.position.y = output[1]
        msg.pose.position.z = output[2]
        # Check for orientation
        if output.shape[0] == 7:
            msg.pose.orientation.w = output[3]
            msg.pose.orientation.x = output[4]
            msg.pose.orientation.y = output[5]
            msg.pose.orientation.z = output[6]
        return msg


class ComponentStatus(SupportedType):
    """Component Health Status"""

    _ros_type = ROSComponentStatus


class Twist(SupportedType):
    """Twist for Control Commands"""

    _ros_type = ROSTwist
