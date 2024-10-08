"""ROS Subscribers Callback Classes"""

import os
from abc import abstractmethod
from typing import Any, Callable, Optional, Union, Dict
import numpy as np
from geometry_msgs.msg import Pose
from jinja2.environment import Template
from nav_msgs.msg import OccupancyGrid, Odometry
from PIL import Image as PILImage
from rclpy.logging import get_logger
from rclpy.subscription import Subscription
from tf2_ros import TransformStamped

from . import utils


class GenericCallback:
    """GenericCallback."""

    def __init__(self, input_topic, node_name: Optional[str] = None) -> None:
        """__init__

        :param input_topic:
        :type input_topic: Topic
        :param node_name: Name of the ros node using the callback object, defaults to None
        :type node_name: Optional[str], optional
        """

        self.input_topic = input_topic

        # Node name can be changed to a node that the callback is executed in
        # at the time of setting subscriber using set_node_name
        self.node_name: Optional[str] = node_name
        self.msg = None

        self._extra_callback: Optional[Callable] = None
        self._subscriber: Optional[Subscription] = None
        self._post_processors: Optional[list[Callable]] = None

    def set_node_name(self, node_name: str) -> None:
        """Set node name.

        :param node_name:
        :type node_name: str
        :rtype: None
        """
        self.node_name = node_name

    def set_subscriber(self, subscriber: Subscription) -> None:
        """set_subscriber.

        :param subscriber:
        :rtype: None
        """
        self._subscriber = subscriber

    def on_callback_execute(self, callback: Callable) -> None:
        """Attach a method to be executed on topic callback

        :param callback:
        :type callback: Callable
        """
        self._extra_callback = callback

    def callback(self, msg) -> None:
        """
        Topic subscriber callback

        :param msg: Received ros msg
        :type msg: Any
        """
        self.msg = msg
        if self._extra_callback:
            self._extra_callback(
                msg=msg, topic=self.input_topic, output=self.get_output()
            )

    def add_post_processor(self, method: Callable):
        """Add a post processor for callback message

        :param method: Post processor method
        :type method: Callable
        """
        if not self._post_processors:
            self._post_processors = [method]
        else:
            self._post_processors.append(method)

    def get_output(self, **kwargs) -> Any:
        """Post process outputs based on custom callables.
        :param output:
        :param args:
        :param kwargs:
        """
        output = self._get_output(**kwargs)
        if self._post_processors:
            # Apply post processors sequentially if defined
            for processor in self._post_processors:
                post_output = processor(output)
                # if any processor output is None, then send None
                if not post_output:
                    return None
                # type check processor output if incorrect, raise an error
                if type(post_output) is not type(output):
                    raise TypeError(
                        f"The output type of a post_processor for topic {self.input_topic.name} callback should be of type {type(output).__name__} | None. Got post_processor output of type {type(post_output).__name__}"
                    )
                # if all good, set output equal to post output
                output = post_output

        return output

    @abstractmethod
    def _get_output(self, **_) -> Any:
        """
        Gets the output.
        :returns:   Topic content
        :rtype:     Any
        """
        return self.msg

    @property
    def got_msg(self):
        """
        Property is true if an input is received on the topic
        """
        return True if self.msg else False

    def clear_last_msg(self):
        """Clears the last received message on the topic"""
        self.msg = None


class StdMsgCallback(GenericCallback):
    """std_msgs callback"""

    def __init__(self, input_topic, node_name: Optional[str] = None) -> None:
        super().__init__(input_topic, node_name)

    def _get_output(self, **_):
        """
        Gets std_msg data
        """
        if not self.msg:
            return None
        return self.msg.data


class ImageCallback(GenericCallback):
    """
    Image Callback class. Its get method saves an image as bytes
    """

    def __init__(self, input_topic) -> None:
        """
        Constructs a new instance.
        :param      input_topic:  Subscription topic
        :type       input_topic:  Input
        """
        super().__init__(input_topic)
        # fixed image needs to be a path to PIL readable image
        if hasattr(input_topic, "fixed"):
            if os.path.isfile(input_topic.fixed):
                try:
                    self.msg = PILImage.open(input_topic.fixed)
                except Exception:
                    get_logger(self.node_name).error(
                        f"Fixed path {input_topic.fixed} provided for Image topic is not readable PIL image"
                    )
            else:
                get_logger(self.node_name).error(
                    f"Fixed path {input_topic.fixed} provided for Image topic is not a valid file path"
                )

    def _get_output(self, **_) -> Optional[np.ndarray]:
        """
        Gets image as a byte array.
        :returns:   Image as bytes
        :rtype:     bytes
        """
        if not self.msg:
            return None

        # return bytes if fixed image has been read
        if isinstance(self.msg, PILImage.Image):
            return np.array(self.msg)
        else:
            # pre-process in case of weird encodings and reshape ROS topic
            return utils.image_pre_processing(self.msg)


class TextCallback(GenericCallback):
    """
    Text Callback class. Its get method returns the text
    """

    def __init__(self, input_topic) -> None:
        """
        Constructs a new instance.

        :param      input_topic:  Subscription topic
        :type       input_topic:  str
        """
        super().__init__(input_topic)
        self.msg = input_topic.fixed if hasattr(input_topic, "fixed") else None
        self._template: Optional[Template] = None

    def _get_output(self, **_) -> Optional[str]:
        """Gets text.
        :rtype: str | None
        """

        if not self.msg:
            return None

        # return str if fixed str has been read
        if isinstance(self.msg, str):
            return self.msg
        # return ROS message data
        else:
            if template := self._template:
                context = {self.input_topic.name: self.msg.data}
                return template.render(context)
            return self.msg.data


class AudioCallback(GenericCallback):
    """
    Audio Callback class. Its get method returns the audio
    """

    def __init__(self, input_topic) -> None:
        """
        Constructs a new instance.

        :param      input_topic:  Subscription topic
        :type       input_topic:  str
        """
        super().__init__(input_topic)
        if hasattr(input_topic, "fixed"):
            if os.path.isfile(input_topic.fixed):
                try:
                    with open(input_topic.fixed, "rb") as wavfile:
                        self.msg = wavfile.read()
                except Exception as e:
                    get_logger(self.node_name).error(
                        f"Exception occured: {e}. Fixed path {input_topic.fixed} provided for Audio topic is not readable wav file"
                    )
            else:
                get_logger(self.node_name).error(
                    f"Fixed path {input_topic.fixed} provided for Audio topic is not a valid file path"
                )

    def _get_output(self, **_) -> Optional[bytes]:
        """
        Gets the audio as byte array.
        :returns:   The output.
        :rtype:     { return_type_description }
        """
        if not self.msg:
            return None
        # send multibyte array from fixed file if it exists
        if isinstance(self.msg, bytes):
            return self.msg
        else:
            # read ROS topic data into a byte array
            audio = bytes()
            audio = b"".join(self.msg.data)

            ### uncomment for testing audio reception - requires the wave package ###
            # import wave
            # with wave.open('./test' + '.wav', 'wb') as file:
            #     file.setnchannels(1)
            #     file.setsampwidth(2)
            #     file.setframerate(16000)
            #     file.writeframes(audio)

            return audio


class MapMetaDataCallback(GenericCallback):
    """
    OccupancyGrid MetaData Callback class. Its get method returns dict of meta data from the occupancy grid topic
    """

    def __init__(self, input_topic) -> None:
        """
        Constructs a new instance.

        :param      input_topic:  Subscription topic
        :type       input_topic:  str
        """
        super().__init__(input_topic)
        self.msg = input_topic.fixed if hasattr(input_topic, "fixed") else None

    def _get_output(self, **_) -> Optional[Dict]:
        """
        Get map metadata
        """

        if not self.msg:
            return None
        # send fixed dict of map metadata if it exists
        if isinstance(self.msg, Dict):
            return self.msg
        # send map metadata from ROS message
        else:
            return {
                "resolution": self.msg.resolution,
                "width": self.msg.width,
                "height": self.msg.height,
            }


class OdomCallback(GenericCallback):
    """
    Ros Odometry Callback Handler to get the robot state in 2D
    """

    def __init__(
        self,
        input_topic,
        node_name: Optional[str] = None,
    ) -> None:
        super().__init__(input_topic, node_name)
        self.__tf: Optional[TransformStamped] = None

    @property
    def transformation(self) -> Optional[TransformStamped]:
        """
        Sets a new transformation value

        :return:  Detected transform from source to desired goal frame
        :rtype: TransformStamped
        """
        return self.__tf

    @transformation.setter
    def transformation(self, transform: TransformStamped) -> None:
        """
        Sets a new transformation value

        :param transform: Detected transform from source to desired goal frame
        :type transform: TransformStamped
        """
        self.__tf = transform

    def _get_output(self, **_) -> Optional[np.ndarray]:
        """
        Get position data.
        """
        if not self.msg:
            return None
        # send fixed position array if it exists
        if isinstance(self.msg, np.ndarray):
            return self.msg

        # If a transform is given apply it to the message
        if self.transformation:
            odom_transformed = self._transform(self.msg, self.transformation)
            return self._process(odom_transformed)

        # send position data from ROS message
        return self._process(self.msg)

    def _process(self, msg: Odometry) -> np.ndarray:
        """Takes Odometry ROS object and converts it to a numpy array with [x, y, z, heading, speed]

        :param msg: Input ROS odomtery message
        :type msg: Odometry

        :return: [x, y, z, heading, speed]
        :rtype: np.ndarray
        """
        heading = 2 * np.arctan2(
            msg.pose.pose.orientation.z, msg.pose.pose.orientation.w
        )

        speed = np.sqrt(msg.twist.twist.linear.y**2 + msg.twist.twist.linear.y**2)

        position = msg.pose.pose.position

        return np.array([position.x, position.y, position.z, heading, speed])

    def _transform(self, msg: Odometry, transform: TransformStamped) -> Odometry:
        """
        Applies a transform to a given Odometry message

        :param msg: Odometry message in source frame
        :type msg: Odometry
        :param transform: Odometry transform from current to goal frame
        :type transform: TransformStamped

        :return: Odometry data in new frame
        :rtype: Odometry
        """
        odom_in_map_pose_data = Odometry()
        odom_in_map_pose_data.pose.pose.position = transform.transform.translation
        odom_in_map_pose_data.pose.pose.orientation = transform.transform.rotation

        odom_in_goal_pose_data: Odometry = utils.odom_from_frame1_to_frame2(
            pose_1_in_2=odom_in_map_pose_data, pose_target_in_1=msg
        )
        return odom_in_goal_pose_data


class PointCallback(GenericCallback):
    """
    Ros Pose Callback Handler to get the robot state in 2D
    """

    def __init__(
        self,
        input_topic,
        node_name: Optional[str] = None,
    ) -> None:
        super().__init__(input_topic, node_name)

    def _get_output(self, **_) -> Optional[np.ndarray]:
        """
        Gets the point message as a numpy array

        :returns:   [x, y, z]
        :rtype:     Optional[np.ndarray]
        """
        if not self.msg:
            return None

        return np.array([self.msg.x, self.msg.y, self.msg.z])


class PointStampedCallback(GenericCallback):
    """
    Ros Pose Callback Handler to get the robot state in 2D
    """

    def __init__(
        self,
        input_topic,
        node_name: Optional[str] = None,
    ) -> None:
        super().__init__(input_topic, node_name)

    def _get_output(self, **_) -> Optional[np.ndarray]:
        """
        Gets the point stamped message as a numpy array

        :returns:   [x, y, z]
        :rtype:     Optional[np.ndarray]
        """
        if not self.msg:
            return None

        return np.array([self.msg.point.x, self.msg.point.y, self.msg.point.z])


class PoseCallback(GenericCallback):
    """
    Ros Pose Callback Handler to get the robot state in 2D
    """

    def __init__(
        self,
        input_topic,
        node_name: Optional[str] = None,
    ) -> None:
        super().__init__(input_topic, node_name)
        self.__tf: Optional[TransformStamped] = None

    @property
    def transformation(self) -> Optional[TransformStamped]:
        """
        Sets a new transformation value

        :return:  Detected transform from source to desired goal frame
        :rtype: TransformStamped
        """
        return self.__tf

    @transformation.setter
    def transformation(self, transform: TransformStamped) -> None:
        """
        Sets a new transformation value

        :param transform: Detected transform from source to desired goal frame
        :type transform: TransformStamped
        """
        self.__tf = transform

    def _process(self, msg: Pose) -> np.ndarray:
        """Takes Pose ROS object and converts it to a numpy array with [x, y, z, heading]

        :param msg: Input ROS Pose message
        :type msg: Pose

        :return: [x, y, z, heading]
        :rtype: np.ndarray
        """
        heading = 2 * np.arctan2(msg.orientation.z, msg.orientation.w)

        position = msg.position

        return np.array([position.x, position.y, position.z, heading])

    def _transform(self, msg: Pose, transform: TransformStamped) -> Pose:
        """
        Applies a transform to a given Odometry message

        :param msg: Pose message in source frame
        :type msg: Pose
        :param transform: Pose transform from current to goal frame
        :type transform: TransformStamped

        :return: Pose data in new frame
        :rtype: Pose
        """
        odom_in_map_pose_data = Odometry()
        odom_in_map_pose_data.pose.pose.position = transform.transform.translation
        odom_in_map_pose_data.pose.pose.orientation = transform.transform.rotation

        msg_as_odom = Odometry()
        msg_as_odom.pose.pose.position = msg.position
        msg_as_odom.pose.pose.orientation = msg.orientation

        odom_in_goal_pose_data: Odometry = utils.odom_from_frame1_to_frame2(
            pose_1_in_2=odom_in_map_pose_data, pose_target_in_1=msg_as_odom
        )
        output = Pose()
        output.position = odom_in_goal_pose_data.pose.pose.position
        output.orientation = odom_in_goal_pose_data.pose.pose.orientation

        return output

    def _get_output(self, **_) -> Optional[np.ndarray]:
        """
        Gets the Pose message as a numpy array

        :returns:   [x, y, z, heading]
        :rtype:     Optional[np.ndarray]
        """
        if not self.msg:
            return None

        # If a transform is given apply it to the message
        if self.transformation:
            pose_transformed = self._transform(self.msg, self.transformation)
            return self._process(pose_transformed)

        # send position data from ROS message
        return self._process(self.msg)


class PoseStampedCallback(PoseCallback):
    """
    Ros PoseStamped Callback Handler to get the robot state in 2D
    """

    def __init__(
        self,
        input_topic,
        node_name: Optional[str] = None,
    ) -> None:
        super().__init__(input_topic, node_name)

    def _get_output(self, **_) -> Optional[np.ndarray]:
        """
        Gets the PoseStamped message as a numpy array

        :returns:   [x, y, z, heading]
        :rtype:     Optional[np.ndarray]
        """
        if not self.msg:
            return None

        # If a trnsform is given apply it to the message
        if self.transformation:
            pose_transformed = self._transform(self.msg.pose, self.transformation)
            return self._process(pose_transformed)

        # send position data from ROS message
        return self._process(self.msg.pose)


class OccupancyGridCallback(GenericCallback):
    def __init__(
        self,
        input_topic,
        node_name: Optional[str] = None,
        to_numpy: Optional[bool] = True,
        twoD_to_threeD_conversion_height: Optional[float] = 0.01,
    ) -> None:
        super().__init__(input_topic, node_name)
        self.__to_numpy = to_numpy
        self.__twoD_to_threeD_conversion_height = twoD_to_threeD_conversion_height

    def _get_output(
        self, get_metadata: bool = False, **_
    ) -> Optional[Union[OccupancyGrid, np.ndarray, Dict]]:
        """
        Gets the OccupancyGrid message raw or as a numpy array

        :returns:   Map as an OccupancyGrid or numpy array
        :rtype:     Optional[Union[OccupancyGrid, np.ndarray]]
        """
        if not self.msg:
            return None

        origin_x = self.msg.info.origin.position.x
        origin_y = self.msg.info.origin.position.y
        origin_yaw = 2 * np.arctan2(
            self.msg.info.origin.orientation.z, self.msg.info.origin.orientation.w
        )

        # Get only the map meta data from the message
        if get_metadata:
            return {
                "resolution": self.msg.info.resolution,
                "width": self.msg.info.width,
                "height": self.msg.info.height,
                "origin_x": origin_x,
                "origin_y": origin_y,
                "origin_yaw": origin_yaw,
            }

        # If conversion to numpy is set to False return the ROS message as it is
        if not self.__to_numpy:
            return self.msg

        # Convert to a numpy array with occupied cells
        occupied_cells = []

        for y in range(self.msg.info.height):
            for x in range(self.msg.info.width):
                index = x + y * self.msg.info.width
                occupancy_value = self.msg.data[index]
                if occupancy_value != 0:  # Occupied and unknown cells
                    # Convert grid coordinates to world coordinates
                    map_x = (x + 0.5) * self.msg.info.resolution
                    map_y = (y + 0.5) * self.msg.info.resolution

                    # Apply the rotation and translation to get world coordinates
                    world_x = (
                        origin_x
                        + (map_x * np.cos(origin_yaw))
                        - (map_y * np.sin(origin_yaw))
                    )
                    world_y = (
                        origin_y
                        + (map_x * np.sin(origin_yaw))
                        + (map_y * np.cos(origin_yaw))
                    )

                    occupied_cells.append((
                        world_x,
                        world_y,
                        self.__twoD_to_threeD_conversion_height,
                    ))

        occupied_cells_np = np.array(occupied_cells)

        return occupied_cells_np
