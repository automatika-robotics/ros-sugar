"""ROS Publishers"""

from typing import Any, Callable, Optional
from rclpy.logging import get_logger
from rclpy.publisher import Publisher as ROSPublisher


class Publisher:
    """Publisher."""

    def __init__(self, output_topic, node_name: Optional[str] = None) -> None:
        """__init__.

        :param input_topic:
        :type topic: Input
        :rtype: None
        """

        self.output_topic = output_topic

        # Node name can be changed to a node that the callback is executed in
        # at the time of setting subscriber using set_node_name
        self.node_name: Optional[str] = node_name

        self._publisher: Optional[ROSPublisher] = None
        self._pre_processors: Optional[list[Callable]] = None

    def set_node_name(self, node_name: str) -> None:
        """Set node name.

        :param node_name:
        :type node_name: str
        :rtype: None
        """
        self.node_name = node_name

    def set_publisher(self, publisher: ROSPublisher) -> None:
        """set_publisher.

        :param publisher: Publisher
        :rtype: None
        """
        self._publisher = publisher

    def add_pre_processor(self, method: Callable):
        """Add a pre processor for publisher message

        :param method: Pre processor method
        :type method: Callable
        """
        if not self._pre_processors:
            self._pre_processors = [method]
        else:
            self._pre_processors.append(method)

    def publish(self, output: Any, *args, **kwargs) -> None:
        """
        Publish using the publisher

        :param output: ROS message to publish
        :type output: Any
        """
        # Apply any output pre_processors sequentially before publishing, if defined
        if self._publisher:
            if self._pre_processors:
                for processor in self._pre_processors:
                    pre_output = processor(output)
                    # if any processor output is None, then dont publish
                    if pre_output is None:
                        return None
                    # type check processor output if incorrect, raise an error
                    if type(pre_output) is not type(output):
                        get_logger(self.node_name).warn(
                            f"The output produced by the component for topic {self.output_topic.name} is of type {type(output).__name__}. Got pre_processor output of type {type(pre_output).__name__}"
                        )
                    # if all good, set output equal to post output
                    output = pre_output
            msg = self.output_topic.msg_type.convert(output, *args, **kwargs)
            if msg:
                self._publisher.publish(msg)
