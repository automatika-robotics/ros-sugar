"""ROS Service/Action Client Wrapper"""

import time as rostime
from typing import Any, Optional

import rclpy
from attrs import Factory, define, field
from rclpy.action.client import ActionClient
from rclpy.callback_groups import CallbackGroup, ReentrantCallbackGroup
from rclpy.executors import Executor

from .config import BaseAttrs, base_validators
from .core import BaseNode


@define
class ServiceClientConfig(BaseAttrs):
    """
    Basic configuration for any ROS service client
    """

    srv_type: type = field()
    name: str = field()
    timeout_secs: float = field(
        default=1.0, validator=base_validators.in_range(min_value=1e-9, max_value=1e9)
    )  # timeout after calling the service
    attempt_period_secs: float = field(
        default=0.1, validator=base_validators.in_range(min_value=1e-9, max_value=1e9)
    )  # time period to attempt to call the service again


@define
class ActionClientConfig(BaseAttrs):
    """
    Basic configuration for any ROS action client
    """

    action_type: type = field()
    name: str = field()
    timeout_secs: float = field(
        default=1.0, validator=base_validators.in_range(min_value=1e-9, max_value=1e9)
    )  # timeout after calling the action
    attempt_period_secs: float = field(
        default=0.1, validator=base_validators.in_range(min_value=1e-9, max_value=1e9)
    )  # time period to attempt to call the action again
    feedback_check_period: float = field(
        default=0.05, validator=base_validators.in_range(min_value=1e-9, max_value=1e9)
    )  # time period to check for the action feedback
    feedback_check_timeout: float = field(
        default=5.0, validator=base_validators.in_range(min_value=1e-9, max_value=1e9)
    )  # timeout if feedback is not recieved after x seconds
    callback_group: CallbackGroup = field(
        default=Factory(ReentrantCallbackGroup)
    )  # callback group for the feedback callback of the action


class ServiceClientHandler:
    """
    General purpose service client class
    """

    def __init__(
        self,
        client_node: BaseNode,
        config: Optional[ServiceClientConfig] = None,
        srv_name: Optional[str] = None,
        srv_type: Optional[type] = None,
    ) -> None:
        """
        Init the client

        :param client_node: ROS node used to run the client
        :type client_node: Node
        :param config: Service client configuration
        :type srv_name: ServiceClientConfig

        """
        if not config and (srv_name and srv_type):
            config = ServiceClientConfig(name=srv_name, srv_type=srv_type)
        else:
            raise ValueError(
                "Cannot initialize service client. Provide a valid config or a valid service name and service type"
            )

        # If config is provided plus additional name or type -> update name or type
        if srv_name:
            config.name = srv_name

        if srv_type:
            config.srv_type = srv_type

        self.config = config
        self.node = client_node
        self.node.get_logger().debug(
            f"creating client for {self.config.name} of type {self.config.srv_type}"
        )
        self.client = self.node.create_client(self.config.srv_type, self.config.name)

    def send_request(self, req_msg, executor: Optional[Executor] = None):
        """
        Sends a request to the service returns the response
        In case of failure, the method attempts sending the request again multiple time according to the given config

        :param req_msg: Service request msg
        :type req_msg: Any
        :return: Service result
        :rtype: Any
        """
        _timeout_count: float = 0.0  # timeout counter

        # Check if the service is available every attempt_period_secs
        while not self.client.wait_for_service(
            timeout_sec=self.config.attempt_period_secs
        ):
            # If the service is not available give warning
            self.node.get_logger().warn(
                f"Service {self.config.name} not available, Waiting... timeout in {(self.config.timeout_secs - _timeout_count):.2f} secs"
            )
            _timeout_count += self.config.attempt_period_secs

            # Check for service request timeout
            if _timeout_count > self.config.timeout_secs:
                self.node.get_logger().warn(
                    f"Service {self.config.name} is not available, Cancelling"
                )
                return None

        # Service is available
        self.node.get_logger().debug(
            f"Service {self.config.name} is available, Sending request..."
        )

        # Check request type
        if not isinstance(req_msg, self.config.srv_type.Request):
            self.node.get_logger().error(
                f"Invalid request message for service '{self.config.name}'. Service takes request message of type '{self.config.srv_type.Request}', got '{type(req_msg)}'"
            )
            return None

        # send request
        self.request = req_msg
        self.future = self.client.call_async(self.request)

        # Spin until response
        if executor:
            while not self.future:
                rclpy.spin_once(
                    self.node, executor=executor, timeout_sec=self.node.config.loop_rate
                )
        else:
            rclpy.spin_until_future_complete(self.node, self.future)

        # return response
        return self.future.result()


class ActionClientHandler:
    """
    General purpose action client class
    """

    def __init__(
        self,
        client_node: BaseNode,
        config: Optional[ActionClientConfig] = None,
        action_name: Optional[str] = None,
        action_type: Optional[type] = None,
    ):
        """
        Init an action client handler

        :param client_node: ROS node using the client
        :type client_node: rclpy.node.Node
        :param config: Client config
        :type config: ActionClientConfig
        """
        self.reset()

        if not config and (action_name and action_type):
            config = ActionClientConfig(name=action_name, action_type=action_type)
        else:
            raise ValueError(
                "Cannot initialize action client. Provide a valid config or a valid action name and action type"
            )

        # If config is provided plus additional name or type -> update name or type
        if action_name:
            config.name = action_name

        if action_type:
            config.action_type = action_type

        self.config = config
        self.node = client_node
        self.client = ActionClient(
            self.node,
            self.config.action_type,
            self.config.name,
            callback_group=self.config.callback_group,
        )

    def reset(self):
        """
        Resst the client handler
        """
        self.old_feedback_count: int = -1
        self.feedback_count: int = 0
        self.goal_rejcted = False
        self.goal_accepted = False
        self.action_returned = False

    def send_request(
        self, request_msg: Any, wait_until_first_feedback: bool = True
    ) -> bool:
        """
        Sends a request to an action server

        :param request_msg: Action request message
        :type request_msg: Action_Type.Goal
        :param wait_until_first_feedback: Wait until the server returns its first feedback, defaults to True
        :type wait_until_first_feedback: bool, optional

        :return: If action server is available
        :rtype: bool
        """

        # Making request to the server
        _path_timeout_count: float = 0.0
        # Wait until the server is available
        while not self.client.wait_for_server(
            timeout_sec=self.config.attempt_period_secs
        ):
            self.node.get_logger().warning(
                "Waiting for Server node to become available...", once=True
            )

            _path_timeout_count += self.config.attempt_period_secs

            # timeout in attempt_period_secs
            if _path_timeout_count > self.config.timeout_secs:
                self.node.get_logger().error(
                    "Server node is not available - cannot start action service"
                )
                return False

        self.node.get_logger().info(f"Sending request to {self.config.name}")

        # Check request type
        if not isinstance(request_msg, self.config.action_type.Goal):
            self.node.get_logger().error(
                f"Invalid request message for action '{self.config.name}'. Service takes request message of type '{self.config.action_type.Goal}', got '{type(request_msg)}'"
            )
            return False

        # If available, send request and get future response, and feedback callback method
        self._send_goal_future = self.client.send_goal_async(
            request_msg, feedback_callback=self.action_feedback_callback
        )

        # Wait until the action returns the first feedback
        while wait_until_first_feedback and self.feedback_count <= 0:
            self.node.get_logger().warn("Waiting for action feedback", once=True)
            if not self.got_new_feedback():
                return False
            pass

        # Add method when action is done
        self._send_goal_future.add_done_callback(self.action_response_callback)

        return True

    # METHOD WHEN ACTION IS DONE
    def action_response_callback(self, future):
        """
        Callback when getting the action server responses

        :param future: Action result future
        :type future: Any
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.goal_rejcted = True
            return
        self.goal_accepted = True

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.action_result_callback)
        return

    # METHOD TO GET THE RESULT WHEN DONE
    def action_result_callback(self, future):
        """
        Treats the path tracker action result

        :param future: Action result future
        :type future: Any
        """
        self.action_result = future.result().result
        self.action_returned = True

    def action_feedback_callback(self, feedback_msg: Any):
        """
        Method to be called at each action feedback

        :param feedback_msg: Action feedback message
        :type feedback_msg: Any
        """
        # Increase the feedback counter
        self.feedback_count += 1
        self.feedback_msg = feedback_msg

    def got_new_feedback(self) -> bool:
        """
        Checks if the client got a new feedback from the server within a specified time limit

        :return: Feedback updated on time
        :rtype: bool
        """
        # if did not get back wait and check
        _check_counter: float = 0.0
        while _check_counter < self.config.feedback_check_timeout:
            if self.feedback_count > self.old_feedback_count:
                self.old_feedback_count = self.feedback_count
                return True
            _check_counter += self.config.feedback_check_period
            rostime.sleep(self.config.feedback_check_period)
        return False
