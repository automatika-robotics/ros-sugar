from typing import Callable, List, Optional, Union

import rclpy
from builtin_interfaces.msg import Time
from rclpy import callback_groups
from rclpy.client import Client
from rclpy.node import Node
from tf2_ros.transform_listener import TransformListener

from ..config import BaseConfig, QoSConfig
from ..tf import TFListener, TFListenerConfig


class BaseNode(Node):
    """BaseNode."""

    def __init__(
        self,
        node_name: str,
        node_config: Optional[BaseConfig] = None,
        callback_group=None,
        start_on_init: bool = True,
        *args,
        **kwargs,
    ):
        """
        Base Node

        :param node_name: Name of the node
        :type node_name: str
        :param node_config: Node parameters
        :type node_config: BaseConfig Or a Custom config class inhering from BaseConfig
        :param callback_group: Callback group used for the executor
        :type callback_group: ReentrantCallbackGroup | MutuallyExclusiveCallbackGroup
        :param lifecycle_node: Used to disable node activation on init in lifecycle nodes, defaults to True
        :type lifecycle_node: bool, optional
        """
        # SET NAME AND CALLBACK GROUP
        self.node_name = node_name

        if not callback_group:
            callback_group = callback_groups.ReentrantCallbackGroup()

        self.callback_group = callback_group

        if not node_config:
            node_config = BaseConfig()

        self.config = node_config

        # List to keep all node clients
        self.clients_list = []

        # Start Node
        if start_on_init:
            Node.__init__(self, node_name, *args, **kwargs)

            # Activate the base node
            self.activate()

        # Setup the launch command-line arguments list
        self._cmd_line_kwargs_list = []

    def rclpy_init_node(self, *args, **kwargs):
        """
        To init the node with rclpy and activate default services
        """
        Node.__init__(self, self.node_name, *args, **kwargs)
        self.get_logger().info(f"NODE {self.get_name()} STARTED")

    def configure(self, config_file: str):
        """
        Configure component from yaml file

        :param config_file: Path to file
        :type config_file: str
        """

        self.config.from_yaml(
            config_file, nested_root_name=self.node_name, get_common=True
        )

    def _execution_step(self):
        """
        Main execution of the component, executed at each timer tick with rate 'loop_rate' from config
        """
        pass

    def _execute_once(self):
        """
        Executed once when the component is started
        """
        pass

    def add_execute_once(self, method: Callable):
        """
        Add method to be executed once when the component is started

        :param method: Callable to be executed
        :type method: Callable
        """
        self._extra_execute_once = method

    def add_execute_in_loop(self, method: Callable):
        """
        Add method to be executed each loop_step in the component

        :param method: Callable to be executed
        :type method: Callable
        """
        self._extra_execute_loop = method

    def get_ros_time(self) -> Time:
        """
        Helper method to get ROS time from the node

        :return: ROS time now
        :rtype: Time
        """
        return self.get_clock().now().to_msg()

    def get_secs_time(self) -> float:
        """
        Gets the current ROS time as float in seconds

        :param node: ROS node
        :type node: Node

        :return: ROS time as float
        :rtype: float
        """
        ros_time = self.get_ros_time()
        return float(ros_time.sec + 1e-9 * ros_time.nanosec)

    @property
    def launch_cmd_args(self) -> List[str]:
        """
        List of command line arguments

        :return: ROS launch command line arguments
        :rtype: List[str]
        """
        return self._cmd_line_kwargs_list

    @launch_cmd_args.setter
    def launch_cmd_args(self, values: List):
        """launch_cmd_args.

        :param values:
        :type values: List
        """
        try:
            for i, val in enumerate(values):
                if val.startswith("--"):
                    if val not in self._cmd_line_kwargs_list:
                        # Add new value to the list
                        self._cmd_line_kwargs_list.append(val)
                        self._cmd_line_kwargs_list.append(str(values[i + 1]))
                    else:
                        # Update an existing value
                        idx = self._cmd_line_kwargs_list.index(val)
                        self._cmd_line_kwargs_list[idx + 1] = str(values[i + 1])
        except IndexError as e:
            raise IndexError("Launch commands require more arguments to update") from e

    # TO USE FOR ROS LAUNCH
    @property
    def config_json(self) -> Union[str, bytes]:
        """
        Component config as a json string

        :return: Config json
        :rtype: str
        """
        return self.config.to_json()

    @config_json.setter
    def config_json(self, value: str):
        """
        Component config from json string

        :param value: Config json
        :type value: str
        """
        self.config.from_json(value)

    # Node transitions/ init
    def activate(self):
        """
        Create required subscriptions, publications, timers, ... etc. to activate the node
        """
        # Declare and init any flags used to track the node functionalities
        self.init_flags()

        # Init any global node variables
        self.init_variables()

        # Setup node publishers and subscribers
        self.create_all_subscribers()

        self.create_all_publishers()

        # Setup node services: servers and clients
        self.create_all_services()

        self.create_all_service_clients()

        # Setup node actions: servers and clients
        self.create_all_action_servers()

        self.create_all_action_clients()

        # Setup node timers
        self.create_all_timers()

    def deactivate(self):
        """
        Destroy all declared subscriptions, publications, timers, ... etc. to deactivate the node
        """
        self.destroy_all_action_servers()

        self.destroy_all_services()

        self.destroy_all_subscribers()

        self.destroy_all_publishers()

        self.destroy_all_timers()

        self.destroy_all_action_clients()

        self.destroy_all_service_clients()

    def setup_qos(self, qos_policy: QoSConfig) -> rclpy.qos.QoSProfile:
        """
        Setup QoS profile from given QoSConfig

        :param qos_policy: QoS policy config parameters
        :type qos_policy: QoSConfig

        :return: QoS Profile to use in ros publishers/subscribers
        :rtype: rclpy.qos.QoSProfile
        """
        return rclpy.qos.QoSProfile(
            reliability=qos_policy.reliability,
            history=qos_policy.history,
            depth=qos_policy.queue_size,
            durability=qos_policy.durability,
        )

    def init_flags(self):
        """
        Set up node flags
        """
        pass

    def init_variables(self):
        """
        Set up node variables
        """
        pass

    def create_tf_listener(self, tf_config: TFListenerConfig) -> TFListener:
        """
        Creates a new transform listener to lookup a transform with given config and return the transform lookup handler

        :param tf_config: Transform listener config
        :type tf_config: TFListenerConfig

        :return: Transform lookup handler object
        :rtype: TransformListener
        """
        tf_handler = TFListener(tf_config=tf_config, node_name=self.node_name)
        transform_listener = TransformListener(buffer=tf_handler.tf_buffer, node=self)
        tf_handler.set_listener(transform_listener)
        transform_timer = self.create_timer(
            1 / tf_config.lookup_rate, tf_handler.timer_callback
        )  # timer to lookup the transform with given rate
        tf_handler.timer = transform_timer
        return tf_handler

    def create_client(self, *args, **kwargs) -> Client:
        """
        Overwrites the Node create client method to add to the clients list

        :return: ROS service client
        :rtype: rclpy.client.Client
        """
        _new_client = super().create_client(*args, **kwargs)
        if hasattr(self, "clients_list"):
            self.clients_list.append(_new_client)
        return _new_client

    def create_all_subscribers(self):
        """
        Creates all node subscribers
        """
        pass

    def create_all_publishers(self):
        """
        Creates all node publishers
        """
        pass

    def create_all_services(self):
        """
        Creates all node service servers
        """
        pass

    def create_all_service_clients(self):
        """
        Creates all node service clients
        """
        pass

    def create_all_action_servers(self):
        """
        Creates all node action servers
        """
        pass

    def create_all_action_clients(self):
        """
        Creates all node action clients
        """
        pass

    def create_all_timers(self):
        """
        Creates all node timers
        """
        pass

    def destroy_all_subscribers(self):
        """
        Destroys all node subscribers
        """
        pass

    def destroy_all_publishers(self):
        """
        Destroys all node publishers
        """
        pass

    def destroy_all_services(self):
        """
        Destroys all node services
        """
        pass

    def destroy_all_action_servers(self):
        """
        Destroys all action servers
        """
        pass

    def destroy_all_action_clients(self):
        """
        Destroys all action clients
        """
        pass

    def destroy_all_service_clients(self):
        """destroy_all_service_clients."""
        pass

    def destroy_all_timers(self):
        """destroy_all_timers."""
        pass
