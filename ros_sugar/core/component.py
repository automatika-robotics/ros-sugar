"""Base Component"""

import os
import time
import json
import socket
from abc import abstractmethod
from typing import Any, Dict, List, Optional, Union, Callable, Sequence, Tuple

from rclpy.action.server import ActionServer, CancelResponse, GoalResponse
from rclpy.utilities import try_shutdown
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy import lifecycle
from rclpy.publisher import Publisher as ROSPublisher
from rclpy.subscription import Subscription
from ros_sugar_interfaces.msg import ComponentStatus
from ros_sugar_interfaces.srv import (
    ChangeParameter,
    ChangeParameters,
    ConfigureFromYaml,
    ReplaceTopic,
)

from .action import Action
from .event import Event
from ..events import json_to_events_list
from ..io.callbacks import GenericCallback
from ..config.base_config import BaseComponentConfig, ComponentRunType
from ..io.topic import Topic
from .fallbacks import ComponentFallbacks, Fallback
from .node import BaseNode
from .status import Status
from ..utils import (
    camel_to_snake_case,
    component_action,
    component_fallback,
    get_methods_with_decorator,
    log_srv,
)
from ..io.publisher import Publisher


class BaseComponent(BaseNode, lifecycle.Node):
    def __init__(
        self,
        component_name: str,
        inputs: Optional[Sequence[Topic]] = None,
        outputs: Optional[Sequence[Topic]] = None,
        config: Optional[BaseComponentConfig] = None,
        config_file: Optional[str] = None,
        callback_group=None,
        enable_health_broadcast: bool = True,
        fallbacks: Optional[ComponentFallbacks] = None,
        main_action_type: Optional[type] = None,
        main_srv_type: Optional[type] = None,
        **kwargs,
    ):
        """Initialize a component

        :param component_name: ROS2 node name
        :type component_name: str
        :param inputs: Component input topics, defaults to None
        :type inputs: Optional[Sequence[Topic]], optional
        :param outputs: Component output topics, defaults to None
        :type outputs: Optional[Sequence[Topic]], optional
        :param config: Component config, defaults to None
        :type config: Optional[BaseComponentConfig], optional
        :param config_file: Path to YAML configuration file, defaults to None
        :type config_file: Optional[str], optional
        :param callback_group: Main callback group, defaults to None
        :type callback_group: _type_, optional
        :param enable_health_broadcast: Enable publishing the component health status, defaults to True
        :type enable_health_broadcast: bool, optional
        :param fallbacks: Component fallbacks, defaults to None
        :type fallbacks: Optional[ComponentFallbacks], optional
        :param main_action_type: Component main ROS2 action server type (Used when the component is running as an ActionServer), defaults to None
        :type main_action_type: Optional[type], optional
        :param main_srv_type: Component main ROS2 service type (Used when the component is running as a Server), defaults to None
        :type main_srv_type: Optional[type], optional
        """
        # Setup Config
        self.config: BaseComponentConfig = config or BaseComponentConfig()

        # Component health status - Inits with healthy status
        self.health_status = Status()
        self.__enable_health_publishing = enable_health_broadcast

        callback_group = callback_group or ReentrantCallbackGroup()

        BaseNode.__init__(
            self,
            node_name=component_name,
            node_config=config,
            callback_group=callback_group,
            start_on_init=False,
            **kwargs,
        )

        # setup inputs and outputs
        self.callbacks: Dict[str, GenericCallback] = {}
        if inputs:
            self.in_topics = inputs
            self.callbacks = {
                input.name: input.msg_type.callback(input) for input in self.in_topics
            }

        self.publishers_dict: Dict[str, Publisher] = {}
        if outputs:
            self.out_topics = outputs
            self.publishers_dict = {
                output.name: Publisher(output, node_name=self.node_name)
                for output in self.out_topics
            }

        if config_file:
            self._config_file = config_file
        else:
            self._config_file = None

        # NOTE: Default fallback for any failure is set to broadcast status and max retries to None (i.e. component keeps executing on_any_fail action)
        if not fallbacks:
            fallbacks = ComponentFallbacks(
                on_any_fail=Fallback(
                    action=Action(method=self.broadcast_status), max_retries=None
                )
            )
        self.__fallbacks = fallbacks
        self.__fallbacks_giveup: bool = False

        if self.config.use_without_launcher:
            # Create default services for changing config/inputs/outputs during runtime
            self._create_default_services()

        self.action_type = main_action_type
        self.service_type = main_srv_type
        self._external_processors: Dict[
            str, Tuple[List[Union[Callable, socket.socket]], str]
        ] = {}

        self.__events: Optional[List[Event]] = None
        self.__actions: Optional[List[List[Action]]] = None
        self.__event_listeners: List[Subscription] = []

        # To use without launcher -> Init the ROS2 node directly
        if self.config.use_without_launcher:
            self.rclpy_init_node(component_name, **kwargs)

    def rclpy_init_node(self, *args, **kwargs):
        """
        To init the node with rclpy and activate default services
        """
        lifecycle.Node.__init__(self, self.node_name, *args, **kwargs)
        self.get_logger().info(
            f"LIFECYCLE NODE {self.get_name()} STARTED AND REQUIRES CONFIGURATION"
        )
        self._create_default_services()

    # Managing Inputs/Outputs
    def _add_ros_subscriber(self, callback: GenericCallback):
        """Creates a subscriber to be attached to an input message.

        :param msg:
        :type msg: Input
        :param callback:
        :type callback: GenericCallback
        """
        _subscriber = self.create_subscription(
            msg_type=callback.input_topic.ros_msg_type,
            topic=callback.input_topic.name,
            qos_profile=self.setup_qos(callback.input_topic.qos_profile),
            callback=callback.callback,
            callback_group=self.callback_group,
        )
        self.get_logger().debug(
            f"Started subscriber to topic: {callback.input_topic.name} of type {callback.input_topic.msg_type}"
        )
        return _subscriber

    def _add_ros_publisher(self, publisher: Publisher) -> ROSPublisher:
        """
        Sets the publisher attribute of a component for a given Topic
        """
        qos_profile = self.setup_qos(publisher.output_topic.qos_profile)
        return self.create_publisher(
            publisher.output_topic.ros_msg_type,
            publisher.output_topic.name,
            qos_profile,
        )

    def attach_custom_callback(self, input_topic: Topic, callable: Callable) -> None:
        """
        Method to attach custom method to subscriber callbacks
        """
        if not callable(callable):
            raise TypeError("A custom callback must be a Callable")
        if callback := self.callbacks.get(input_topic.name):
            if not callback:
                raise TypeError("Specified input topic does not exist")
            callback.on_callback_execute(callable)

    def add_callback_postprocessor(self, input_topic: Topic, func: Callable) -> None:
        """Adds a callable as a post processor for topic callback.
        :param input_topic:
        :type input_topic: Topic
        :param callable:
        :type func: Callable
        """
        if not callable(func):
            raise TypeError(
                "A postprocessor must be a Callable with input and output types the same as the topic."
            )
        if callback := self.callbacks.get(input_topic.name):
            if not callback:
                raise TypeError("Specified input topic does not exist")

            if self._external_processors.get(input_topic.name):
                self._external_processors[input_topic.name][0].append(func)
            else:
                self._external_processors[input_topic.name] = ([func], "postprocessor")

    def add_publisher_preprocessor(self, output_topic: Topic, func: Callable) -> None:
        """Adds a callable as a pre processor for topic publisher.
        :param output_topic:
        :type output_topic: Topic
        :param callable:
        :type func: Callable
        """
        if not callable(func):
            raise TypeError(
                "A preprocessor must be a Callable with input and output types the same as the topic."
            )
        if self.publishers_dict:
            if publisher := self.publishers_dict.get(output_topic.name):
                if not publisher:
                    raise TypeError("Specified output topic does not exist")
            if self._external_processors.get(output_topic.name):
                self._external_processors[output_topic.name][0].append(func)
            else:
                self._external_processors[output_topic.name] = ([func], "preprocessor")
        else:
            raise TypeError(
                "The component does not have any output topics specified. Add output topics with Component.outputs method"
            )

    # CREATION AND DESTRUCTION METHODS
    def create_all_subscribers(self):
        """
        Creates all node subscribers from component inputs
        """
        self.get_logger().info("STARTING ALL SUBSCRIBERS")
        # Create subscribers
        for callback in self.callbacks.values():
            callback.set_node_name(self.node_name)
            callback.set_subscriber(self._add_ros_subscriber(callback))

    def create_all_publishers(self):
        """
        Creates all node publishers from component outputs
        """
        self.get_logger().info("STARTING ALL PUBLISHERS")
        if self.__enable_health_publishing:
            # Create status publisher
            self.health_status_publisher: ROSPublisher = self.create_publisher(
                msg_type=ComponentStatus,
                topic=f"{self.get_name()}_status",
                qos_profile=1,
            )
        # Create publisher and attach it to output publisher object
        for publisher in self.publishers_dict.values():
            publisher.set_node_name(self.node_name)
            # Set ROS publisher for each output publisher
            publisher.set_publisher(self._add_ros_publisher(publisher))

    def create_all_timers(self):
        """
        Creates all node timers
        """
        # If component is not used as a server start the main execution timer
        if self.run_type != ComponentRunType.TIMED:
            return
        self.get_logger().info("CREATING MAIN TIMER")
        self._execution_timer = self.create_timer(
            timer_period_sec=1 / self.config.loop_rate,
            callback=self._main,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

    def create_all_action_servers(self):
        """
        Action servers creation
        """
        if self.run_type != ComponentRunType.ACTION_SERVER:
            return
        if not self.action_type:
            raise TypeError(
                f"Cannot start component '{self.node_name}' as an ActionServer without specifying 'action_type'"
            )
        action_name = self.main_action_name
        # callback group to avoid parallel execution of action loops
        action_callback_group = MutuallyExclusiveCallbackGroup()
        self.action_server = ActionServer(
            node=self,
            action_type=self.action_type,
            action_name=action_name,
            execute_callback=self.main_action_callback,
            goal_callback=self._main_action_goal_callback,
            handle_accepted_callback=self._main_action_handle_accepted_callback,
            cancel_callback=self._main_action_cancel_callback,
            callback_group=action_callback_group,
        )

    def create_all_services(self):
        """
        Services creation
        """
        if self.run_type != ComponentRunType.SERVER:
            return
        if not self.service_type:
            raise TypeError(
                f"Cannot start component '{self.node_name}' as a Server without specifying 'service_type'"
            )
        srv_name = self.main_srv_name
        if not srv_name:
            raise TypeError(
                f"Cannot start component '{self.node_name}' as a Server without specifying 'service_type'"
            )
        self.server = self.create_service(
            self.service_type,
            srv_name,
            self.main_service_callback,
        )

    def destroy_all_timers(self):
        """
        Destroys all node timers
        """
        if hasattr(self, "_execution_timer"):
            self.get_logger().info("DESTROYING MAIN TIMER")
            self.destroy_timer(self._execution_timer)

    def destroy_all_subscribers(self):
        """
        Destroys all node subscribers
        """
        self.get_logger().info("DESTROYING ALL SUBSCRIBERS")
        for listener in self.__event_listeners:
            self.destroy_subscription(listener)
        # Destroy all input subscribers
        for callback in self.callbacks:
            if callback._subscriber:
                self.destroy_subscription(callback._subscriber)
                callback._subscriber = None

    def destroy_all_publishers(self):
        """
        Destroys all node publishers
        """
        self.get_logger().info("DESTROYING ALL PUBLISHERS")
        if self.__enable_health_publishing:
            # Destroy health status publisher
            self.destroy_publisher(self.health_status_publisher)

        for publisher in self.publishers_dict.values():
            if publisher._publisher:
                self.destroy_publisher(publisher._publisher)

    def destroy_all_services(self):
        """
        Destroys all node services
        """
        # Destroy node main Server if runtype is server
        if self.run_type == ComponentRunType.SERVER:
            self.destroy_service(self.server)

    def destroy_all_action_servers(self):
        """
        Destroys all action servers
        """
        # Destroy node main Server if runtype is action server
        if self.run_type == ComponentRunType.ACTION_SERVER:
            self.action_server.destroy()

    def _turn_on_events_management(self) -> None:
        """
        Turn on event by starting a listener to the event topic

        :param event: Event to be activated
        :type event: Event
        :param qos_profile: Subscriber QoS Profile, defaults to 1
        :type qos_profile: Union[QoSProfile, int], optional
        """
        if not self.__events or not self.__actions:
            return

        self.__event_listeners = []
        for event, actions in zip(self.__events, self.__actions):
            # Register action to event callback to get executed on trigger
            event.register_actions(actions)
            # Create listener to the event trigger topic
            listener = self.create_subscription(
                msg_type=event.event_topic.ros_msg_type,
                topic=event.event_topic.name,
                callback=event.callback,
                qos_profile=self.setup_qos(event.event_topic.qos_profile),
                callback_group=MutuallyExclusiveCallbackGroup(),
            )
            self.__event_listeners.append(listener)

    def got_all_inputs(
        self,
        inputs_to_check: Optional[List[str]] = None,
        inputs_to_exclude: Optional[List[str]] = None,
    ) -> bool:
        """
        Check if all input topics are being published

        :param inputs_to_check: List of input keys to check, defaults to None
        :type inputs_to_check: list[str] | None, optional

        :param inputs_to_exclude: List of input keys to exclude from check, defaults to None
        :type inputs_to_exclude: list[str] | None, optional

        :return: If all inputs are published
        :rtype: bool
        """

        if inputs_to_exclude:
            # If a non valid key is provided raise an error
            if not all(item in self.callbacks.keys() for item in inputs_to_exclude):
                raise ValueError(
                    f"Checking inputs is trying to exclude a non existing topic key(s): {inputs_to_exclude}. Available keys: {self.callbacks.keys()}"
                )
            inputs_dict_to_check = self.__exclude_keys_from_dict(
                self.callbacks, inputs_to_exclude
            )

        elif inputs_to_check:
            # If a non valid key is provided raise an error
            if not all(item in self.callbacks.keys() for item in inputs_to_check):
                raise ValueError(
                    f"Checking inputs is trying to restrict check to non existing topic key(s): {inputs_to_check}. Available keys: {self.callbacks.keys()}"
                )
            inputs_dict_to_check = self.__restrict_keys_from_dict(
                self.callbacks, inputs_to_check
            )

        else:
            inputs_dict_to_check = self.callbacks

        # Check if all callbacks of the selected topics got input messages
        for callback in inputs_dict_to_check.values():
            if not callback.got_msg:
                return False
        return True

    def get_missing_inputs(self) -> list[str]:
        """
        Get a list of input topic names not being published

        :return: List of unpublished topics
        :rtype: list[str]
        """
        unpublished_topics = []
        for callback in self.callbacks.values():
            if not callback.got_msg:
                unpublished_topics.append(callback.input_topic.name)
        return unpublished_topics

    def __exclude_keys_from_dict(self, input_dict: Dict, key_list: List) -> Dict:
        """Return input_dict without excluded keys

        :param input_dict: Input dictionary
        :type input_dict: Dict
        :param key_list: Keys to exclude
        :type key_list: List
        :return: Output dictionary
        :rtype: Dict
        """
        return {key: value for key, value in input_dict.items() if key not in key_list}

    def __restrict_keys_from_dict(self, input_dict: Dict, key_list: List) -> Dict:
        """Return input_dict with only restricted keys

        :param input_dict: Input dictionary
        :type input_dict: Dict
        :param key_list: Keys to restrict
        :type key_list: List
        :return: Output dictionary
        :rtype: Dict
        """
        return {key: value for key, value in input_dict.items() if key in key_list}

    # CONFIGURATION
    def configure(self, config_file: str):
        """
        Configure component from yaml file

        :param config_file: Path to file
        :type config_file: str
        """
        self.config.from_yaml(
            config_file, nested_root_name=self.node_name, get_common=True
        )

    @property
    def run_type(self) -> ComponentRunType:
        """
        Component run type: Timed, ActionServer or Server

        :return: Timed, ActionServer or Server
        :rtype: str
        """
        return self.config.run_type

    @run_type.setter
    def run_type(self, value: ComponentRunType):
        self.config.run_type = value

    @property
    def fallback_rate(self) -> float:
        """
        Component fallback rate: Rate in which the component checks for fallbacks and executes a fallback actions if a failure is detected

        :return: Fallback rate (Hz)
        :rtype: float
        """
        return self.config.fallback_rate

    @fallback_rate.setter
    def fallback_rate(self, value: float):
        self.config.fallback_rate = value

    @property
    def loop_rate(self) -> float:
        """
        Component loop rate: Rate in which the component executes its main (_execution_step)

        :return: Loop rate (Hz)
        :rtype: float
        """
        return self.config.loop_rate

    @loop_rate.setter
    def loop_rate(self, value: float):
        self.config.loop_rate = value

    @property
    def events(self) -> Optional[List[Event]]:
        return self.__events

    @events.setter
    def events(self, event_list: List[Event]) -> None:
        self.__events = event_list

    @property
    def events_actions(self) -> Dict[str, List[Action]]:
        """Getter of component Events/Actions

        :return: Dictionary of monitored Events and associated Actions
        :rtype: Dict[str, List[Action]]
        """
        events_actions_names = {}
        if not self.__events or not self.__actions:
            return {}
        for event, action_set in zip(self.__events, self.__actions):
            events_actions_names[event.name] = action_set
        return events_actions_names

    @events_actions.setter
    def events_actions(
        self, events_actions_dict: Dict[Event, Union[Action, List[Action]]]
    ):
        """Setter of component Events/Actions

        :param events_actions_dict: Dictionary of Events and associated Actions
        :type events_actions_dict: Dict[Event, List[Action]]
        :raises ValueError: If a given Action does not correspond to a valid component method
        """
        self.__events = []
        self.__actions = []
        for event, actions in events_actions_dict.items():
            action_set = actions if isinstance(actions, list) else [actions]
            for action in action_set:
                if not hasattr(self, action.action_name):
                    raise ValueError(
                        f"Component '{self.node_name}' does not support action '{action.action_name}'"
                    )
            self.__events.append(event)
            self.__actions.append(action_set)

    # SERIALIZATION AND DESERIALIZATION
    def update_cmd_args_list(self):
        """
        Update launch command arguments
        """
        self.launch_cmd_args = [
            "--component_type",
            self.__class__.__name__,
            "--config_type",
            self.config.__class__.__name__,
            "--config",
            self.config_json,
            "--node_name",
            self.node_name,
            "--inputs",
            self._inputs_json,
            "--outputs",
            self._outputs_json,
        ]

        if self._config_file:
            self.launch_cmd_args = ["--config_file", self._config_file]

        if self.__events:
            self.launch_cmd_args = ["--events", self._events_json]

        if self.__actions:
            self.launch_cmd_args = ["--actions", self._actions_json]

        if self._external_processors:
            self.launch_cmd_args = [
                "--external_processors",
                self._external_processors_json,
            ]

    @property
    def _events_json(self) -> Union[str, bytes]:
        """Getter of serialized component Events

        :return: Serialized Events List
        :rtype: Union[str, bytes]
        """
        if not self.__events:
            return "[]"
        return json.dumps([event.json for event in self.__events])

    @_events_json.setter
    def _events_json(self, events_serialized: Union[str, bytes]):
        """Setter of component events from JSON serialized events

        :param events_serialized: Serialized Events List
        :type events_serialized: Union[str, bytes]
        """
        self.__events = json_to_events_list(events_serialized)

    @property
    def _actions_json(self) -> Union[str, bytes]:
        """Getter of serialized component Actions

        :return: Serialized Actions: {event_name: serialized_action}
        :rtype: Union[str, bytes]
        """
        actions_dict = {}
        for event_name, action_set in self.events_actions.items():
            actions_serialized = []
            for action in action_set:
                actions_serialized.append(action.dictionary)
            actions_dict[event_name] = actions_serialized
        return json.dumps(actions_dict)

    @_actions_json.setter
    def _actions_json(self, actions_serialized: Union[str, bytes]):
        """Setter of component events from JSON serialized actions

        :param actions_serialized: Serialized Actions List
        :type actions_serialized: Union[str, bytes]
        """
        self.__actions = []
        actions_dict: Dict = json.loads(actions_serialized)
        for action_list in actions_dict.values():
            reconstructed_action_list = []
            for action_dict in action_list:
                if not hasattr(self, action_dict["action_name"]):
                    raise AttributeError(
                        f"Component '{self.node_name}' does not contain requested Action method '{action_dict['action_name']}'"
                    )
                method = getattr(self, action_dict["action_name"])
                reconstructed_action = Action(
                    method=method,
                    args=action_dict["args"],
                    kwargs=action_dict["kwargs"],
                )
                reconstructed_action_list.append(reconstructed_action)
            self.__actions.append(reconstructed_action_list)

    @property
    def _inputs_json(self) -> Union[str, bytes, bytearray]:
        """
        Serialize component inputs to json

        :return: Serialized inputs
        :rtype:  str | bytes | bytearray
        """
        if not hasattr(self, "in_topics"):
            return "[]"
        return json.dumps([topic.to_json() for topic in self.in_topics])

    @_inputs_json.setter
    def _inputs_json(self, value: Union[str, bytes, bytearray]):
        """
        Component inputs from serialized inputs (json)

        :return: Serialized inputs
        :rtype:  str | bytes | bytearray

        :param value: Serialized inputs
        :type value: Union[str, bytes, bytearray]
        """
        self.in_topics = json.loads(value)
        self.callbacks = {
            input.name: input.msg_type.callback(input) for input in self.in_topics
        }

    @property
    def _outputs_json(self) -> Union[str, bytes, bytearray]:
        """
        Serialize component inputs to json

        :return: Serialized inputs
        :rtype:  str | bytes | bytearray
        """
        if not hasattr(self, "out_topics"):
            return "[]"
        return json.dumps([topic.to_json() for topic in self.out_topics])

    @_outputs_json.setter
    def _outputs_json(self, value: Union[str, bytes, bytearray]):
        """
        Component inputs from serialized inputs (json)

        :return: Serialized inputs
        :rtype:  str | bytes | bytearray

        :param value: Serialized inputs
        :type value: Union[str, bytes, bytearray]
        """
        self.out_topics = json.loads(value)
        self.publishers_dict = {
            output.name: Publisher(output, node_name=self.node_name)
            for output in self.out_topics
        }

    @property
    def _external_processors_json(self) -> Union[str, bytes]:
        """Getter of serialized external processors

        :return: Serialized external processors definition
        :rtype: Union[str, bytes]
        """
        return json.dumps({
            topic_name: ([p.__name__ for p in processors], processor_type)  # type: ignore
            for topic_name, (
                processors,
                processor_type,
            ) in self._external_processors.items()
        })

    @_external_processors_json.setter
    def _external_processors_json(self, processors_serialized: Union[str, bytes]):
        """Setter of external processors from JSON serialized processors

        :param processors_serialized: Serialized Processors Dict
        :type processors_serialized: Union[str, bytes]
        """
        processors_data = json.loads(processors_serialized)
        # Create sockets out of function names and connect them
        for key, processor_data in processors_data.items():
            for idx, func_name in enumerate(processor_data[0]):
                sock_file = f"/tmp/{self.node_name}_{key}_{func_name}.socket"
                if not os.path.exists(sock_file):
                    self.get_logger().error(
                        f"File {sock_file} doesn't exists. The external processors have not been setup properly. Exiting .. "
                    )
                    raise KeyboardInterrupt()

                sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                sock.settimeout(1)  # timeout set to 1s
                sock.connect(sock_file)
                processor_data[0][idx] = sock

    # DUNDER METHODS
    def __matmul__(self, stream) -> Optional[Topic]:
        """
        @

        :param stream: _description_
        :type stream: _type_
        :raises TypeError: _description_
        :return: _description_
        :rtype: _type_
        """
        got_topic: bool = False
        if isinstance(stream, str):
            # search in inputs
            if self.in_topics:
                got_topic = next(
                    (topic for topic in self.in_topics if topic.name == stream), None
                )

            # search in outputs
            elif not got_topic and self.out_topics:
                got_topic = next(
                    (topic for topic in self.out_topics if topic.name == stream), None
                )

            return got_topic

        elif isinstance(stream, Topic):
            if self.in_topics:
                got_topic = next(
                    (topic for topic in self.in_topics if topic == stream), None
                )

            elif not got_topic and self.out_topics:
                got_topic = next(
                    (topic for topic in self.out_topics if topic == stream), None
                )

            return got_topic
        else:
            raise TypeError(
                "Component method '@' can only be used with a topic defined by a string key name or a Topic instance"
            )

    # TODO: Implement more dunder methods for a more intuitive API with components

    # MAIN ACTION SERVER HELPER METHODS AND CALLBACKS
    @abstractmethod
    def main_action_callback(self, goal_handle):
        """
        Component main action server callback - used if component started with run_as_action_server=True

        :param goal_handle: Action goal handle
        :type goal_handle: action_type.Goal
        """
        if self.run_type == ComponentRunType.ACTION_SERVER:
            raise NotImplementedError
        pass

    def _main_action_goal_callback(self, _):
        """
        Goal callback for the main component action server

        :param goal_request: _description_
        :type goal_request: Any action goal handler type
        :return: ACCEPT
        :rtype: rclpy.action.GoalResponse
        """
        self.get_logger().info("Received goal request")
        return GoalResponse.ACCEPT

    def _main_action_handle_accepted_callback(self, goal_handle):
        """
        Main component action server callback when handle is accepted
        """
        self.get_logger().info("Goal accepted")
        goal_handle.execute()

    def _main_action_cancel_callback(self, _):
        """Main component action server callback when handle is canceled

        :param goal_handle: _description_
        :type goal_handle: _type_
        :return: _description_
        :rtype: _type_
        """
        self.get_logger().warn("Received cancel request")
        try:
            return CancelResponse.ACCEPT
        except Exception as e:
            self.get_logger().error(f"Failed to send accept cancel request: {e}")

    @property
    def main_action_name(self) -> Optional[str]:
        """
        Name of the main action server created by the component

        :return: ActionServer name
        :rtype: str
        """
        if self.action_type and hasattr(self.action_type, "__name__"):
            return f"{self.node_name}/{camel_to_snake_case(self.action_type.__name__)}"
        return None

    # MAIN SERVER HELPER METHODS AND CALLBACK
    @property
    def main_srv_name(self) -> Optional[str]:
        """
        Name of the main server created by the component

        :return: Server name
        :rtype: str
        """
        if self.service_type and hasattr(self.service_type, "__name__"):
            return f"{self.node_name}/{camel_to_snake_case(self.service_type.__name__)}"
        return None

    @abstractmethod
    def main_service_callback(self, request, response):
        """
        Component main service callback - used if component started with run_as_server=True

        :param request: Service request
        :type request: service_type.Request
        :param response: Service response
        :type response: service_type.Response
        """
        if self.run_type == ComponentRunType.SERVER:
            raise NotImplementedError
        return response

    def _create_default_services(self):
        """
        Creates default services for updating parameters and changing input/output topics
        """
        # to handle one call at a time set callback to new MutuallyExclusiveCallbackGroup

        # Config update services
        self._update_parameter_srv = self.create_service(
            srv_type=ChangeParameter,
            srv_name=f"{self.get_name()}/update_config_parameter",
            callback=self._update_config_parameter_srv_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self._update_parameters_srv = self.create_service(
            srv_type=ChangeParameters,
            srv_name=f"{self.get_name()}/update_config_parameters",
            callback=self._update_config_parameters_srv_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        # Input/Output update services
        self._topic_change_srv = self.create_service(
            srv_type=ReplaceTopic,
            srv_name=f"{self.get_name()}/change_topic",
            callback=self._change_topic_srv_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self._configure_from_yaml_srv = self.create_service(
            srv_type=ConfigureFromYaml,
            srv_name=f"{self.get_name()}/configure_from_yaml",
            callback=self._configure_from_yaml_srv_callback,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

    # EVENTS/ACTIONS RELATED SERVICES
    @log_srv
    def _configure_from_yaml_srv_callback(
        self, request: ConfigureFromYaml.Request, response: ConfigureFromYaml.Response
    ) -> ConfigureFromYaml.Response:
        """
        Configure the component from yaml service callback

        :param request: _description_
        :type request: ConfigureFromYaml.Request
        :param response: _description_
        :type response: ConfigureFromYaml.Response
        :return: _description_
        :rtype: ConfigureFromYaml.Response
        """
        try:
            # Reconfigure and restart the node
            reconfigured = self.reconfigure(request.path_to_file)
            if reconfigured:
                response.success = True
            else:
                response.success = False
                response.error_msg = "Failed to Restart the node"
        except Exception as e:
            response.success = False
            response.error_msg = f"{e}"
        return response

    def _update_config_param_from_str_value(
        self, param_name: str, param_str_value: str
    ) -> Optional[str]:
        """
        Helper method to update config parameter value and return an error message if it is not updated

        :param param_name: _description_
        :type param_name: str
        :param param_str_value: _description_
        :type param_str_value: str

        :return: Error message - None if the parameter is updated without errors
        :rtype: str | None
        """
        error_msg: Optional[str] = None
        if not self.config.has_attribute(param_name):
            error_msg = f"'{self.config.__class__.__name__}' does not contain an attribute '{param_name}'"

        param_type = self.config.get_attribute_type(param_name)

        try:
            parsed_param = param_type(param_str_value) if param_type else None
            self.config.update_value(param_name, parsed_param)

        except Exception as e:
            error_msg = f"'{self.config.__class__.__name__}' attribute '{param_name}' is of type {param_type}. Error message details: {e}"

        return error_msg

    @classmethod
    def get_change_parameters_msg_from_config(
        cls, config: BaseComponentConfig
    ) -> ChangeParameters.Request:
        """
        Helper method to update config parameter value and return an error message if it is not updated

        :param config: _description_
        :type config: ComponentConfig or child class

        :return: Request message for change parameters services corresponding to the given config
        :rtype: ChangeParameters.Request
        """
        param_names = []
        param_values = []
        for param_name, param_value in config.asdict().items():
            if not isinstance(param_value, Dict):
                param_names.append(param_name)
                param_values.append(str(param_value))
            else:
                for nested_name, nested_value in param_value.items():
                    param_names.append(f"{param_name}.{nested_name}")
                    param_values.append(str(nested_value))

        request_msg = ChangeParameters.Request()
        request_msg.names = param_names
        request_msg.values = param_values

        return request_msg

    @log_srv
    def _update_config_parameter_srv_callback(
        self, request: ChangeParameter.Request, response: ChangeParameter.Response
    ) -> ChangeParameter.Response:
        """
        Update config parameter service callback

        :param request: _description_
        :type request: ChangeParameter.Request
        :param response: _description_
        :type response: ChangeParameter.Response
        :return: _description_
        :rtype: ChangeParameter.Response
        """
        param_name: str = request.name
        param_str_value: str = request.value
        # TODO: Handle keep_alive
        # keep_alive = request.keep_alive

        error_msg = self._update_config_param_from_str_value(
            param_name, param_str_value
        )

        if not error_msg:
            response.success = True
        else:
            response.success = False
            response.error_msg = error_msg

        return response

    @log_srv
    def _update_config_parameters_srv_callback(
        self, request: ChangeParameters.Request, response: ChangeParameters.Response
    ) -> ChangeParameters.Response:
        """
        Update list of config parameters service callback

        :param request: _description_
        :type request: ChangeParameters.Request
        :param response: _description_
        :type response: ChangeParameters.Response
        :return: _description_
        :rtype: ChangeParameters.Response
        """
        param_names: List[str] = request.names
        param_str_values: List[str] = request.values
        # TODO: handle not keeping the node alive
        # keep_alive = request.keep_alive

        response.success = []
        response.error_msg = []

        for name, val in zip(param_names, param_str_values):
            error_msg = self._update_config_param_from_str_value(name, val)

            if not error_msg:
                response.success.append(True)
                response.error_msg.append("")
            else:
                response.success.append(False)
                response.error_msg.append(error_msg)

        return response

    def _replace_input_topic(
        self, topic_name: str, new_name: str, msg_type: str
    ) -> Optional[str]:
        """
        Replace an existing input topic

        :param topic_name: Existing input topic name
        :type topic_name: str
        :param new_name: New input topic name
        :type new_name: str
        :param msg_type: New message type
        :type msg_type: str

        :return: Error message if replacement failed, else None
        :rtype: str | None
        """
        error_msg = None
        # Normalize names
        normalized_topic_name = (
            topic_name[1:] if topic_name.startswith("/") else topic_name
        )

        if topic_name not in self.callbacks.keys():
            error_msg = f"Topic {topic_name} is not found in Component inputs"
            return error_msg

        callback = self.callbacks[normalized_topic_name]

        try:
            # Create new topic
            new_topic = Topic(name=new_name, msg_type=msg_type)
            callback.input_topic = new_topic
        except Exception as e:
            error_msg = f"Invalid topic parameters: {e}"
            return error_msg

        # Destroy subscription if it is already activated and create new callback
        if callback._subscriber:
            self.get_logger().info(
                f"Destroying subscriber for old topic '{topic_name}'"
            )
            self.destroy_subscription(callback._subscriber)

            self.get_logger().info(f"Creating subscriber for new topic '{new_name}'")
            callback.set_subscriber(self._add_ros_subscriber(callback))
            return None

        # Update in_topics list (If the previous subscriber is not created it will get created from in_topics on activation)
        self.in_topics = [
            topic for topic in self.in_topics if topic.name != normalized_topic_name
        ]
        self.in_topics.append(new_topic)
        return None

    def _replace_output_topic(
        self, topic_name: str, new_name: str, msg_type: str
    ) -> Optional[str]:
        error_msg = None
        # Normalize names
        normalized_topic_name = (
            topic_name[1:] if topic_name.startswith("/") else topic_name
        )

        if topic_name not in self.publishers_dict.keys():
            error_msg = f"Topic {topic_name} is not found in Component outputs"
            return error_msg

        publisher = self.publishers_dict[normalized_topic_name]

        try:
            # Create new topic
            new_topic = Topic(name=new_name, msg_type=msg_type)
            publisher.output_topic = new_topic
        except Exception as e:
            error_msg = f"Invalid topic parameters: {e}"
            return error_msg

        # Destroy subscription if it is already activated and create new callback
        if publisher._publisher:
            self.get_logger().info(f"Destroying publisher for old topic '{topic_name}'")
            self.destroy_publisher(publisher._publisher)

            self.get_logger().info(f"Creating publisher for new topic '{new_name}'")
            publisher.set_publisher(self._add_ros_publisher(publisher))
            return None

        # Update in_topics list (If the previous subscriber is not created it will get created from in_topics on activation)
        self.out_topics = [
            topic for topic in self.out_topics if topic.name != normalized_topic_name
        ]
        self.out_topics.append(new_topic)
        return None

    @log_srv
    def _change_topic_srv_callback(
        self, request: ReplaceTopic.Request, response: ReplaceTopic.Response
    ) -> ReplaceTopic.Response:
        """
        Change topic service callback

        :param request: _description_
        :type request: ReplaceTopic.Request
        :param response: _description_
        :type response: ReplaceTopic.Response
        :return: _description_
        :rtype: ReplaceTopic.Response
        """
        if request.direction == ReplaceTopic.Request.INPUT_TOPIC:
            error_msg = self._replace_input_topic(
                request.old_name, request.new_name, request.new_msg_type
            )
            if not error_msg:
                response.success = True
            else:
                response.success = False
                response.error_msg = error_msg

        elif request.direction == ReplaceTopic.Request.OUTPUT_TOPIC:
            self._replace_output_topic(
                request.old_name, request.new_name, request.new_msg_type
            )
            response.success = False
            response.error_msg = "Not implemented"
        else:
            response.success = False
            response.error_msg = f"Got invalid direction value '{request.direction}'. Direction can only be in [{ReplaceTopic.INPUT_TOPIC} -> input, or {ReplaceTopic.OUTPUT_TOPIC} -> output]"

        return response

    # END OF EVENTS/ACTIONS RELATED SERVICES
    def is_topic_of_type(self, input, msg_type: type) -> bool:
        """
        Checks if a topic contains a msg of given type

        :param input: Object to check
        :type input: Topic
        :param msg_type: Topic message type to check
        :type msg_type: type

        :return: If input is a Topic with given message type
        :rtype: bool
        """
        return isinstance(input, Topic) and input.ros_msg_type == msg_type

    def attach_callbacks(self):
        """
        Run on-activate node. Override this method to attach methods to callbacks
        """
        pass

    def _attach_external_processors(self):
        """
        Attach external processors
        """
        for topic_name, (
            processors,
            processor_type,
        ) in self._external_processors.items():
            if processor_type == "preprocessor":
                self.publishers_dict[topic_name].add_pre_processors(processors)
            elif processor_type == "postprocessor":
                self.callbacks[topic_name].add_post_processors(processors)

    def _destroy_external_processors(self):
        """
        Destroy external processors
        """
        if len(self._external_processors):
            for processors, _ in self._external_processors.values():
                for processor in processors:
                    if isinstance(processor, socket.socket):
                        processor.close()

    # MAIN
    def _main(self):
        """
        Component execution step every loop_step
        """
        # Additional execution loop if exists
        if hasattr(self, "_extra_execute_loop"):
            self._extra_execute_loop()

        # Execute main loop
        self._execution_step()

        if self.__enable_health_publishing:
            self.health_status_publisher.publish(self.health_status())

        # Execute once
        if not hasattr(self, "_exec_started"):
            self._execute_once()
            if hasattr(self, "_extra_execute_once"):
                self._extra_execute_once()
            self._exec_started = True

    # COMPONENT ACTIONS
    @property
    def available_actions(self) -> List[str]:
        """
        Getter of available component actions

        :return: Methods names
        :rtype: List[str]
        """
        return get_methods_with_decorator(self, decorator_name="component_action")

    @component_action
    def start(self) -> bool:
        """
        Start the component - trigger_activate

        :return: If the component is started
        :rtype: bool
        """
        current_state = self.lifecycle_state

        if current_state == 3:
            # Component already active
            return True

        elif current_state in [1, 4]:
            # unconfigured or finalized -> configure again before starting
            self.trigger_configure()

        while current_state > 4:
            self.get_logger().warn(
                "Waiting for ongoing transition to end before executing new transition",
                once=True,
            )
            pass

        # configured and inactive
        self.trigger_activate()
        return True

    @component_action
    def stop(self) -> bool:
        """
        Stop the component - trigger_deactivate

        :return: If the component is stopped
        :rtype: bool
        """
        current_state = self.lifecycle_state

        if current_state in [1, 2, 4]:
            # Already not active
            return True

        while current_state > 4:
            self.get_logger().warn(
                "Waiting for ongoing transition to end before executing new transition",
                once=True,
            )
            pass

        self.trigger_deactivate()
        return True

    @component_action
    def reconfigure(self, new_config: Any, keep_alive: bool = False) -> bool:
        """
        Reconfigure the component - cleanup->stop->trigger_configure->start

        :param new_config: New component config
        :type new_config: Any
        :param keep_alive: Reconfigure while the component is online, defaults to False
        :type keep_alive: bool, optional

        :return: If the component is Reconfigured
        :rtype: bool
        """
        self.get_logger().warn("Reconfiguring component...")

        # set new config as params attr
        if isinstance(new_config, str):
            self.configure(config_file=new_config)
        elif isinstance(new_config, self.config.__class__):
            self.config = new_config

        if keep_alive:
            return True

        initial_state = self.lifecycle_state

        if initial_state == 2:
            # Already configured -> cleanup first
            self.trigger_cleanup()

        if initial_state == 3:
            # active -> deactivate then cleanup
            self.trigger_deactivate()
            self.trigger_cleanup()

        while initial_state > 4:
            self.get_logger().warn(
                "Waiting for ongoing transition to end before executing new transition",
                once=True,
            )
            pass

        # configure and go to configure (or active if the component was already active)
        self.trigger_configure()

        if initial_state >= 3:
            self.trigger_activate()
        return True

    @component_action
    def restart(self, wait_time: Optional[float] = None) -> bool:
        """
        Restart the component - stop->start

        :return: If the component is Reconfigured
        :rtype: bool
        """
        current_state = self.lifecycle_state

        if current_state == 1:
            self.trigger_configure()

        if current_state == 3:
            self.trigger_deactivate()

        while current_state > 4:
            self.get_logger().warn(
                "Waiting for ongoing transition to end before executing new transition",
                once=True,
            )
            pass

        if wait_time:
            self.get_logger().warn(
                f"Waiting for requested time '{wait_time}'seconds before starting again...",
            )
            time.sleep(wait_time)

        # not configured -> configure and start
        self.trigger_activate()
        return True

    @component_action
    def set_param(
        self, param_name: str, new_value: Any, keep_alive: bool = True
    ) -> bool:
        """
        Change the value of one component parameter

        :param param_name: _description_
        :type param_name: str
        :param new_value: _description_
        :type new_value: Any
        :param keep_alive: To keep the component running when updating value, defaults to True
        :type keep_alive: bool, optional

        :raises Exception: Parameter could not be updated to given value

        :return: Parameter updated
        :rtype: bool
        """
        try:
            if keep_alive:
                self.config.update_value(param_name, new_value)
            else:
                self.stop()
                self.config.update_value(param_name, new_value)
                self.start()
        except Exception:
            raise
        return True

    @component_action
    def set_params(
        self, params_names: List[str], new_values: List, keep_alive: bool = True
    ) -> bool:
        """
        Change the value of multiple component parameters

        :param param_name: _description_
        :type param_name: str
        :param new_value: _description_
        :type new_value: Any
        :param keep_alive: To keep the component running when updating value, defaults to True
        :type keep_alive: bool, optional

        :raises Exception: Parameter could not be updated to given value

        :return: Parameter updated
        :rtype: bool
        """
        try:
            if keep_alive:
                for param_name, new_value in zip(params_names, new_values):
                    self.config.update_value(param_name, new_value)
            else:
                self.stop()
                for param_name, new_value in zip(params_names, new_values):
                    self.config.update_value(param_name, new_value)
                self.start()
        except Exception:
            raise
        return True

    # END OF ACTIONS

    # FALLBACKS
    def _fallbacks_check_callback(self):
        """
        Checks component health status and executes corresponding fallback in case of any detected failure
        """
        if self.health_status.is_healthy:
            # Component is healthy -> nothing to do,
            return

        try:
            if self.__fallbacks_giveup:
                # All fallbacks are already exhausted
                self.get_logger().error(
                    "All possible fallbacks are exhausted -> Component is givingup"
                )
                self.__fallbacks.execute_giveup()

            elif (
                self.health_status.is_algorithm_fail
                and self.__fallbacks.on_algorithm_fail
            ):
                self.get_logger().warn(
                    "Algorithm Failure Detected -> Executing Fallback ..."
                )
                self.__fallbacks_giveup = self.__fallbacks.execute_algorithm_fallback()

            elif (
                self.health_status.is_component_fail
                and self.__fallbacks.on_component_fail
            ):
                self.get_logger().warn(
                    "Component Failure Detected -> Executing Fallback ..."
                )
                self.__fallbacks_giveup = self.__fallbacks.execute_component_fallback()

            elif self.health_status.is_system_fail and self.__fallbacks.on_system_fail:
                self.get_logger().warn(
                    "System Failure Detected -> Executing Fallback ..."
                )
                self.__fallbacks_giveup = self.__fallbacks.execute_system_fallback()

            else:
                self.__fallbacks_giveup = self.__fallbacks.execute_generic_fallback()

        except ValueError:
            # ValueError is thrown when no fallbacks are defined for detected failure
            if self.__enable_health_publishing:
                self.get_logger().warn(
                    "No fallback policy is defined for detected failure -> Failure is broadcasted"
                )
            else:
                self.get_logger().error(
                    "No fallback policy is defined for detected failure and health publishing is disabled. Component will keep running but will possibly not be running correctly"
                )

    @property
    def fallbacks(self) -> List[str]:
        """
        Gets all available component fallback methods

        :return: List of available fallback names
        :rtype: List[str]
        """
        component_actions = get_methods_with_decorator(
            self, decorator_name="component_action"
        )
        component_fallbacks = get_methods_with_decorator(
            self, decorator_name="component_fallback"
        )
        return component_actions + component_fallbacks

    def _is_valid_fallback_action(self, action: Union[List[Action], Action]) -> bool:
        """Checks if a given action / list of actions are valid component fallbacks

        :param fallback: Method to be checked
        :type fallback: Union[List[Action], Action]

        :return: If action is a valid component fallback
        :rtype: bool
        """
        if isinstance(action, List):
            for f in action:
                self.__check_fallback_action(f)
            return True
        return self.__check_fallback_action(action)

    def __check_fallback_action(self, action: Optional[Action]) -> bool:
        """Checks if a given action is a valid component fallback

        :param fallback: Method to be checked
        :type fallback: Union[List[Action], Action]

        :return: If action is a valid component fallback
        :rtype: bool
        """
        if not action:
            raise ValueError("Cannot set fallback action to None")

        if action.parent_component is not self.node_name:
            raise TypeError(
                f"Non valid failure fallback {action.parent_component}.{action.action_name}. Component fallback can only be a component's own fallback method"
            )
        if action.action_name not in self.fallbacks:
            raise TypeError(
                f"Non valid failure fallback {action.parent_component}.{action.action_name}. Available component fallbacks are the following methods: '{self.fallbacks}'"
            )
        return True

    def on_fail(
        self, action: Union[List[Action], Action], max_retries: Optional[int] = None
    ) -> None:
        """
        Set the fallback strategy (action) on any fail

        :param action: Action to be executed on failure
        :type action: Union[List[Action], Action]
        :param max_retries: Maximum number of action execution retries. None is equivalent to unlimited retries, defaults to None
        :type max_retries: Optional[int], optional
        """
        if self._is_valid_fallback_action(action):
            self.__fallbacks.on_any_fail = Fallback(
                action=action, max_retries=max_retries
            )

    def on_system_fail(
        self, action: Union[List[Action], Action], max_retries: Optional[int] = None
    ) -> None:
        """
        Set the fallback strategy (action) on system fail

        :param action: Action to be executed on failure
        :type action: Union[List[Action], Action]
        :param max_retries: Maximum number of action execution retries. None is equivalent to unlimited retries, defaults to None
        :type max_retries: Optional[int], optional
        """
        if self._is_valid_fallback_action(action):
            self.__fallbacks.on_system_fail = Fallback(
                action=action, max_retries=max_retries
            )

    def on_component_fail(
        self, action: Union[List[Action], Action], max_retries: Optional[int] = None
    ) -> None:
        """
        Set the fallback strategy (action) on component fail

        :param action: Action to be executed on failure
        :type action: Union[List[Action], Action]
        :param max_retries: Maximum number of action execution retries. None is equivalent to unlimited retries, defaults to None
        :type max_retries: Optional[int], optional
        """
        if self._is_valid_fallback_action(action):
            self.__fallbacks.on_component_fail = Fallback(
                action=action, max_retries=max_retries
            )

    def on_algorithm_fail(
        self, action: Union[List[Action], Action], max_retries: Optional[int] = None
    ) -> None:
        """
        Set the fallback strategy (action) on algorithm fail

        :param action: Action to be executed on failure
        :type action: Union[List[Action], Action]
        :param max_retries: Maximum number of action execution retries. None is equivalent to unlimited retries, defaults to None
        :type max_retries: Optional[int], optional
        """
        if self._is_valid_fallback_action(action):
            self.__fallbacks.on_algorithm_fail = Fallback(
                action=action, max_retries=max_retries
            )

    @component_fallback
    def broadcast_status(self) -> None:
        """
        Component fallback defined to only broadcast the current state so it is handled by an external manager.
        Used as the default fallback strategy for any system (external) failure
        """
        if hasattr(self, "health_status_publisher"):
            self.health_status_publisher.publish(self.health_status())

    # LIFECYCLE ON TRANSITIONS CUSTOM METHODS
    @property
    def lifecycle_state(self) -> Optional[int]:
        """
        lifecycle state machine current state getter

        :return: _description_
        :rtype: int
        """
        if hasattr(self, "_state_machine"):
            return self._state_machine.current_state[0]

    def on_configure(
        self, state: lifecycle.State
    ) -> lifecycle.TransitionCallbackReturn:
        """
        Method on node state transition to Configured
        Declares node parameters and inits the base node (with initial flags and variables)

        :param state: Current node state
        :type state: lifecycle.State

        :return: Node state transition result
        :rtype: lifecycle.TransitionCallbackReturn
        """
        try:
            # Call custom method
            self.health_status.set_healthy()
            self.custom_on_configure()

            self.get_logger().info(
                f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'configured'"
            )

        except Exception as e:
            self.get_logger().error(
                f"Transition error for node {self.get_name()} to transition to state '{state.label}': {e}"
            )
            self.health_status.set_fail_component(component_names=[self.get_name()])
            self.custom_on_error()

        return super().on_configure(state)

    def on_activate(self, state: lifecycle.State) -> lifecycle.TransitionCallbackReturn:
        """
        Method on node state transition to Active
        Starts node subscriptions, publications, services and clients

        :param state: Current node state
        :type state: lifecycle.State

        :return: Node state transition result
        :rtype: lifecycle.TransitionCallbackReturn
        """
        try:
            self.activate()
            # Declare transition
            self.get_logger().info(
                f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'activate'"
            )

            self.attach_callbacks()

            self._turn_on_events_management()

            # Create external processors
            self._attach_external_processors()

            # Create failure check timer
            self.__fallbacks_check_timer = self.create_timer(
                timer_period_sec=1 / self.config.fallback_rate,
                callback=self._fallbacks_check_callback,
                callback_group=MutuallyExclusiveCallbackGroup(),
            )

            self.health_status.set_healthy()
            # Call custom method
            self.custom_on_activate()

        except Exception as e:
            self.get_logger().error(
                f"Transition error for node {self.get_name()} to transition to state '{state.label}': {e}"
            )
            self.health_status.set_fail_component(component_names=[self.get_name()])
            self.custom_on_error()

        return super().on_activate(state)

    def on_deactivate(
        self, state: lifecycle.State
    ) -> lifecycle.TransitionCallbackReturn:
        """
        Method on node state transition to Deactivate

        :param state: Current node state
        :type state: lifecycle.State

        :return: Node state transition result
        :rtype: lifecycle.TransitionCallbackReturn
        """
        try:
            self.deactivate()
            # Declare transition
            self.get_logger().info(
                f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'deactivate'"
            )

            self.destroy_timer(self.__fallbacks_check_timer)
            self.health_status.set_healthy()
            # Call custom method
            self.custom_on_deactivate()

        except Exception as e:
            self.get_logger().error(
                f"Transition error for node {self.get_name()} to transition to state '{state.label}': {e}"
            )
            self.health_status.set_fail_component(component_names=[self.get_name()])
            self.custom_on_error()

        return super().on_deactivate(state)

    def on_shutdown(self, state: lifecycle.State) -> lifecycle.TransitionCallbackReturn:
        """
        Method on node state transition to Finalized

        :param state: Current node state
        :type state: lifecycle.State

        :return: Node state transition result
        :rtype: lifecycle.TransitionCallbackReturn
        """
        try:
            try_shutdown()
            self.get_logger().info(
                f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'shutdown'"
            )
            self.health_status.set_healthy()
            # Call custom method
            self.custom_on_shutdown()
        except Exception as e:
            self.get_logger().error(
                f"Transition error for node {self.get_name()} to transition to state '{state.label}': {e}"
            )
            self.health_status.set_fail_component(component_names=[self.get_name()])
            self.custom_on_error()

        return super().on_shutdown(state)

    def on_cleanup(self, state: lifecycle.State) -> lifecycle.TransitionCallbackReturn:
        """
        Method on node state transition to unConfigured
        Starts node subscriptions, publications, services and clients

        :param state: Current node state
        :type state: lifecycle.State

        :return: Node state transition result
        :rtype: lifecycle.TransitionCallbackReturn
        """
        try:
            # Call custom method
            self.custom_on_cleanup()

            # Declare transition
            self.get_logger().info(
                f"Node '{self.get_name()}' is in state '{state.label}'. Transitioning to 'unconfigured'"
            )

        except Exception as e:
            self.get_logger().error(
                f"Transition error for node {self.get_name()} to transition from state '{state.label}': {e}"
            )

        return super().on_cleanup(state)

    def on_error(
        self, state: lifecycle.LifecycleState
    ) -> lifecycle.TransitionCallbackReturn:
        """
        Handles a transition error

        When a transition returns TransitionCallbackReturn.FAILURE or TransitionCallbackReturn.ERROR.
        """
        self.get_logger().error(
            f"Transition error for node {self.get_name()} - {state}"
        )
        self.health_status.set_fail_component(component_names=[self.get_name()])
        self.custom_on_error()
        return super().on_error(state)

    def custom_on_configure(self) -> None:
        """
        Method called on configure to overwrite with custom configuration
        """
        pass

    def custom_on_activate(self) -> None:
        """
        Method called on activation to overwrite with custom activation
        """

        pass

    def custom_on_deactivate(self) -> None:
        """
        Method called on deactivation to overwrite with custom deactivation
        """
        pass

    def custom_on_shutdown(self) -> None:
        """
        Method called on shutdown to overwrite with custom shutdown
        """
        pass

    def custom_on_error(self) -> None:
        """
        Method called on transition error to overwrite with custom transition error handling
        """
        pass

    def custom_on_cleanup(self) -> None:
        """
        Method called on cleanup to overwrite with custom cleanup
        """
        pass
