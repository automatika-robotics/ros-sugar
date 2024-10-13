"""Monitor"""

import os
from functools import partial
from typing import Any, Callable, Dict, List, Optional, Union
from rclpy.publisher import Publisher
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from ros_sugar_interfaces.msg import ComponentStatus
from ros_sugar_interfaces.srv import (
    ChangeParameter,
    ChangeParameters,
    ConfigureFromYaml,
    ReplaceTopic,
)

from .. import base_clients
from .component import BaseComponent
from ..config import BaseConfig
from ..io.topic import Topic
from .event import Event
from .node import BaseNode
from .action import Action
from ..launch import logger


class Monitor(BaseNode):
    """
    Monitor is a ROS2 Node (not Lifecycle) responsible of monitoring the status of the stack (rest of the running nodes) and managing requests/responses from the Orchestrator.

    :::{note} When launching the stack using the Launcher, the user is not required to configure the Monitor. The Launcher will configure and launch its own Monitor internally.
    :::

    ## Main Functionalities:
    - Creates Subscribers to registered Events. The Monitor is configured to declare an InternalEvent back to the Launcher so the corresponding Action can be executed (see source implementation in launch_actions.py)
    - Creates Subscribers to all registered Components health status topics
    - Creates clients for all components main services and main action servers
    - Creates service clients to components reconfiguration services to handle actions sent from the Launcher
    """

    def __init__(
        self,
        components_names: List[str],
        enable_health_status_monitoring: bool = True,
        events_actions: Optional[Dict[Event, List[Action]]] = None,
        events_to_emit: Optional[List[Event]] = None,
        config: Optional[BaseConfig] = None,
        services_components: Optional[List[BaseComponent]] = None,
        action_servers_components: Optional[List[BaseComponent]] = None,
        activate_on_start: Optional[List[BaseComponent]] = None,
        start_on_init: bool = False,
        component_name: str = "monitor",
        callback_group: Optional[
            Union[MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup]
        ] = None,
        *args,
        **kwargs,
    ):
        """
        Setup the Monitor node

        :param components_names: List containing the ROS2 Node names of the components to be monitored
        :type components_names: List[str]
        :param events: List of Events to be monitored, defaults to None
        :type events: Optional[List[Event]], optional
        :param actions: Dictionary that associates each Event to a valid Action. Should be provided in case events is not None. An error is thrown if an invalid event name , defaults to None
        :type actions: Optional[Dict[str, Action]], optional
        :param config: Basic node configuration, defaults to None
        :type config: Optional[BaseConfig], optional
        :param services_components: List of components running as Servers, defaults to None
        :type services_components: Optional[List[Component]], optional
        :param action_servers_components: List of components running as Action Servers, defaults to None
        :type action_servers_components: Optional[List[Component]], optional
        :param activate_on_start: List of Lifecycle components to activate on start, defaults to None
        :type activate_on_start: Optional[List[Component]], optional
        :param start_on_init: To activate provided components on start, defaults to False
        :type start_on_init: bool, optional
        :param component_name: Name of the ROS2 node, defaults to "monitor"
        :type component_name: str, optional
        :param callback_group: Callback group, defaults to None
        :type callback_group:  Optional[Union[MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup]], optional
        """
        self._events_actions = events_actions
        self._internal_events = events_to_emit
        self._components_to_monitor = components_names
        self._service_components = services_components
        self._action_components = action_servers_components
        monitor_name = f"{component_name}_{os.getpid()}"
        super().__init__(
            monitor_name,
            config,
            callback_group,
            start_on_init,
            *args,
            **kwargs,
        )

        # Server nodes handlers
        self._update_parameter_srv_client: Dict[
            str, base_clients.ServiceClientHandler
        ] = {}
        self._update_parameters_srv_client: Dict[
            str, base_clients.ServiceClientHandler
        ] = {}
        self._topic_change_srv_client: Dict[str, base_clients.ServiceClientHandler] = {}
        self._configure_from_yaml_srv_client: Dict[
            str, base_clients.ServiceClientHandler
        ] = {}
        self._main_srv_clients: Dict[str, base_clients.ServiceClientHandler] = {}
        self._main_action_clients: Dict[str, base_clients.ActionClientHandler] = {}

        self._components_to_activate_on_start = activate_on_start

        self._enable_health_monitoring: bool = enable_health_status_monitoring

        self.__components_activation_event: Optional[Callable] = None

    def add_components_activation_event(self, method) -> None:
        """
        Adds a method to be executed when components are activated

        :param method: Method to be executed on components activation
        :type method: Callable
        """
        self.__components_activation_event = method

    def create_all_timers(self) -> None:
        """
        Create all timers
        """
        # Create a timer for components activation
        if self._components_to_activate_on_start:
            callback_group = MutuallyExclusiveCallbackGroup()
            self.__components_monitor_timer = self.create_timer(
                timer_period_sec=1,
                callback=self._check_and_activate_components,
                callback_group=callback_group,
            )
        super().create_all_timers()

    def _check_and_activate_components(self) -> None:
        """
        Checks and activates requested components
        """
        node_names = self.get_node_names()
        components_to_activate_names = (
            [comp.node_name for comp in self._components_to_activate_on_start]
            if self._components_to_activate_on_start
            else []
        )
        if set(components_to_activate_names).issubset(set(node_names)):
            logger.info(f"NODES '{components_to_activate_names}' ARE UP ... ACTIVATING")
            if self.__components_activation_event:
                self.__components_activation_event()
            self.destroy_timer(self.__components_monitor_timer)
        else:
            logger.info(
                f"Waiting for Nodes '{components_to_activate_names}' to come up to activate ..."
            )

    @property
    def events(self):
        """
        Monitored events getter

        :return: Events list
        :rtype: List[Event]
        """
        return self._events_actions.keys()

    def _turn_on_component_management(self, component_name: str) -> None:
        """
        Created clients for all main services in a given component
        - Change a component parameter
        - Change a set of component parameters
        - Replace a topic
        - Reconfigure component from yaml file

        :param component_name: Name of the component (ROS node name)
        :type component_name: str
        """
        self._update_parameter_srv_client[component_name] = (
            base_clients.ServiceClientHandler(
                client_node=self,
                srv_type=ChangeParameter,
                srv_name=f"/{component_name}/update_config_parameter",
            )
        )

        self._update_parameters_srv_client[component_name] = (
            base_clients.ServiceClientHandler(
                client_node=self,
                srv_type=ChangeParameters,
                srv_name=f"{component_name}/update_config_parameters",
            )
        )

        # Input/Output update services

        self._topic_change_srv_client[component_name] = (
            base_clients.ServiceClientHandler(
                client_node=self,
                srv_type=ReplaceTopic,
                srv_name=f"{component_name}/change_topic",
            )
        )

        self._configure_from_yaml_srv_client[component_name] = (
            base_clients.ServiceClientHandler(
                client_node=self,
                srv_type=ConfigureFromYaml,
                srv_name=f"{component_name}/configure_from_yaml",
            )
        )

    def configure_component(
        self,
        component: BaseComponent,
        new_config: Union[object, str],
        keep_alive: bool,
    ) -> None:
        """
        Configure a given component from config instance or config file
        Creates and send the request to the component service

        :param component: Component to configure
        :type component: BaseComponent
        :param config: Config instance or path to config file
        :type config: object | str
        :param keep_alive: To keep the component running while configuring
        :type keep_alive: bool
        :param executor: Used to spin the monitor node until the service response is received, defaults to None
        :type executor: ROS Executor, optional
        """
        try:
            # For config instance prepare change parameters request
            if isinstance(new_config, component.config.__class__):
                request_msg: ChangeParameters.Request = (
                    component.get_change_parameters_msg_from_config(new_config)
                )
                request_msg.keep_alive = keep_alive
                self._update_parameters_srv_client[component.node_name].send_request(
                    request_msg, executor=self.executor
                )
            else:
                # For string send a configure from yaml request
                request_msg_yaml = ConfigureFromYaml.Request()
                request_msg_yaml.path_to_file = new_config
                self._configure_from_yaml_srv_client[component.node_name].send_request(
                    request_msg_yaml, executor=self.executor
                )
        except Exception as e:
            self.get_logger().error(
                f"Unable to configure component {component.node_name}: {e}"
            )

    def update_parameter(
        self,
        component: BaseComponent,
        param_name: str,
        new_value: Any,
        keep_alive: bool = True,
    ) -> None:
        """Sends a ChangeParameter service request to given component

        :param component: _description_
        :type component: BaseComponent
        :param param_name: _description_
        :type param_name: str
        :param new_value: _description_
        :type new_value: Any
        :param keep_alive: _description_, defaults to True
        :type keep_alive: bool, optional
        """
        srv_client: base_clients.ServiceClientHandler = (
            self._update_parameter_srv_client[component.node_name]
        )
        srv_request = ChangeParameter.Request()
        srv_request.name = param_name
        srv_request.value = str(new_value)
        srv_request.keep_alive = keep_alive
        srv_client.send_request(req_msg=srv_request, executor=self.executor)

    def update_parameters(
        self,
        component: BaseComponent,
        params_names: List[str],
        new_values: List,
        keep_alive: bool = True,
        **_,
    ) -> None:
        """Sends a ChangeParameters service request to given component

        :param component: _description_
        :type component: BaseComponent
        :param params_names: _description_
        :type params_names: List[str]
        :param new_values: _description_
        :type new_values: List
        :param keep_alive: _description_, defaults to True
        :type keep_alive: bool, optional
        """
        srv_client: base_clients.ServiceClientHandler = (
            self._update_parameters_srv_client[component.node_name]
        )
        srv_request = ChangeParameters.Request()
        srv_request.names = params_names
        srv_request.values = str(new_values)
        srv_request.keep_alive = keep_alive
        srv_client.send_request(req_msg=srv_request, executor=self.executor)

    def __get_srv_client(
        self, srv_name: str, srv_type: type
    ) -> base_clients.ServiceClientHandler:
        """Helper method to get a service client handler for the provided service name/type

        :param srv_name: Service name
        :type srv_name: str
        :param srv_type: Service type (ROS2 service)
        :type srv_type: type

        :return: Service client handler
        :rtype: base_clients.ServiceClientHandler
        """
        # Check if the client is already created (clients are created for main component services)
        for main_srv_client in self._main_srv_clients.values():
            if (
                main_srv_client.config.name == srv_name
                and main_srv_client.config.srv_type == srv_type
            ):
                return main_srv_client
        # If no return -> service client does not exist -> create it
        return base_clients.ServiceClientHandler(
            client_node=self, srv_name=srv_name, srv_type=srv_type
        )

    def __get_action_client(
        self, action_name: str, action_type: type
    ) -> base_clients.ActionClientHandler:
        """Helper method to get a ros action client handler for the provided service name/type

        :param action_name: Action name
        :type action_name: str
        :param action_type: Action type (ROS2 action)
        :type action_type: type

        :return: Action client handler
        :rtype: base_clients.ActionClientHandler
        """
        # Check if the client is already created (clients are created for main component services)
        for main_action_client in self._main_action_clients.values():
            if (
                main_action_client.config.name == action_name
                and main_action_client.config.action_type == action_type
            ):
                return main_action_client
        # If no return -> service client does not exist -> create it
        return base_clients.ActionClientHandler(
            client_node=self, action_name=action_name, action_type=action_type
        )

    def send_srv_request(
        self, srv_name: str, srv_type: type, srv_request_msg: Any, **_
    ) -> None:
        """Action to send a ROS2 service request during runtime

        :param srv_name: Service name
        :type srv_name: str
        :param srv_type: Service type (ROS2 service)
        :type srv_type: type
        :param srv_request_msg: Service request message
        :type srv_request_msg: Any
        """
        srv_client = self.__get_srv_client(srv_name, srv_type)
        srv_client.send_request(srv_request_msg, executor=self.executor)

    def send_action_goal(
        self, action_name: str, action_type: type, action_request_msg: Any, **_
    ) -> None:
        """Action to send a ROS2 action goal during runtime

        :param action_name: ROS2 action name
        :type action_name: str
        :param action_type: ROS2 action type
        :type action_type: type
        :param action_request_msg: ROS2 action goal message
        :type action_request_msg: Any
        """
        action_client = self.__get_action_client(action_name, action_type)
        action_client.send_request(action_request_msg)

    def publish_message(
        self,
        topic: Topic,
        msg: Any,
        publish_rate: Optional[float] = None,
        publish_period: Optional[float] = None,
        **_,
    ) -> None:
        """Action to publish a message to a given topic

        :param topic: Published topic
        :type topic: Topic
        :param msg: Published message
        :type msg: Any
        :param publish_rate: Publishing rate, if None the message is published once, defaults to None
        :type publish_rate: Optional[float], optional
        :param publish_period: Publishing period, if none and rate is given the message is published forever, defaults to None
        :type publish_period: Optional[float], optional
        """
        publisher: Publisher = self.create_publisher(
            msg_type=topic.ros_msg_type,
            topic=topic.name,
            qos_profile=self.setup_qos(topic.qos_profile),
        )
        # Publish once
        if not publish_rate:
            publisher.publish(msg)
            self.destroy_publisher(publisher)
        elif not publish_period:
            # Publish forever
            self.create_timer(
                timer_period_sec=1 / publish_rate,
                callback=partial(publisher.publish, msg),
            )
        else:
            # Publish with rate for given period
            max_time: float = self.get_secs_time() + publish_period
            timer_name: str = f"timer_{topic.name}_"
            callback = partial(
                self._timer_publish_msg_loop,
                timer_name=timer_name,
                max_time=max_time,
                publisher=publisher,
                msg=msg,
            )
            setattr(
                self,
                timer_name,
                self.create_timer(timer_period_sec=1 / publish_rate, callback=callback),
            )

    def _timer_publish_msg_loop(
        self, timer_name: str, max_time: float, publisher: Publisher, msg: Any
    ) -> None:
        """Timer callback to publish a message until max_time is reached then destroy the timer

        :param max_time: _description_
        :type max_time: float
        :param publisher: _description_
        :type publisher: Publisher
        :param msg: _description_
        :type msg: Any
        """
        time_now: float = self.get_secs_time()
        if time_now > max_time:
            self.destroy_timer(getattr(self, timer_name))
            self.destroy_publisher(publisher)
            return
        publisher.publish(msg)

    def _activate_event_monitoring(self) -> None:
        """
        Turn on all events
        """
        if self._events_actions:
            for event, actions in self._events_actions.items():
                for action in actions:
                    method = getattr(self, action.action_name)
                    # register action to the event
                    action.executable = partial(method, *action.args, **action.kwargs)
                    event.register_actions(action)
                # Create listener to the event trigger topic
                self.create_subscription(
                    msg_type=event.event_topic.ros_msg_type,
                    topic=event.event_topic.name,
                    callback=event.callback,
                    qos_profile=self.setup_qos(event.event_topic.qos_profile),
                    callback_group=MutuallyExclusiveCallbackGroup(),
                )
        if self._internal_events:
            # Turn on monitoring for internal events (to emit back to launcher)
            for event in self._internal_events:
                self.create_subscription(
                    msg_type=event.event_topic.ros_msg_type,
                    topic=event.event_topic.name,
                    callback=event.callback,
                    qos_profile=self.setup_qos(event.event_topic.qos_profile),
                    callback_group=MutuallyExclusiveCallbackGroup(),
                )

    def _create_status_subscribers(self) -> None:
        """
        Creates subscribers to all the health status topics of the components
        """
        # Reentrant group for multi threaded monitoring
        callback_group = ReentrantCallbackGroup()
        for component_name in self._components_to_monitor:
            logger.debug(f"Creating health status subscriber for: {component_name}")
            self.create_subscription(
                ComponentStatus,
                topic=f"{component_name}_status",
                callback=partial(
                    self._status_check_callback, component_name=component_name
                ),
                qos_profile=10,
                callback_group=callback_group,
            )

    def _status_check_callback(self, msg, component_name: str):
        """
        Callback to check the health status of a component

        :param msg: Health status message
        :type msg: ComponentStatus
        :param component: Node under check
        :type component: Component
        """
        # TODO: handle status
        self.get_logger().debug(f"Form {component_name} got status {msg}")

    def create_all_subscribers(self) -> None:
        """
        Create health status subscribers and events subscribers
        """
        if self._components_to_monitor and self._enable_health_monitoring:
            self._create_status_subscribers()
            for component_name in self._components_to_monitor:
                self._turn_on_component_management(component_name)

        self._activate_event_monitoring()

    def create_all_service_clients(self) -> None:
        """Create service clients for all components running as servers"""
        if self._service_components is None:
            return
        for component in self._service_components:
            self.get_logger().info(
                f"Creating Main Service Client for {component.node_name}"
            )
            self._main_srv_clients[component.node_name] = (
                base_clients.ServiceClientHandler(
                    client_node=self,
                    srv_type=component.service_type,
                    srv_name=component.main_srv_name,
                )
            )

    def create_all_action_clients(self) -> None:
        """Create action clients for all components running as action servers"""
        if self._action_components is None:
            return
        for component in self._action_components:
            self.get_logger().info(
                f"Creating Main Action Client for {component.node_name} with name {component.main_action_name} and type {component.action_type}"
            )
            self._main_action_clients[component.node_name] = (
                base_clients.ActionClientHandler(
                    client_node=self,
                    action_type=component.action_type,
                    action_name=component.main_action_name,
                )
            )
