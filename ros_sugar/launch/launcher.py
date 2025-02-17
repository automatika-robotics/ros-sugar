"""Launcher"""
from __future__ import annotations
import os
import inspect
import sys
import socket
from typing import (
    Awaitable,
    Callable,
    Dict,
    Iterable,
    List,
    Optional,
    Union,
    Any,
    Tuple,
)
from concurrent.futures import ThreadPoolExecutor

import msgpack
import msgpack_numpy as m_pack
import launch
import rclpy
from launch import LaunchDescription, LaunchIntrospector, LaunchService
from launch.action import Action as ROSLaunchAction
from launch.actions import (
    ExecuteProcess,
    GroupAction,
    OpaqueCoroutine,
    OpaqueFunction,
    Shutdown,
)
from launch_ros.actions import LifecycleNode as LifecycleNodeLaunchAction
from launch_ros.actions import Node as NodeLaunchAction
from launch_ros.actions import PushRosNamespace
from lifecycle_msgs.msg import Transition
from rclpy import logging
from rclpy.lifecycle.managed_entity import ManagedEntity

from . import logger
from ..core.action import LogInfo
from ..config.base_config import ComponentRunType
from ..core.action import Action
from ..core.component import BaseComponent
from ..core.monitor import Monitor
from ..core.event import OnInternalEvent, Event
from .launch_actions import ComponentLaunchAction
from ..utils import InvalidAction, action_handler, has_decorator, SomeEntitiesType

# Get ROS distro
__installed_distro = os.environ.get("ROS_DISTRO", "").lower()

if __installed_distro in ["humble", "galactic", "foxy"]:
    # Get local copy for older distributions
    from ._lifecycle_transition import LifecycleTransition
else:
    from launch_ros.actions import LifecycleTransition

# patch msgpack for numpy arrays
m_pack.patch()


class Launcher:
    """
    Launcher is a class created to provide a more pythonic way to launch and configure ROS nodes.

    Launcher starts a pre-configured component or a set of components as ROS2 nodes. Launcher can also manage a set of Events-Actions through its internal Monitor node (See Monitor class).

    ## Available options:
    - Provide a ROS2 namespace to all the components
    - Provide a YAML config file.
    - Enable/Disable events monitoring

    Launcher forwards all the provided Events to its internal Monitor, when the Monitor detects an Event trigger it emits an InternalEvent back to the Launcher. Execution of the Action is done directly by the Launcher or a request is forwarded to the Monitor depending on the selected run method (multi-processes or multi-threaded).

    :::{note} While Launcher supports executing standard [ROS2 launch actions](https://github.com/ros2/launch). Launcher does not support standard [ROS2 launch events](https://github.com/ros2/launch/tree/rolling/launch/launch/events) for the current version.
    :::

    """

    def __init__(
        self,
        namespace: str = "",
        config_file: Optional[str] = None,
        enable_monitoring: bool = True,
        activation_timeout: Optional[float] = None,
    ) -> None:
        """Initialize launcher to manager components launch in ROS2

        :param namespace: ROS2 namespace for all the nodes, defaults to ""
        :type namespace: str, optional
        :param config_file: Path to Yaml configuration file, defaults to None
        :type config_file: str | None, optional
        :param enable_monitoring: Enable components health status monitoring, defaults to True
        :type enable_monitoring: bool, optional
        :param activation_timeout: Timeout (seconds) for waiting on ROS2 nodes to come up for activation, defaults to None
        :type activation_timeout: float, optional
        """
        # Make sure RCLPY in initialized
        if not rclpy.ok():
            rclpy.init()

        # Setup launch description
        self._description = LaunchDescription()
        self._description.add_action(PushRosNamespace(namespace=namespace))

        # Create the launch configuration variables
        self._namespace = namespace
        self._config_file: Optional[str] = config_file
        self.__enable_monitoring: bool = enable_monitoring
        self._launch_group = []

        # Components list and package/executable
        self._components: List[BaseComponent] = []
        self._pkg_executable: List[Tuple[Optional[str], Optional[str]]] = []

        # To track each package log level when the pkg is added
        self._pkg_log_level: Dict[str, str] = {}

        # Component: run_in_process (true/false)
        self.__component_names_to_activate_on_start_mp: List[
            str
        ] = []  # List of multiprocessing component names to activate on start by the monitor
        self.__components_to_activate_on_start_threaded: List[
            BaseComponent
        ] = []  # List of threaded component names to activate on start

        # Timeout for activating components on start
        self.__components_activation_timeout = activation_timeout

        # Events/Actions dictionaries
        self._internal_events: Optional[List[Event]] = None
        self._internal_event_names: Optional[List[str]] = None
        self._ros_actions: Dict[str, List[ROSLaunchAction]] = {}
        # Dictionaries {serialized_event: actions}
        self._monitor_actions: Dict[str, List[Action]] = {}
        self._components_actions: Dict[str, List[Action]] = {}
        self.__events_names: List[str] = []

        # Thread pool for external processors
        self._thread_pool: Union[ThreadPoolExecutor, None] = None

    def add_pkg(
        self,
        components: List[BaseComponent],
        package_name: Optional[str] = None,
        executable_entry_point: Optional[str] = "executable",
        events_actions: Dict[
            Event, Union[Action, ROSLaunchAction, List[Union[Action, ROSLaunchAction]]]
        ]
        | None = None,
        multiprocessing: bool = False,
        activate_all_components_on_start: bool = True,
        components_to_activate_on_start: Optional[List[BaseComponent]] = None,
        ros_log_level: Optional[str] = None,
    ):
        """Add component or a set of components to the launcher from one ROS2 package based on ros_sugar

        :param components: Component to launch and manage
        :type components: List[BaseComponent]
        :param package_name: Components ROS2 package name. Required for multi-process run, defaults to None
        :type package_name: str, optional
        :param executable_entry_point: Components ROS2 entry point name. Required for multi-process run, defaults to "executable"
        :type executable_entry_point: str, optional
        :param events_actions: Events/Actions to monitor, defaults to None
        :type events_actions: Dict[ Event, Union[Action, ROSLaunchAction, List[Union[Action, ROSLaunchAction]]] ] | None, optional
        :param multiprocessing: Run the components in multi-processes, otherwise runs in multi-threading, defaults to False
        :type multiprocessing: bool, optional
        :param activate_all_components_on_start: To activate all the ROS2 lifecycle nodes on bringup, defaults to False
        :type activate_all_components_on_start: bool, optional
        :param components_to_activate_on_start: Set of components to activate on bringup, defaults to None
        :type components_to_activate_on_start: Optional[List[BaseComponent]], optional
        :param ros_log_level: Selected ROS logging level for the package components, defaults to None
        :type ros_log_level: str, optional
        """
        # If multi processing is enabled -> check for package and executable name
        if multiprocessing and (not package_name or not executable_entry_point):
            raise ValueError(
                "Cannot run in multi-processes without specifying ROS2 'package_name' and 'executable_entry_point'"
            )

        package_name = package_name if multiprocessing else None
        executable_entry_point = executable_entry_point if multiprocessing else None

        # Extend existing components
        self._components.extend(components)
        self._pkg_executable.extend(
            [(package_name, executable_entry_point)] * len(components)
        )

        # Register which components to activate on start
        if components_to_activate_on_start:
            if multiprocessing:
                self.__component_names_to_activate_on_start_mp.extend([
                    component.node_name for component in components_to_activate_on_start
                ])
            else:
                self.__components_to_activate_on_start_threaded.extend(
                    components_to_activate_on_start
                )

        elif activate_all_components_on_start:
            if multiprocessing:
                self.__component_names_to_activate_on_start_mp.extend([
                    component.node_name for component in components
                ])
            else:
                self.__components_to_activate_on_start_threaded.extend(components)

        # Parse provided Events/Actions
        if events_actions and self.__enable_monitoring:
            # Rewrite the actions dictionary and updates actions to be passed to the monitor and to the components
            self.__rewrite_actions_for_components(components, events_actions)

        # Configure components from config_file
        for component in components:
            if ros_log_level:
                self._pkg_log_level[component.node_name] = ros_log_level
            if self._config_file:
                component._config_file = self._config_file
                component.config_from_yaml(self._config_file)

    def _setup_component_events_handlers(self, comp: BaseComponent):
        """Parse a component events/actions from the overall components actions

        :param comp: Component
        :type comp: BaseComponent
        """
        if not self._components_actions:
            return
        comp_dict = {}
        for event_serialized, actions in self._components_actions.items():
            for action in actions:
                if comp.node_name == action.parent_component:
                    self.__update_dict_list(comp_dict, event_serialized, action)
        if comp_dict:
            comp.events_actions = comp_dict

    def __update_dict_list(self, dictionary: Dict[str, List], name: str, value: Any):
        """Helper method to add or update an item in a dictionary

        :param dictionary: Dictionary to be updated
        :type dictionary: Dict[Any, List]
        :param name: Item key
        :type name: Any
        :param value: Item value
        :type value: Any
        """
        if dictionary.get(name):
            dictionary[name].append(value)
        else:
            dictionary[name] = [value]

    def __rewrite_actions_for_components(
        self,
        components_list: List[BaseComponent],
        actions_dict: Dict[
            Event, Union[Action, ROSLaunchAction, List[Union[Action, ROSLaunchAction]]]
        ],
    ):
        """
        Rewrites an event/action dictionary against available components

        :param components_list: List of all available components
        :type components_list: List[BaseComponent]
        :param actions_dict: Event/Action dictionary
        :type actions_dict: Dict[Event, Action]

        :raises ValueError: If given component action corresponds to unknown component
        """
        self.__events_names.extend(event.name for event in actions_dict)
        for condition, raw_action in actions_dict.items():
            serialized_condition: str = condition.json
            action_set: List[Union[Action, ROSLaunchAction]] = (
                raw_action if isinstance(raw_action, list) else [raw_action]
            )
            for action in action_set:
                # Check if it is a component action:
                if isinstance(action, Action) and action.component_action:
                    action_object = action.executable.__self__
                    if components_list.count(action_object) <= 0:
                        raise InvalidAction(
                            f"Invalid action for condition '{condition.name}'. Action component '{action_object}' is unknown or not added to Launcher"
                        )
                    self.__update_dict_list(
                        self._components_actions, serialized_condition, action
                    )
                elif isinstance(action, Action) and action.monitor_action:
                    # Action to execute through the monitor
                    self.__update_dict_list(
                        self._monitor_actions, serialized_condition, action
                    )
                elif isinstance(action, Action) or isinstance(action, ROSLaunchAction):
                    # If it is a valid ROS launch action -> nothing is required
                    self.__update_dict_list(self._ros_actions, condition.name, action)
                    if not self._internal_events:
                        self._internal_events = [condition]
                    elif condition not in self._internal_events:
                        self._internal_events.append(condition)

    def _activate_components_action(self) -> SomeEntitiesType:
        """
        Activate all the components in the stack

        :param in_processes: Components run type, If false then run type is in threads
        :type in_processes: bool
        """
        activation_actions = []
        for component_name in self.__component_names_to_activate_on_start_mp:
            activation_actions.extend(self.start(component_name))

        for component in self.__components_to_activate_on_start_threaded:
            start_action = Action(component.start)
            activation_actions.append(start_action.launch_action())
        return activation_actions

    # LAUNCH ACTION HANDLERS
    @action_handler
    def start(self, node_name: str, **_) -> SomeEntitiesType:
        """
        Action to start a node: configure + activate

        :param node_name: _description_
        :type node_name: str
        :return: Launch actions
        :rtype: List[SomeEntitiesType]
        """
        actions = [
            LifecycleTransition(
                lifecycle_node_names=[node_name],
                transition_ids=[
                    Transition.TRANSITION_CONFIGURE,
                    Transition.TRANSITION_ACTIVATE,
                ],
            )
        ]
        return actions

    @action_handler
    def stop(self, node_name: str, **_) -> SomeEntitiesType:
        """
        Action to stop a node: deactivate

        :param node_name: _description_
        :type node_name: str
        :return: Launch actions
        :rtype: List[SomeEntitiesType]
        """
        actions = [
            LifecycleTransition(
                lifecycle_node_names=[node_name],
                transition_ids=[Transition.TRANSITION_DEACTIVATE],
            )
        ]
        return actions

    @action_handler
    def restart(self, node_name: str, **_) -> SomeEntitiesType:
        """
        Action to restart a node: deactivate + activate

        :param node_name: _description_
        :type node_name: str
        :return: Launch actions
        :rtype: List[SomeEntitiesType]
        """
        actions = [
            LifecycleTransition(
                lifecycle_node_names=[node_name],
                transition_ids=[
                    Transition.TRANSITION_DEACTIVATE,
                    Transition.TRANSITION_ACTIVATE,
                ],
            )
        ]
        return actions

    # FALLBACKS
    @property
    def fallback_rate(self) -> Dict:
        """fallback_rate.

        :rtype: Dict
        """
        return {
            component.node_name: component.fallback_rate
            for component in self._components
        }

    @fallback_rate.setter
    def fallback_rate(self, value: float) -> None:
        """
        Set the fallback rate for all components

        :param value: Fallback check rate (Hz)
        :type value: float
        """
        for component in self._components:
            component.fallback_rate = value

    def on_fail(self, action_name: str, max_retries: Optional[int] = None) -> None:
        """
        Set the fallback strategy (action) on any fail for all components

        :param action: Action to be executed on failure
        :type action: Union[List[Action], Action]
        :param max_retries: Maximum number of action execution retries. None is equivalent to unlimited retries, defaults to None
        :type max_retries: Optional[int], optional
        """
        for component in self._components:
            if action_name in component.fallbacks:
                method = getattr(component, action_name)
                method_params = inspect.signature(method).parameters
                if any(
                    x.default is inspect.Parameter.empty for x in method_params.values()
                ):
                    raise ValueError(
                        f"{method} takes {method_params} as arguments. Only actions without any arguments or with keyword only arguments can be set as on_fail actions from the launcher. Use component.on_fail to pass specific arguments."
                    )
                action = Action(method=method)
                component.on_fail(action, max_retries)
            else:
                raise ValueError(
                    f"Non valid action fallback {action_name}: Fallback is not available in component {component.node_name}. Available component fallbacks are the following methods: '{component.fallbacks}'"
                )

    def _get_action_launch_entity(self, action: Action) -> SomeEntitiesType:
        """Gets the action launch entity for a given Action.

        :param action:
        :type action: Action
        :rtype: SomeEntitiesType
        """
        try:
            action_method = getattr(self, action.action_name)
            if not has_decorator(action_method, "action_handler"):
                raise InvalidAction(
                    f"Requested action method {action.action_name} is not a valid event handler"
                )

        except AttributeError as e:
            raise InvalidAction(
                f"Requested unavailable component action: {action.parent_component}.{action.action_name}"
            ) from e
        comp = None
        for comp in self._components:
            if comp.node_name == action.parent_component:
                break
        if not comp:
            raise InvalidAction(
                f"Requested action component {action.parent_component} is unknown"
            )
        return action_method(
            *action.args,
            **action.kwargs,
            node_name=action.parent_component,
            component=comp,
        )

    def _setup_internal_events_handlers(self, nodes_in_processes: bool = True) -> None:
        """Sets up the launch handlers for all internal events.

        :param nodes_in_processes:
        :type nodes_in_processes: bool
        :rtype: None
        """
        # Add event handling actions
        entities_dict: Dict = {}

        if not self._ros_actions:
            return
        for event_name, action_set in self._ros_actions.items():
            log_action = LogInfo(msg=f"GOT TRIGGER FOR EVENT {event_name}")
            entities_dict[event_name] = [log_action]
            for action in action_set:
                if isinstance(action, ROSLaunchAction):
                    entities_dict[event_name].append(action)

                # Check action type
                elif action.component_action and nodes_in_processes:
                    # Re-parse action for component related actions
                    entities = self._get_action_launch_entity(action)
                    if isinstance(entities, list):
                        entities_dict[event_name].extend(entities)
                    else:
                        entities_dict[event_name].append(entities)

                # If the action is not related to a component -> add opaque executable to launch
                else:
                    entities_dict[event_name].append(
                        action.launch_action(monitor_node=self.monitor_node)
                    )

            # Register a new internal event handler
            internal_events_handler = launch.actions.RegisterEventHandler(
                OnInternalEvent(
                    internal_event_name=event_name,
                    entities=entities_dict[event_name],
                )
            )
            self._description.add_action(internal_events_handler)

    def _setup_monitor_node(self, nodes_in_processes: bool = True) -> None:
        """Adds a node to monitor all the launched components and their events

        :param nodes_in_processes: If nodes are being launched in separate processes, defaults to True
        :type nodes_in_processes: bool, optional
        """
        # Update internal events
        if self._internal_events:
            self._internal_event_names = [ev.name for ev in self._internal_events]
            # Check that all internal events have unique names
            if len(set(self._internal_event_names)) != len(self._internal_event_names):
                raise ValueError(
                    "Got duplicate events names. Provide unique names for all your events"
                )

        # Get components running as servers to create clients in Monitor
        services_components = [
            comp
            for comp in self._components
            if comp.run_type == ComponentRunType.SERVER
        ]
        action_components = [
            comp
            for comp in self._components
            if comp.run_type == ComponentRunType.ACTION_SERVER
        ]

        # Setup the monitor node
        components_names = [comp.node_name for comp in self._components]

        # Check that all components have unique names
        if len(set(components_names)) != len(components_names):
            raise ValueError(
                f"Got duplicate component names in: {components_names}. Cannot launch components with duplicate names. Provide unique names for all your components"
            )

        all_components_to_activate_on_start = (
            self.__component_names_to_activate_on_start_mp
            + [
                comp.node_name
                for comp in self.__components_to_activate_on_start_threaded
            ]
        )

        self.monitor_node = Monitor(
            components_names=components_names,
            enable_health_status_monitoring=self.__enable_monitoring,
            events_actions=self._monitor_actions,
            events_to_emit=self._internal_events,
            services_components=services_components,
            action_servers_components=action_components,
            activate_on_start=all_components_to_activate_on_start,
            activation_timeout=self.__components_activation_timeout,
        )

        monitor_action = ComponentLaunchAction(
            node=self.monitor_node,
            namespace=self._namespace,
            name=self.monitor_node.node_name,
        )
        self._description.add_action(monitor_action)

        # Register a activation event
        internal_events_handler_activate = launch.actions.RegisterEventHandler(
            OnInternalEvent(
                internal_event_name="activate_all",
                entities=self._activate_components_action(),
            )
        )
        self._description.add_action(internal_events_handler_activate)

        # Register exit_all event
        exit_all_event_handler = launch.actions.RegisterEventHandler(
            OnInternalEvent(
                internal_event_name="exit_all",
                entities=[
                    Shutdown(
                        reason="Shutting down all nodes due to a detected problem from the system Monitor"
                    )
                ],
            )
        )
        self._description.add_action(exit_all_event_handler)

        self._setup_internal_events_handlers(nodes_in_processes)

    def __listen_for_external_processing(self, sock: socket.socket, func: Callable):
        # Block to accept connections
        conn, _ = sock.accept()
        logger.info(f"EXTERNAL PROCESSOR CONNECTED ON {conn}")
        while True:
            # TODO: Make the buffer size a parameter
            # Block to receive data
            data = conn.recv(1024)
            if not data:
                continue
            # TODO: Retrieve errors
            data = msgpack.unpackb(data)
            result = func(**data)
            logger.debug(f"Got result from external processor: {result}")
            result = msgpack.packb(result)
            conn.sendall(result)

    def _setup_external_processors(self, component: BaseComponent) -> None:
        if not component._external_processors:
            return

        if not self._thread_pool:
            self._thread_pool = ThreadPoolExecutor()

        for key, processor_data in component._external_processors.items():
            for processor in processor_data[0]:
                sock_file = (
                    f"/tmp/{component.node_name}_{key}_{processor.__name__}.socket"  # type: ignore
                )
                if os.path.exists(sock_file):
                    os.remove(sock_file)

                s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                s.bind(sock_file)
                s.listen(0)
                self._thread_pool.submit(
                    self.__listen_for_external_processing, s, processor
                )  # type: ignore

    def _setup_component_in_process(
        self,
        component: BaseComponent,
        pkg_name: str,
        executable_name: str,
        ros_log_level: str = "info",
    ):
        """
        Sets up the launch actions to start the components in separate processes

        :param ros_log_level: Log level for ROS2
        :type ros_log_level: str, default to "info"
        """
        name = component.node_name
        component._update_cmd_args_list()
        self._setup_external_processors(component)
        ros_log_level = (
            self._pkg_log_level[component.node_name]
            if component.node_name in self._pkg_log_level
            else ros_log_level
        )
        # Check if the component is a lifecycle node
        if issubclass(component.__class__, ManagedEntity):
            new_node = LifecycleNodeLaunchAction(
                package=pkg_name,
                exec_name=name,
                namespace=self._namespace,
                name=name,
                executable=executable_name,
                output="screen",
                arguments=component.launch_cmd_args
                + ["--ros-args", "--log-level", ros_log_level],
            )
        else:
            new_node = NodeLaunchAction(
                package=pkg_name,
                exec_name=name,
                namespace=self._namespace,
                name=name,
                executable=executable_name,
                output="screen",
                arguments=component.launch_cmd_args
                + ["--ros-args", "--log-level", ros_log_level],
            )

        self._launch_group.append(new_node)

    def _setup_component_in_thread(self, component, ros_log_level: str = "info"):
        """
        Adds all components to be launched in separate threads
        """
        ros_log_level = (
            self._pkg_log_level[component.node_name]
            if component.node_name in self._pkg_log_level
            else ros_log_level
        )
        component_action = ComponentLaunchAction(
            node=component,
            namespace=self._namespace,
            name=component.node_name,
            output="screen",
            log_level=logging.get_logging_severity_from_string(ros_log_level),
        )
        self._launch_group.append(component_action)

    def _start_ros_launch(self, introspect: bool = True, debug: bool = False):
        """
        Launch all ros nodes

        :param introspect: start LaunchIntrospector, defaults to True
        :type introspect: bool, optional
        :param debug: LaunchService debugger, defaults to True
        :type debug: bool, optional
        """
        if introspect:
            logger.info("-----------------------------------------------")
            logger.info("Starting introspection of launch description...")
            logger.info("-----------------------------------------------")
            logger.info(
                LaunchIntrospector().format_launch_description(self._description)
            )

        logger.info("------------------------------------")
        logger.info("Starting Launch of All Components...")
        logger.info("------------------------------------")

        self.ls = LaunchService(debug=debug)
        self.ls.include_launch_description(self._description)

        self.ls.run(shutdown_when_idle=False)

    def configure(
        self,
        config_file: str,
        component_name: str | None = None,
    ):
        """
        Configure components managed by the Orchestrator

        :param config_file: Path to configuration file (yaml)
        :type config_file: str
        :param component_name: Configure one component with given name, defaults to None
        :type component_name: str | None, optional
        """
        # Configure one component with given name

        if component_name:
            for component in self._components:
                if component.node_name == component_name:
                    component.config_from_yaml(config_file)
            return

        # If no component is specified -> configure all components
        for component in self._components:
            component.config_from_yaml(config_file)

    def add_py_executable(self, path_to_executable: str, name: str = "python3"):
        """
        Adds a python executable to the launcher as a separate process

        :param path_to_executable: _description_
        :type path_to_executable: str
        :param name: _description_, defaults to 'python3'
        :type name: str, optional
        """
        exec_process = ExecuteProcess(
            cmd=[sys.executable, path_to_executable], name=name
        )

        self._description.add_action(exec_process)

    def add_method(
        self,
        method: Callable | Awaitable,
        args: Iterable | None = None,
        kwargs: Dict | None = None,
    ):
        """
        Adds a method action to launch

        :param method: _description_
        :type method: Callable | Awaitable
        :param args: _description_, defaults to None
        :type args: Iterable | None, optional
        :param kwargs: _description_, defaults to None
        :type kwargs: Dict | None, optional
        """
        if inspect.iscoroutine(method):
            method_action = OpaqueCoroutine(coroutine=method, args=args, kwargs=kwargs)
        else:
            method_action = OpaqueFunction(function=method, args=args, kwargs=kwargs)
        self._description.add_action(method_action)

    def _check_duplicate_names(self) -> None:
        """Checks for components/events with duplicate names in the launcher

        :raises ValueError: If two components or events are found with the same name
        """
        for i in range(len(self._components) - 1):
            if self._components[i].node_name == self._components[i + 1].node_name:
                error_msg = f"Found duplicate component name: '{self._components[i].node_name}'. Please use unique names for all your components to avoid duplicate ROS2 node names"
                logger.exception(error_msg)
                raise ValueError(error_msg)

        for i in range(len(self.__events_names) - 1):
            if self.__events_names[i] == self.__events_names[i + 1]:
                error_msg = f"Found duplicate event name: '{self.__events_names[i]}'. Please use unique names for all your events"
                logger.exception(error_msg)
                raise ValueError(error_msg)

    def setup_launch_description(
        self,
        ros_log_level: str = "info",
    ):
        self._check_duplicate_names()

        # SET PROCESS NAME (if setproctitle is available)
        try:
            import setproctitle
            setproctitle.setproctitle(logger.name)
        except ImportError:
            pass

        self._setup_monitor_node()

        for component in self._components:
            self._setup_component_events_handlers(component)

        # Add configured components to launcher
        for idx, component in enumerate(self._components):
            pkg_name, executable_name = self._pkg_executable[idx]
            if pkg_name and executable_name:
                self._setup_component_in_process(
                    component, pkg_name, executable_name, ros_log_level
                )
            else:
                self._setup_component_in_thread(component, ros_log_level)

        group_action = GroupAction(self._launch_group)

        self._description.add_action(group_action)

    def bringup(
        self,
        config_file: str | None = None,
        introspect: bool = False,
        launch_debug: bool = False,
        ros_log_level: str = "info",
    ):
        """
        Bring up the Launcher
        """
        if not self._components:
            raise ValueError(
                "Cannot bringup without adding any components. Use 'add_pkg' method to add a set of components from one ROS2 package then use 'bringup' to start and run your system"
            )

        if config_file:
            self.configure(config_file)

        self.setup_launch_description(ros_log_level)

        self._start_ros_launch(introspect, launch_debug)

        if self._thread_pool:
            self._thread_pool.shutdown()

        logger.info("------------------------------------")
        logger.info("ALL COMPONENTS ENDED")
        logger.info("------------------------------------")
