import argparse
import logging
from typing import Optional, List, Type

import rclpy
import setproctitle
from rclpy.executors import MultiThreadedExecutor
from rclpy.utilities import try_shutdown

from ros_sugar.io import Topic


def _parse_args() -> tuple[argparse.Namespace, List[str]]:
    """Parse arguments."""
    parser = argparse.ArgumentParser(description="Component Executable Config")
    parser.add_argument(
        "--config_type", type=str, help="Component configuration class name"
    )
    parser.add_argument("--component_type", type=str, help="Component class name")
    parser.add_argument(
        "--node_name",
        type=str,
        help="Component ROS2 node name",
    )
    parser.add_argument("--config", type=str, help="Component configuration object")
    parser.add_argument(
        "--inputs",
        type=str,
        help="Component input topics",
    )
    parser.add_argument(
        "--outputs",
        type=str,
        help="Component output topics",
    )

    parser.add_argument(
        "--config_file", type=str, help="Path to configuration YAML file"
    )
    parser.add_argument(
        "--events", type=str, help="Events to be monitored by the component"
    )
    parser.add_argument(
        "--actions", type=str, help="Actions associated with the component Events"
    )
    parser.add_argument(
        "--external_processors",
        type=str,
        help="External processors associated with the component input and output topics",
    )
    return parser.parse_known_args()


def _parse_component_config(
    args: argparse.Namespace, config_classes: List[Type]
) -> Optional[object]:
    """Parse the component config object

    :param args: Command line arguments
    :type args: argparse.Namespace

    :return: Component config object
    :rtype: object
    """
    config_type = args.config_type or None
    config_class = None
    if not config_type:
        logging.warning(
            f"Unknown config_type '{config_type}'. Proceeding with default config for component"
        )
        return None

    # Get config type and update from json arg
    config_class = next(
        (conf_cls for conf_cls in config_classes if conf_cls.__name__ == config_type),
        None,
    )
    if not config_class:
        logging.warning(
            f"Unknown config_type '{config_type}'. Proceeding with default config for component"
        )
        return None

    config_json = args.config

    config = config_class(**config_json) if config_json else config_class()

    return config


def _parse_ros_args(args_names: list[str]) -> list[str]:
    """Parse ROS arguments from command line arguments

    :param args_names: List of all parsed arguments
    :type args_names: list[str]

    :return: List ROS parsed arguments
    :rtype: list[str]
    """
    # Look for --ros-args in ros_args
    ros_args_start = None
    if "--ros-args" in args_names:
        ros_args_start = args_names.index("--ros-args")

    if ros_args_start is not None:
        ros_specific_args = args_names[ros_args_start:]
    else:
        ros_specific_args = []
    return ros_specific_args


def executable_main(*, list_of_components: List[Type], list_of_configs: List[Type]):
    """Executable main function to run a component as a ROS2 node in a new process.
    Used to start a node using Launcher

    To use in your custom package based on ros_sugar:
    - Create an executable.py with a main and import your custom components and their config classes in it:
        ```python
            #!/usr/bin/env python3
            from ros_sugar import executable_main

            my_components_list = ...
            my_configs_list = ...

            def main(args=None):
                executable_main(list_of_components=my_components_list, list_of_configs=my_configs_list)
        ```
    - Add your executable as an entry point in your custom package setup.py:
        ```python
            from setuptools import find_packages, setup

            package_name = "my_awesome_pkg"

            console_scripts = [
                "executable = my_awesome_pkg.executable:main",
            ]

            setup(
                name=package_name,
                version="1",
                packages=find_packages(),
                install_requires=["setuptools"],
                zip_safe=True,
                maintainer="Cyberdyne Systems",
                maintainer_email="contact@cyberdynesystems.com",
                description="My awesome ROS2 sugar package",
                entry_points={
                    "console_scripts": console_scripts,
                },
            )
        ```
    - Provide your package name and your entry point name to ros_sugar launcher in your script.

    :param list_of_components: List of all known Component classes in the package
    :type list_of_components: List[Type]
    :param list_of_configs: List of all known ComponentConfig classes in the package
    :type list_of_configs: List[Type]
    :raises ValueError: If component or component config are unknown classes
    :raises ValueError: If component cannot be started with provided arguments
    """
    args, args_names = _parse_args()

    # Initialize rclpy with the ros-specific arguments
    rclpy.init(args=_parse_ros_args(args_names))

    component_type = args.component_type or None

    if not component_type:
        raise ValueError("Cannot launch withput providing a component_type")

    comp_class = next(
        (
            comp_cls
            for comp_cls in list_of_components
            if comp_cls.__name__ == component_type
        ),
        None,
    )

    if not comp_class:
        raise ValueError(
            f"Cannot launch unknown component type '{component_type}'. Known types are: '{list_of_components}'"
        )

    # Get name
    component_name = args.node_name or None

    if not component_name:
        raise ValueError("Cannot launch component without specifying a name")

    # SET PROCESS NAME
    setproctitle.setproctitle(component_name)

    config = _parse_component_config(args, list_of_configs)

    # Get Yaml config file if provided
    config_file = args.config_file or None

    # Get inputs/outputs
    inputs = [Topic(**i) for i in args.inputs] if args.inputs else None
    outputs = [Topic(**o) for o in args.outputs] if args.outputs else None

    # Init the component
    component = comp_class(
        inputs=inputs,
        outputs=outputs,
        config=config,
        component_name=component_name,
        config_file=config_file,
    )

    # Init the node with rclpy
    component.rclpy_init_node()

    # Set events/actions
    events_json = args.events or None
    actions_json = args.actions or None

    if events_json and actions_json:
        component._events_json = events_json
        component._actions_json = actions_json

    # Set external processors
    external_processors = args.external_processors or None
    if external_processors:
        component._external_processors_json = external_processors

    executor = MultiThreadedExecutor()

    executor.add_node(component)

    try:
        executor.spin()

    except KeyboardInterrupt:
        pass

    finally:
        executor.remove_node(component)
        try_shutdown()
