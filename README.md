<picture>
  <source media="(prefers-color-scheme: dark)" srcset="docs/_static/ROS_SUGAR.png">
  <source media="(prefers-color-scheme: light)" srcset="docs/_static/ROS_SUGAR_DARK.png">
  <img alt="ROS Sugar Logo." src="docs/_static/ROS_SUGAR_DARK.png"  width="50%">
</picture>


ROS SUGAR 🍬 provides a whole lot of syntactic sugar for creating multinode ROS2 event-driven systems and management using an intuitive Python API.

- Learn more about the [**design concepts**](https://automatika-robotics.github.io/ros-sugar/design/index.html) in ROS Sugar 📚
- Learn how to [**create your own ROS2 package**](https://automatika-robotics.github.io/ros-sugar/use.html) using ROS Sugar 🚀

> [!NOTE]
> This is an alpha release of ROS Sugar. Breaking changes are to be expected.

## Packages created using ROS Sugar

- [**Kompass**](https://automatikarobotics.com/kompass/): a framework for building robust and comprehensive event-driven navigation stacks using an easy-to-use and intuitive Python API
- [**ROS Agents**](https://automatika-robotics.github.io/ros-agents/): a fully-loaded framework for creating interactive embodied agents that can understand, remember, and act upon contextual information from their environment.

## Overview

ROS Sugar is built for ROS2 developers who want to create robust, event-driven systems with multiple nodes that are easy to use and can be configured and started with an intuitive python API. It provides primitives for writing ROS nodes and events/actions which can start/stop/modify the nodes, in the spirit of event driven software standard. ROS Sugar is also a replacement for the ROS Launch API.

A [Component](https://automatika-robotics.github.io/ros-sugar/design/component.html) is the main execution unit in ROS Sugar, each component is configured with [Inputs/Outputs](https://automatika-robotics.github.io/ros-sugar/design/topics.md) and [Fallback](https://automatika-robotics.github.io/ros-sugar/design/fallbacks.html) behaviors. Additionally, each component updates its own [Health Status](https://automatika-robotics.github.io/ros-sugar/design/status.html). Components can be handled and reconfigured dynamically at runtime using [Events](https://automatika-robotics.github.io/ros-sugar/design/events.html) and [Actions](https://automatika-robotics.github.io/ros-sugar/design/actions.html). Events, Actions and Components are passed to the [Launcher](https://automatika-robotics.github.io/ros-sugar/design/launcher.html) which runs the set of components as using multi-threaded or multi-process execution. The Launcher also uses an internal [Monitor](https://automatika-robotics.github.io/ros-sugar/design/monitor.html) to keep track of the components and monitor events.


<img src="docs/_static/images/diagrams/component.jpg" alt="Base Component" width="700px">


<img src="docs/_static/images/diagrams/multi_threaded.jpg" alt="Multi-threaded execution" width="500px">


<img src="docs/_static/images/diagrams/multi_process.jpg" alt="Multi-process execution" width="500px">


## Building from source

- ``` mkdir -p ros-sugar-ws/src```
- ``` cd ros-sugar-ws/src```
- ``` git clone https://github.com/automatika-robotics/ros-sugar```
- ``` cd ..```
- ``` colcon build --symlink-install```

## Copyright

The code in this distribution is Copyright (c) 2024 Automatika Robotics unless explicitly indicated otherwise.

ROS Sugar is made available under the MIT license. Details can be found in the [LICENSE](LICENSE) file.

## Contributions

ROS Agents has been developed in collaboration between [Automatika Robotics](https://automatikarobotics.com/) and [Inria](https://inria.fr/). Contributions from the community are most welcome.
