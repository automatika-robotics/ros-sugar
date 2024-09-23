---
orphan: true
---

# {py:mod}`ros_sugar.core.node`

```{py:module} ros_sugar.core.node
```

```{autodoc2-docstring} ros_sugar.core.node
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`BaseNode <ros_sugar.core.node.BaseNode>`
  - ```{autodoc2-docstring} ros_sugar.core.node.BaseNode
    :summary:
    ```
````

### API

`````{py:class} BaseNode(node_name: str, node_config: typing.Optional[ros_sugar.config.BaseConfig] = None, callback_group=None, start_on_init: bool = True, *args, **kwargs)
:canonical: ros_sugar.core.node.BaseNode

Bases: {py:obj}`rclpy.node.Node`

```{autodoc2-docstring} ros_sugar.core.node.BaseNode
```

````{py:method} rclpy_init_node(*args, **kwargs)
:canonical: ros_sugar.core.node.BaseNode.rclpy_init_node

```{autodoc2-docstring} ros_sugar.core.node.BaseNode.rclpy_init_node
```

````

````{py:method} configure(config_file: str)
:canonical: ros_sugar.core.node.BaseNode.configure

```{autodoc2-docstring} ros_sugar.core.node.BaseNode.configure
```

````

````{py:method} add_execute_once(method: typing.Callable)
:canonical: ros_sugar.core.node.BaseNode.add_execute_once

```{autodoc2-docstring} ros_sugar.core.node.BaseNode.add_execute_once
```

````

````{py:method} add_execute_in_loop(method: typing.Callable)
:canonical: ros_sugar.core.node.BaseNode.add_execute_in_loop

```{autodoc2-docstring} ros_sugar.core.node.BaseNode.add_execute_in_loop
```

````

````{py:method} get_ros_time() -> builtin_interfaces.msg.Time
:canonical: ros_sugar.core.node.BaseNode.get_ros_time

```{autodoc2-docstring} ros_sugar.core.node.BaseNode.get_ros_time
```

````

````{py:method} get_secs_time() -> float
:canonical: ros_sugar.core.node.BaseNode.get_secs_time

```{autodoc2-docstring} ros_sugar.core.node.BaseNode.get_secs_time
```

````

````{py:property} launch_cmd_args
:canonical: ros_sugar.core.node.BaseNode.launch_cmd_args
:type: typing.List[str]

```{autodoc2-docstring} ros_sugar.core.node.BaseNode.launch_cmd_args
```

````

````{py:property} config_json
:canonical: ros_sugar.core.node.BaseNode.config_json
:type: typing.Union[str, bytes]

```{autodoc2-docstring} ros_sugar.core.node.BaseNode.config_json
```

````

````{py:method} activate()
:canonical: ros_sugar.core.node.BaseNode.activate

```{autodoc2-docstring} ros_sugar.core.node.BaseNode.activate
```

````

````{py:method} deactivate()
:canonical: ros_sugar.core.node.BaseNode.deactivate

```{autodoc2-docstring} ros_sugar.core.node.BaseNode.deactivate
```

````

````{py:method} setup_qos(qos_policy: ros_sugar.config.QoSConfig) -> rclpy.qos.QoSProfile
:canonical: ros_sugar.core.node.BaseNode.setup_qos

```{autodoc2-docstring} ros_sugar.core.node.BaseNode.setup_qos
```

````

````{py:method} init_flags()
:canonical: ros_sugar.core.node.BaseNode.init_flags

```{autodoc2-docstring} ros_sugar.core.node.BaseNode.init_flags
```

````

````{py:method} init_variables()
:canonical: ros_sugar.core.node.BaseNode.init_variables

```{autodoc2-docstring} ros_sugar.core.node.BaseNode.init_variables
```

````

````{py:method} create_tf_listener(tf_config: ros_sugar.tf.TFListenerConfig) -> ros_sugar.tf.TFListener
:canonical: ros_sugar.core.node.BaseNode.create_tf_listener

```{autodoc2-docstring} ros_sugar.core.node.BaseNode.create_tf_listener
```

````

````{py:method} create_client(*args, **kwargs) -> rclpy.client.Client
:canonical: ros_sugar.core.node.BaseNode.create_client

```{autodoc2-docstring} ros_sugar.core.node.BaseNode.create_client
```

````

````{py:method} create_all_subscribers()
:canonical: ros_sugar.core.node.BaseNode.create_all_subscribers

```{autodoc2-docstring} ros_sugar.core.node.BaseNode.create_all_subscribers
```

````

````{py:method} create_all_publishers()
:canonical: ros_sugar.core.node.BaseNode.create_all_publishers

```{autodoc2-docstring} ros_sugar.core.node.BaseNode.create_all_publishers
```

````

````{py:method} create_all_services()
:canonical: ros_sugar.core.node.BaseNode.create_all_services

```{autodoc2-docstring} ros_sugar.core.node.BaseNode.create_all_services
```

````

````{py:method} create_all_service_clients()
:canonical: ros_sugar.core.node.BaseNode.create_all_service_clients

```{autodoc2-docstring} ros_sugar.core.node.BaseNode.create_all_service_clients
```

````

````{py:method} create_all_action_servers()
:canonical: ros_sugar.core.node.BaseNode.create_all_action_servers

```{autodoc2-docstring} ros_sugar.core.node.BaseNode.create_all_action_servers
```

````

````{py:method} create_all_action_clients()
:canonical: ros_sugar.core.node.BaseNode.create_all_action_clients

```{autodoc2-docstring} ros_sugar.core.node.BaseNode.create_all_action_clients
```

````

````{py:method} create_all_timers()
:canonical: ros_sugar.core.node.BaseNode.create_all_timers

```{autodoc2-docstring} ros_sugar.core.node.BaseNode.create_all_timers
```

````

````{py:method} destroy_all_subscribers()
:canonical: ros_sugar.core.node.BaseNode.destroy_all_subscribers

```{autodoc2-docstring} ros_sugar.core.node.BaseNode.destroy_all_subscribers
```

````

````{py:method} destroy_all_publishers()
:canonical: ros_sugar.core.node.BaseNode.destroy_all_publishers

```{autodoc2-docstring} ros_sugar.core.node.BaseNode.destroy_all_publishers
```

````

````{py:method} destroy_all_services()
:canonical: ros_sugar.core.node.BaseNode.destroy_all_services

```{autodoc2-docstring} ros_sugar.core.node.BaseNode.destroy_all_services
```

````

````{py:method} destroy_all_action_servers()
:canonical: ros_sugar.core.node.BaseNode.destroy_all_action_servers

```{autodoc2-docstring} ros_sugar.core.node.BaseNode.destroy_all_action_servers
```

````

````{py:method} destroy_all_action_clients()
:canonical: ros_sugar.core.node.BaseNode.destroy_all_action_clients

```{autodoc2-docstring} ros_sugar.core.node.BaseNode.destroy_all_action_clients
```

````

````{py:method} destroy_all_service_clients()
:canonical: ros_sugar.core.node.BaseNode.destroy_all_service_clients

```{autodoc2-docstring} ros_sugar.core.node.BaseNode.destroy_all_service_clients
```

````

````{py:method} destroy_all_timers()
:canonical: ros_sugar.core.node.BaseNode.destroy_all_timers

```{autodoc2-docstring} ros_sugar.core.node.BaseNode.destroy_all_timers
```

````

````{py:attribute} PARAM_REL_TOL
:canonical: ros_sugar.core.node.BaseNode.PARAM_REL_TOL
:value: >
   1e-06

````

````{py:property} publishers
:canonical: ros_sugar.core.node.BaseNode.publishers
:type: typing.Iterator[rclpy.publisher.Publisher]

````

````{py:property} subscriptions
:canonical: ros_sugar.core.node.BaseNode.subscriptions
:type: typing.Iterator[rclpy.subscription.Subscription]

````

````{py:property} clients
:canonical: ros_sugar.core.node.BaseNode.clients
:type: typing.Iterator[rclpy.client.Client]

````

````{py:property} services
:canonical: ros_sugar.core.node.BaseNode.services
:type: typing.Iterator[rclpy.service.Service]

````

````{py:property} timers
:canonical: ros_sugar.core.node.BaseNode.timers
:type: typing.Iterator[rclpy.timer.Timer]

````

````{py:property} guards
:canonical: ros_sugar.core.node.BaseNode.guards
:type: typing.Iterator[rclpy.guard_condition.GuardCondition]

````

````{py:property} waitables
:canonical: ros_sugar.core.node.BaseNode.waitables
:type: typing.Iterator[rclpy.waitable.Waitable]

````

````{py:property} executor
:canonical: ros_sugar.core.node.BaseNode.executor
:type: typing.Optional[rclpy.executors.Executor]

````

````{py:property} context
:canonical: ros_sugar.core.node.BaseNode.context
:type: rclpy.context.Context

````

````{py:property} default_callback_group
:canonical: ros_sugar.core.node.BaseNode.default_callback_group
:type: rclpy.callback_groups.CallbackGroup

````

````{py:property} handle
:canonical: ros_sugar.core.node.BaseNode.handle

````

````{py:method} get_name() -> str
:canonical: ros_sugar.core.node.BaseNode.get_name

````

````{py:method} get_namespace() -> str
:canonical: ros_sugar.core.node.BaseNode.get_namespace

````

````{py:method} get_clock() -> rclpy.clock.Clock
:canonical: ros_sugar.core.node.BaseNode.get_clock

````

````{py:method} get_logger()
:canonical: ros_sugar.core.node.BaseNode.get_logger

````

````{py:method} declare_parameter(name: str, value: typing.Any = None, descriptor: typing.Optional[rcl_interfaces.msg.ParameterDescriptor] = None, ignore_override: bool = False) -> rclpy.parameter.Parameter
:canonical: ros_sugar.core.node.BaseNode.declare_parameter

````

````{py:method} declare_parameters(namespace: str, parameters: typing.List[typing.Union[typing.Tuple[str], typing.Tuple[str, rclpy.parameter.Parameter.Type], typing.Tuple[str, typing.Any, rcl_interfaces.msg.ParameterDescriptor]]], ignore_override: bool = False) -> typing.List[rclpy.parameter.Parameter]
:canonical: ros_sugar.core.node.BaseNode.declare_parameters

````

````{py:method} undeclare_parameter(name: str)
:canonical: ros_sugar.core.node.BaseNode.undeclare_parameter

````

````{py:method} has_parameter(name: str) -> bool
:canonical: ros_sugar.core.node.BaseNode.has_parameter

````

````{py:method} get_parameter_types(names: typing.List[str]) -> typing.List[rclpy.parameter.Parameter.Type]
:canonical: ros_sugar.core.node.BaseNode.get_parameter_types

````

````{py:method} get_parameter_type(name: str) -> rclpy.parameter.Parameter.Type
:canonical: ros_sugar.core.node.BaseNode.get_parameter_type

````

````{py:method} get_parameters(names: typing.List[str]) -> typing.List[rclpy.parameter.Parameter]
:canonical: ros_sugar.core.node.BaseNode.get_parameters

````

````{py:method} get_parameter(name: str) -> rclpy.parameter.Parameter
:canonical: ros_sugar.core.node.BaseNode.get_parameter

````

````{py:method} get_parameter_or(name: str, alternative_value: typing.Optional[rclpy.parameter.Parameter] = None) -> rclpy.parameter.Parameter
:canonical: ros_sugar.core.node.BaseNode.get_parameter_or

````

````{py:method} get_parameters_by_prefix(prefix: str) -> typing.Dict[str, typing.Optional[typing.Union[bool, int, float, str, bytes, typing.Sequence[bool], typing.Sequence[int], typing.Sequence[float], typing.Sequence[str]]]]
:canonical: ros_sugar.core.node.BaseNode.get_parameters_by_prefix

````

````{py:method} set_parameters(parameter_list: typing.List[rclpy.parameter.Parameter]) -> typing.List[rcl_interfaces.msg.SetParametersResult]
:canonical: ros_sugar.core.node.BaseNode.set_parameters

````

````{py:method} set_parameters_atomically(parameter_list: typing.List[rclpy.parameter.Parameter]) -> rcl_interfaces.msg.SetParametersResult
:canonical: ros_sugar.core.node.BaseNode.set_parameters_atomically

````

````{py:method} add_pre_set_parameters_callback(callback: typing.Callable[[typing.List[rclpy.parameter.Parameter]], typing.List[rclpy.parameter.Parameter]]) -> None
:canonical: ros_sugar.core.node.BaseNode.add_pre_set_parameters_callback

````

````{py:method} add_on_set_parameters_callback(callback: typing.Callable[[typing.List[rclpy.parameter.Parameter]], rcl_interfaces.msg.SetParametersResult]) -> None
:canonical: ros_sugar.core.node.BaseNode.add_on_set_parameters_callback

````

````{py:method} add_post_set_parameters_callback(callback: typing.Callable[[typing.List[rclpy.parameter.Parameter]], None]) -> None
:canonical: ros_sugar.core.node.BaseNode.add_post_set_parameters_callback

````

````{py:method} remove_pre_set_parameters_callback(callback: typing.Callable[[typing.List[rclpy.parameter.Parameter]], typing.List[rclpy.parameter.Parameter]]) -> None
:canonical: ros_sugar.core.node.BaseNode.remove_pre_set_parameters_callback

````

````{py:method} remove_on_set_parameters_callback(callback: typing.Callable[[typing.List[rclpy.parameter.Parameter]], rcl_interfaces.msg.SetParametersResult]) -> None
:canonical: ros_sugar.core.node.BaseNode.remove_on_set_parameters_callback

````

````{py:method} remove_post_set_parameters_callback(callback: typing.Callable[[typing.List[rclpy.parameter.Parameter]], None]) -> None
:canonical: ros_sugar.core.node.BaseNode.remove_post_set_parameters_callback

````

````{py:method} describe_parameter(name: str) -> rcl_interfaces.msg.ParameterDescriptor
:canonical: ros_sugar.core.node.BaseNode.describe_parameter

````

````{py:method} describe_parameters(names: typing.List[str]) -> typing.List[rcl_interfaces.msg.ParameterDescriptor]
:canonical: ros_sugar.core.node.BaseNode.describe_parameters

````

````{py:method} set_descriptor(name: str, descriptor: rcl_interfaces.msg.ParameterDescriptor, alternative_value: typing.Optional[rcl_interfaces.msg.ParameterValue] = None) -> rcl_interfaces.msg.ParameterValue
:canonical: ros_sugar.core.node.BaseNode.set_descriptor

````

````{py:method} add_waitable(waitable: rclpy.waitable.Waitable) -> None
:canonical: ros_sugar.core.node.BaseNode.add_waitable

````

````{py:method} remove_waitable(waitable: rclpy.waitable.Waitable) -> None
:canonical: ros_sugar.core.node.BaseNode.remove_waitable

````

````{py:method} resolve_topic_name(topic: str, *, only_expand: bool = False) -> str
:canonical: ros_sugar.core.node.BaseNode.resolve_topic_name

````

````{py:method} resolve_service_name(service: str, *, only_expand: bool = False) -> str
:canonical: ros_sugar.core.node.BaseNode.resolve_service_name

````

````{py:method} create_publisher(msg_type, topic: str, qos_profile: typing.Union[rclpy.qos.QoSProfile, int], *, callback_group: typing.Optional[rclpy.callback_groups.CallbackGroup] = None, event_callbacks: typing.Optional[rclpy.event_handler.PublisherEventCallbacks] = None, qos_overriding_options: typing.Optional[rclpy.qos_overriding_options.QoSOverridingOptions] = None, publisher_class: typing.Type[rclpy.publisher.Publisher] = Publisher) -> rclpy.publisher.Publisher
:canonical: ros_sugar.core.node.BaseNode.create_publisher

````

````{py:method} create_subscription(msg_type, topic: str, callback: typing.Callable[[rclpy.node.MsgType], None], qos_profile: typing.Union[rclpy.qos.QoSProfile, int], *, callback_group: typing.Optional[rclpy.callback_groups.CallbackGroup] = None, event_callbacks: typing.Optional[rclpy.event_handler.SubscriptionEventCallbacks] = None, qos_overriding_options: typing.Optional[rclpy.qos_overriding_options.QoSOverridingOptions] = None, raw: bool = False) -> rclpy.subscription.Subscription
:canonical: ros_sugar.core.node.BaseNode.create_subscription

````

````{py:method} create_service(srv_type, srv_name: str, callback: typing.Callable[[rclpy.node.SrvTypeRequest, rclpy.node.SrvTypeResponse], rclpy.node.SrvTypeResponse], *, qos_profile: rclpy.qos.QoSProfile = qos_profile_services_default, callback_group: rclpy.callback_groups.CallbackGroup = None) -> rclpy.service.Service
:canonical: ros_sugar.core.node.BaseNode.create_service

````

````{py:method} create_timer(timer_period_sec: float, callback: typing.Callable, callback_group: rclpy.callback_groups.CallbackGroup = None, clock: rclpy.clock.Clock = None) -> rclpy.timer.Timer
:canonical: ros_sugar.core.node.BaseNode.create_timer

````

````{py:method} create_guard_condition(callback: typing.Callable, callback_group: rclpy.callback_groups.CallbackGroup = None) -> rclpy.guard_condition.GuardCondition
:canonical: ros_sugar.core.node.BaseNode.create_guard_condition

````

````{py:method} create_rate(frequency: float, clock: rclpy.clock.Clock = None) -> rclpy.timer.Rate
:canonical: ros_sugar.core.node.BaseNode.create_rate

````

````{py:method} destroy_publisher(publisher: rclpy.publisher.Publisher) -> bool
:canonical: ros_sugar.core.node.BaseNode.destroy_publisher

````

````{py:method} destroy_subscription(subscription: rclpy.subscription.Subscription) -> bool
:canonical: ros_sugar.core.node.BaseNode.destroy_subscription

````

````{py:method} destroy_client(client: rclpy.client.Client) -> bool
:canonical: ros_sugar.core.node.BaseNode.destroy_client

````

````{py:method} destroy_service(service: rclpy.service.Service) -> bool
:canonical: ros_sugar.core.node.BaseNode.destroy_service

````

````{py:method} destroy_timer(timer: rclpy.timer.Timer) -> bool
:canonical: ros_sugar.core.node.BaseNode.destroy_timer

````

````{py:method} destroy_guard_condition(guard: rclpy.guard_condition.GuardCondition) -> bool
:canonical: ros_sugar.core.node.BaseNode.destroy_guard_condition

````

````{py:method} destroy_rate(rate: rclpy.timer.Rate) -> bool
:canonical: ros_sugar.core.node.BaseNode.destroy_rate

````

````{py:method} destroy_node()
:canonical: ros_sugar.core.node.BaseNode.destroy_node

````

````{py:method} get_publisher_names_and_types_by_node(node_name: str, node_namespace: str, no_demangle: bool = False) -> typing.List[typing.Tuple[str, typing.List[str]]]
:canonical: ros_sugar.core.node.BaseNode.get_publisher_names_and_types_by_node

````

````{py:method} get_subscriber_names_and_types_by_node(node_name: str, node_namespace: str, no_demangle: bool = False) -> typing.List[typing.Tuple[str, typing.List[str]]]
:canonical: ros_sugar.core.node.BaseNode.get_subscriber_names_and_types_by_node

````

````{py:method} get_service_names_and_types_by_node(node_name: str, node_namespace: str) -> typing.List[typing.Tuple[str, typing.List[str]]]
:canonical: ros_sugar.core.node.BaseNode.get_service_names_and_types_by_node

````

````{py:method} get_client_names_and_types_by_node(node_name: str, node_namespace: str) -> typing.List[typing.Tuple[str, typing.List[str]]]
:canonical: ros_sugar.core.node.BaseNode.get_client_names_and_types_by_node

````

````{py:method} get_topic_names_and_types(no_demangle: bool = False) -> typing.List[typing.Tuple[str, typing.List[str]]]
:canonical: ros_sugar.core.node.BaseNode.get_topic_names_and_types

````

````{py:method} get_service_names_and_types() -> typing.List[typing.Tuple[str, typing.List[str]]]
:canonical: ros_sugar.core.node.BaseNode.get_service_names_and_types

````

````{py:method} get_node_names() -> typing.List[str]
:canonical: ros_sugar.core.node.BaseNode.get_node_names

````

````{py:method} get_fully_qualified_node_names() -> typing.List[str]
:canonical: ros_sugar.core.node.BaseNode.get_fully_qualified_node_names

````

````{py:method} get_node_names_and_namespaces() -> typing.List[typing.Tuple[str, str]]
:canonical: ros_sugar.core.node.BaseNode.get_node_names_and_namespaces

````

````{py:method} get_node_names_and_namespaces_with_enclaves() -> typing.List[typing.Tuple[str, str, str]]
:canonical: ros_sugar.core.node.BaseNode.get_node_names_and_namespaces_with_enclaves

````

````{py:method} get_fully_qualified_name() -> str
:canonical: ros_sugar.core.node.BaseNode.get_fully_qualified_name

````

````{py:method} count_publishers(topic_name: str) -> int
:canonical: ros_sugar.core.node.BaseNode.count_publishers

````

````{py:method} count_subscribers(topic_name: str) -> int
:canonical: ros_sugar.core.node.BaseNode.count_subscribers

````

````{py:method} get_publishers_info_by_topic(topic_name: str, no_mangle: bool = False) -> typing.List[rclpy.topic_endpoint_info.TopicEndpointInfo]
:canonical: ros_sugar.core.node.BaseNode.get_publishers_info_by_topic

````

````{py:method} get_subscriptions_info_by_topic(topic_name: str, no_mangle: bool = False) -> typing.List[rclpy.topic_endpoint_info.TopicEndpointInfo]
:canonical: ros_sugar.core.node.BaseNode.get_subscriptions_info_by_topic

````

````{py:method} wait_for_node(fully_qualified_node_name: str, timeout: float) -> bool
:canonical: ros_sugar.core.node.BaseNode.wait_for_node

````

`````
