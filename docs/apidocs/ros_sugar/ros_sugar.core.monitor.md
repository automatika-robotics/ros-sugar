# {py:mod}`ros_sugar.core.monitor`

```{py:module} ros_sugar.core.monitor
```

```{autodoc2-docstring} ros_sugar.core.monitor
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`Monitor <ros_sugar.core.monitor.Monitor>`
  - ```{autodoc2-docstring} ros_sugar.core.monitor.Monitor
    :summary:
    ```
````

### API

`````{py:class} Monitor(components_names: typing.List[str], enable_health_status_monitoring: bool = True, events_actions: typing.Optional[typing.Dict[ros_sugar.core.event.Event, typing.List[ros_sugar.core.action.Action]]] = None, events_to_emit: typing.Optional[typing.List[ros_sugar.core.event.Event]] = None, config: typing.Optional[ros_sugar.config.BaseConfig] = None, services_components: typing.Optional[typing.List[ros_sugar.core.component.BaseComponent]] = None, action_servers_components: typing.Optional[typing.List[ros_sugar.core.component.BaseComponent]] = None, activate_on_start: typing.Optional[typing.List[ros_sugar.core.component.BaseComponent]] = None, start_on_init: bool = False, component_name: str = 'monitor', callback_group: typing.Optional[typing.Union[rclpy.callback_groups.MutuallyExclusiveCallbackGroup, rclpy.callback_groups.ReentrantCallbackGroup]] = None, *args, **kwargs)
:canonical: ros_sugar.core.monitor.Monitor

Bases: {py:obj}`ros_sugar.core.node.BaseNode`

```{autodoc2-docstring} ros_sugar.core.monitor.Monitor
```

````{py:method} add_components_activation_event(method) -> None
:canonical: ros_sugar.core.monitor.Monitor.add_components_activation_event

```{autodoc2-docstring} ros_sugar.core.monitor.Monitor.add_components_activation_event
```

````

````{py:method} create_all_timers() -> None
:canonical: ros_sugar.core.monitor.Monitor.create_all_timers

```{autodoc2-docstring} ros_sugar.core.monitor.Monitor.create_all_timers
```

````

````{py:property} events
:canonical: ros_sugar.core.monitor.Monitor.events

```{autodoc2-docstring} ros_sugar.core.monitor.Monitor.events
```

````

````{py:method} configure_component(component: ros_sugar.core.component.BaseComponent, new_config: typing.Union[object, str], keep_alive: bool) -> None
:canonical: ros_sugar.core.monitor.Monitor.configure_component

```{autodoc2-docstring} ros_sugar.core.monitor.Monitor.configure_component
```

````

````{py:method} update_parameter(component: ros_sugar.core.component.BaseComponent, param_name: str, new_value: typing.Any, keep_alive: bool = True) -> None
:canonical: ros_sugar.core.monitor.Monitor.update_parameter

```{autodoc2-docstring} ros_sugar.core.monitor.Monitor.update_parameter
```

````

````{py:method} update_parameters(component: ros_sugar.core.component.BaseComponent, params_names: typing.List[str], new_values: typing.List, keep_alive: bool = True, **_) -> None
:canonical: ros_sugar.core.monitor.Monitor.update_parameters

```{autodoc2-docstring} ros_sugar.core.monitor.Monitor.update_parameters
```

````

````{py:method} send_srv_request(srv_name: str, srv_type: type, srv_request_msg: typing.Any, **_) -> None
:canonical: ros_sugar.core.monitor.Monitor.send_srv_request

```{autodoc2-docstring} ros_sugar.core.monitor.Monitor.send_srv_request
```

````

````{py:method} send_action_goal(action_name: str, action_type: type, action_request_msg: typing.Any, **_) -> None
:canonical: ros_sugar.core.monitor.Monitor.send_action_goal

```{autodoc2-docstring} ros_sugar.core.monitor.Monitor.send_action_goal
```

````

````{py:method} publish_message(topic: ros_sugar.io.topic.Topic, msg: typing.Any, publish_rate: typing.Optional[float] = None, publish_period: typing.Optional[float] = None, **_) -> None
:canonical: ros_sugar.core.monitor.Monitor.publish_message

```{autodoc2-docstring} ros_sugar.core.monitor.Monitor.publish_message
```

````

````{py:method} create_all_subscribers() -> None
:canonical: ros_sugar.core.monitor.Monitor.create_all_subscribers

```{autodoc2-docstring} ros_sugar.core.monitor.Monitor.create_all_subscribers
```

````

````{py:method} create_all_service_clients() -> None
:canonical: ros_sugar.core.monitor.Monitor.create_all_service_clients

```{autodoc2-docstring} ros_sugar.core.monitor.Monitor.create_all_service_clients
```

````

````{py:method} create_all_action_clients() -> None
:canonical: ros_sugar.core.monitor.Monitor.create_all_action_clients

```{autodoc2-docstring} ros_sugar.core.monitor.Monitor.create_all_action_clients
```

````

````{py:method} rclpy_init_node(*args, **kwargs)
:canonical: ros_sugar.core.monitor.Monitor.rclpy_init_node

````

````{py:method} configure(config_file: str)
:canonical: ros_sugar.core.monitor.Monitor.configure

````

````{py:method} add_execute_once(method: typing.Callable)
:canonical: ros_sugar.core.monitor.Monitor.add_execute_once

````

````{py:method} add_execute_in_loop(method: typing.Callable)
:canonical: ros_sugar.core.monitor.Monitor.add_execute_in_loop

````

````{py:method} get_ros_time() -> builtin_interfaces.msg.Time
:canonical: ros_sugar.core.monitor.Monitor.get_ros_time

````

````{py:method} get_secs_time() -> float
:canonical: ros_sugar.core.monitor.Monitor.get_secs_time

````

````{py:property} launch_cmd_args
:canonical: ros_sugar.core.monitor.Monitor.launch_cmd_args
:type: typing.List[str]

````

````{py:property} config_json
:canonical: ros_sugar.core.monitor.Monitor.config_json
:type: typing.Union[str, bytes]

````

````{py:method} activate()
:canonical: ros_sugar.core.monitor.Monitor.activate

````

````{py:method} deactivate()
:canonical: ros_sugar.core.monitor.Monitor.deactivate

````

````{py:method} setup_qos(qos_policy: ros_sugar.config.QoSConfig) -> rclpy.qos.QoSProfile
:canonical: ros_sugar.core.monitor.Monitor.setup_qos

````

````{py:method} init_flags()
:canonical: ros_sugar.core.monitor.Monitor.init_flags

````

````{py:method} init_variables()
:canonical: ros_sugar.core.monitor.Monitor.init_variables

````

````{py:method} create_tf_listener(tf_config: ros_sugar.tf.TFListenerConfig) -> ros_sugar.tf.TFListener
:canonical: ros_sugar.core.monitor.Monitor.create_tf_listener

````

````{py:method} create_client(*args, **kwargs) -> rclpy.client.Client
:canonical: ros_sugar.core.monitor.Monitor.create_client

````

````{py:method} create_all_publishers()
:canonical: ros_sugar.core.monitor.Monitor.create_all_publishers

````

````{py:method} create_all_services()
:canonical: ros_sugar.core.monitor.Monitor.create_all_services

````

````{py:method} create_all_action_servers()
:canonical: ros_sugar.core.monitor.Monitor.create_all_action_servers

````

````{py:method} destroy_all_subscribers()
:canonical: ros_sugar.core.monitor.Monitor.destroy_all_subscribers

````

````{py:method} destroy_all_publishers()
:canonical: ros_sugar.core.monitor.Monitor.destroy_all_publishers

````

````{py:method} destroy_all_services()
:canonical: ros_sugar.core.monitor.Monitor.destroy_all_services

````

````{py:method} destroy_all_action_servers()
:canonical: ros_sugar.core.monitor.Monitor.destroy_all_action_servers

````

````{py:method} destroy_all_action_clients()
:canonical: ros_sugar.core.monitor.Monitor.destroy_all_action_clients

````

````{py:method} destroy_all_service_clients()
:canonical: ros_sugar.core.monitor.Monitor.destroy_all_service_clients

````

````{py:method} destroy_all_timers()
:canonical: ros_sugar.core.monitor.Monitor.destroy_all_timers

````

````{py:attribute} PARAM_REL_TOL
:canonical: ros_sugar.core.monitor.Monitor.PARAM_REL_TOL
:value: >
   1e-06

````

````{py:property} publishers
:canonical: ros_sugar.core.monitor.Monitor.publishers
:type: typing.Iterator[rclpy.publisher.Publisher]

````

````{py:property} subscriptions
:canonical: ros_sugar.core.monitor.Monitor.subscriptions
:type: typing.Iterator[rclpy.subscription.Subscription]

````

````{py:property} clients
:canonical: ros_sugar.core.monitor.Monitor.clients
:type: typing.Iterator[rclpy.client.Client]

````

````{py:property} services
:canonical: ros_sugar.core.monitor.Monitor.services
:type: typing.Iterator[rclpy.service.Service]

````

````{py:property} timers
:canonical: ros_sugar.core.monitor.Monitor.timers
:type: typing.Iterator[rclpy.timer.Timer]

````

````{py:property} guards
:canonical: ros_sugar.core.monitor.Monitor.guards
:type: typing.Iterator[rclpy.guard_condition.GuardCondition]

````

````{py:property} waitables
:canonical: ros_sugar.core.monitor.Monitor.waitables
:type: typing.Iterator[rclpy.waitable.Waitable]

````

````{py:property} executor
:canonical: ros_sugar.core.monitor.Monitor.executor
:type: typing.Optional[rclpy.executors.Executor]

````

````{py:property} context
:canonical: ros_sugar.core.monitor.Monitor.context
:type: rclpy.context.Context

````

````{py:property} default_callback_group
:canonical: ros_sugar.core.monitor.Monitor.default_callback_group
:type: rclpy.callback_groups.CallbackGroup

````

````{py:property} handle
:canonical: ros_sugar.core.monitor.Monitor.handle

````

````{py:method} get_name() -> str
:canonical: ros_sugar.core.monitor.Monitor.get_name

````

````{py:method} get_namespace() -> str
:canonical: ros_sugar.core.monitor.Monitor.get_namespace

````

````{py:method} get_clock() -> rclpy.clock.Clock
:canonical: ros_sugar.core.monitor.Monitor.get_clock

````

````{py:method} get_logger()
:canonical: ros_sugar.core.monitor.Monitor.get_logger

````

````{py:method} declare_parameter(name: str, value: typing.Any = None, descriptor: typing.Optional[rcl_interfaces.msg.ParameterDescriptor] = None, ignore_override: bool = False) -> rclpy.parameter.Parameter
:canonical: ros_sugar.core.monitor.Monitor.declare_parameter

````

````{py:method} declare_parameters(namespace: str, parameters: typing.List[typing.Union[typing.Tuple[str], typing.Tuple[str, rclpy.parameter.Parameter.Type], typing.Tuple[str, typing.Any, rcl_interfaces.msg.ParameterDescriptor]]], ignore_override: bool = False) -> typing.List[rclpy.parameter.Parameter]
:canonical: ros_sugar.core.monitor.Monitor.declare_parameters

````

````{py:method} undeclare_parameter(name: str)
:canonical: ros_sugar.core.monitor.Monitor.undeclare_parameter

````

````{py:method} has_parameter(name: str) -> bool
:canonical: ros_sugar.core.monitor.Monitor.has_parameter

````

````{py:method} get_parameter_types(names: typing.List[str]) -> typing.List[rclpy.parameter.Parameter.Type]
:canonical: ros_sugar.core.monitor.Monitor.get_parameter_types

````

````{py:method} get_parameter_type(name: str) -> rclpy.parameter.Parameter.Type
:canonical: ros_sugar.core.monitor.Monitor.get_parameter_type

````

````{py:method} get_parameters(names: typing.List[str]) -> typing.List[rclpy.parameter.Parameter]
:canonical: ros_sugar.core.monitor.Monitor.get_parameters

````

````{py:method} get_parameter(name: str) -> rclpy.parameter.Parameter
:canonical: ros_sugar.core.monitor.Monitor.get_parameter

````

````{py:method} get_parameter_or(name: str, alternative_value: typing.Optional[rclpy.parameter.Parameter] = None) -> rclpy.parameter.Parameter
:canonical: ros_sugar.core.monitor.Monitor.get_parameter_or

````

````{py:method} get_parameters_by_prefix(prefix: str) -> typing.Dict[str, typing.Optional[typing.Union[bool, int, float, str, bytes, typing.Sequence[bool], typing.Sequence[int], typing.Sequence[float], typing.Sequence[str]]]]
:canonical: ros_sugar.core.monitor.Monitor.get_parameters_by_prefix

````

````{py:method} set_parameters(parameter_list: typing.List[rclpy.parameter.Parameter]) -> typing.List[rcl_interfaces.msg.SetParametersResult]
:canonical: ros_sugar.core.monitor.Monitor.set_parameters

````

````{py:method} set_parameters_atomically(parameter_list: typing.List[rclpy.parameter.Parameter]) -> rcl_interfaces.msg.SetParametersResult
:canonical: ros_sugar.core.monitor.Monitor.set_parameters_atomically

````

````{py:method} add_pre_set_parameters_callback(callback: typing.Callable[[typing.List[rclpy.parameter.Parameter]], typing.List[rclpy.parameter.Parameter]]) -> None
:canonical: ros_sugar.core.monitor.Monitor.add_pre_set_parameters_callback

````

````{py:method} add_on_set_parameters_callback(callback: typing.Callable[[typing.List[rclpy.parameter.Parameter]], rcl_interfaces.msg.SetParametersResult]) -> None
:canonical: ros_sugar.core.monitor.Monitor.add_on_set_parameters_callback

````

````{py:method} add_post_set_parameters_callback(callback: typing.Callable[[typing.List[rclpy.parameter.Parameter]], None]) -> None
:canonical: ros_sugar.core.monitor.Monitor.add_post_set_parameters_callback

````

````{py:method} remove_pre_set_parameters_callback(callback: typing.Callable[[typing.List[rclpy.parameter.Parameter]], typing.List[rclpy.parameter.Parameter]]) -> None
:canonical: ros_sugar.core.monitor.Monitor.remove_pre_set_parameters_callback

````

````{py:method} remove_on_set_parameters_callback(callback: typing.Callable[[typing.List[rclpy.parameter.Parameter]], rcl_interfaces.msg.SetParametersResult]) -> None
:canonical: ros_sugar.core.monitor.Monitor.remove_on_set_parameters_callback

````

````{py:method} remove_post_set_parameters_callback(callback: typing.Callable[[typing.List[rclpy.parameter.Parameter]], None]) -> None
:canonical: ros_sugar.core.monitor.Monitor.remove_post_set_parameters_callback

````

````{py:method} describe_parameter(name: str) -> rcl_interfaces.msg.ParameterDescriptor
:canonical: ros_sugar.core.monitor.Monitor.describe_parameter

````

````{py:method} describe_parameters(names: typing.List[str]) -> typing.List[rcl_interfaces.msg.ParameterDescriptor]
:canonical: ros_sugar.core.monitor.Monitor.describe_parameters

````

````{py:method} set_descriptor(name: str, descriptor: rcl_interfaces.msg.ParameterDescriptor, alternative_value: typing.Optional[rcl_interfaces.msg.ParameterValue] = None) -> rcl_interfaces.msg.ParameterValue
:canonical: ros_sugar.core.monitor.Monitor.set_descriptor

````

````{py:method} add_waitable(waitable: rclpy.waitable.Waitable) -> None
:canonical: ros_sugar.core.monitor.Monitor.add_waitable

````

````{py:method} remove_waitable(waitable: rclpy.waitable.Waitable) -> None
:canonical: ros_sugar.core.monitor.Monitor.remove_waitable

````

````{py:method} resolve_topic_name(topic: str, *, only_expand: bool = False) -> str
:canonical: ros_sugar.core.monitor.Monitor.resolve_topic_name

````

````{py:method} resolve_service_name(service: str, *, only_expand: bool = False) -> str
:canonical: ros_sugar.core.monitor.Monitor.resolve_service_name

````

````{py:method} create_publisher(msg_type, topic: str, qos_profile: typing.Union[rclpy.qos.QoSProfile, int], *, callback_group: typing.Optional[rclpy.callback_groups.CallbackGroup] = None, event_callbacks: typing.Optional[rclpy.event_handler.PublisherEventCallbacks] = None, qos_overriding_options: typing.Optional[rclpy.qos_overriding_options.QoSOverridingOptions] = None, publisher_class: typing.Type[rclpy.publisher.Publisher] = Publisher) -> rclpy.publisher.Publisher
:canonical: ros_sugar.core.monitor.Monitor.create_publisher

````

````{py:method} create_subscription(msg_type, topic: str, callback: typing.Callable[[rclpy.node.MsgType], None], qos_profile: typing.Union[rclpy.qos.QoSProfile, int], *, callback_group: typing.Optional[rclpy.callback_groups.CallbackGroup] = None, event_callbacks: typing.Optional[rclpy.event_handler.SubscriptionEventCallbacks] = None, qos_overriding_options: typing.Optional[rclpy.qos_overriding_options.QoSOverridingOptions] = None, raw: bool = False) -> rclpy.subscription.Subscription
:canonical: ros_sugar.core.monitor.Monitor.create_subscription

````

````{py:method} create_service(srv_type, srv_name: str, callback: typing.Callable[[rclpy.node.SrvTypeRequest, rclpy.node.SrvTypeResponse], rclpy.node.SrvTypeResponse], *, qos_profile: rclpy.qos.QoSProfile = qos_profile_services_default, callback_group: rclpy.callback_groups.CallbackGroup = None) -> rclpy.service.Service
:canonical: ros_sugar.core.monitor.Monitor.create_service

````

````{py:method} create_timer(timer_period_sec: float, callback: typing.Callable, callback_group: rclpy.callback_groups.CallbackGroup = None, clock: rclpy.clock.Clock = None) -> rclpy.timer.Timer
:canonical: ros_sugar.core.monitor.Monitor.create_timer

````

````{py:method} create_guard_condition(callback: typing.Callable, callback_group: rclpy.callback_groups.CallbackGroup = None) -> rclpy.guard_condition.GuardCondition
:canonical: ros_sugar.core.monitor.Monitor.create_guard_condition

````

````{py:method} create_rate(frequency: float, clock: rclpy.clock.Clock = None) -> rclpy.timer.Rate
:canonical: ros_sugar.core.monitor.Monitor.create_rate

````

````{py:method} destroy_publisher(publisher: rclpy.publisher.Publisher) -> bool
:canonical: ros_sugar.core.monitor.Monitor.destroy_publisher

````

````{py:method} destroy_subscription(subscription: rclpy.subscription.Subscription) -> bool
:canonical: ros_sugar.core.monitor.Monitor.destroy_subscription

````

````{py:method} destroy_client(client: rclpy.client.Client) -> bool
:canonical: ros_sugar.core.monitor.Monitor.destroy_client

````

````{py:method} destroy_service(service: rclpy.service.Service) -> bool
:canonical: ros_sugar.core.monitor.Monitor.destroy_service

````

````{py:method} destroy_timer(timer: rclpy.timer.Timer) -> bool
:canonical: ros_sugar.core.monitor.Monitor.destroy_timer

````

````{py:method} destroy_guard_condition(guard: rclpy.guard_condition.GuardCondition) -> bool
:canonical: ros_sugar.core.monitor.Monitor.destroy_guard_condition

````

````{py:method} destroy_rate(rate: rclpy.timer.Rate) -> bool
:canonical: ros_sugar.core.monitor.Monitor.destroy_rate

````

````{py:method} destroy_node()
:canonical: ros_sugar.core.monitor.Monitor.destroy_node

````

````{py:method} get_publisher_names_and_types_by_node(node_name: str, node_namespace: str, no_demangle: bool = False) -> typing.List[typing.Tuple[str, typing.List[str]]]
:canonical: ros_sugar.core.monitor.Monitor.get_publisher_names_and_types_by_node

````

````{py:method} get_subscriber_names_and_types_by_node(node_name: str, node_namespace: str, no_demangle: bool = False) -> typing.List[typing.Tuple[str, typing.List[str]]]
:canonical: ros_sugar.core.monitor.Monitor.get_subscriber_names_and_types_by_node

````

````{py:method} get_service_names_and_types_by_node(node_name: str, node_namespace: str) -> typing.List[typing.Tuple[str, typing.List[str]]]
:canonical: ros_sugar.core.monitor.Monitor.get_service_names_and_types_by_node

````

````{py:method} get_client_names_and_types_by_node(node_name: str, node_namespace: str) -> typing.List[typing.Tuple[str, typing.List[str]]]
:canonical: ros_sugar.core.monitor.Monitor.get_client_names_and_types_by_node

````

````{py:method} get_topic_names_and_types(no_demangle: bool = False) -> typing.List[typing.Tuple[str, typing.List[str]]]
:canonical: ros_sugar.core.monitor.Monitor.get_topic_names_and_types

````

````{py:method} get_service_names_and_types() -> typing.List[typing.Tuple[str, typing.List[str]]]
:canonical: ros_sugar.core.monitor.Monitor.get_service_names_and_types

````

````{py:method} get_node_names() -> typing.List[str]
:canonical: ros_sugar.core.monitor.Monitor.get_node_names

````

````{py:method} get_fully_qualified_node_names() -> typing.List[str]
:canonical: ros_sugar.core.monitor.Monitor.get_fully_qualified_node_names

````

````{py:method} get_node_names_and_namespaces() -> typing.List[typing.Tuple[str, str]]
:canonical: ros_sugar.core.monitor.Monitor.get_node_names_and_namespaces

````

````{py:method} get_node_names_and_namespaces_with_enclaves() -> typing.List[typing.Tuple[str, str, str]]
:canonical: ros_sugar.core.monitor.Monitor.get_node_names_and_namespaces_with_enclaves

````

````{py:method} get_fully_qualified_name() -> str
:canonical: ros_sugar.core.monitor.Monitor.get_fully_qualified_name

````

````{py:method} count_publishers(topic_name: str) -> int
:canonical: ros_sugar.core.monitor.Monitor.count_publishers

````

````{py:method} count_subscribers(topic_name: str) -> int
:canonical: ros_sugar.core.monitor.Monitor.count_subscribers

````

````{py:method} get_publishers_info_by_topic(topic_name: str, no_mangle: bool = False) -> typing.List[rclpy.topic_endpoint_info.TopicEndpointInfo]
:canonical: ros_sugar.core.monitor.Monitor.get_publishers_info_by_topic

````

````{py:method} get_subscriptions_info_by_topic(topic_name: str, no_mangle: bool = False) -> typing.List[rclpy.topic_endpoint_info.TopicEndpointInfo]
:canonical: ros_sugar.core.monitor.Monitor.get_subscriptions_info_by_topic

````

````{py:method} wait_for_node(fully_qualified_node_name: str, timeout: float) -> bool
:canonical: ros_sugar.core.monitor.Monitor.wait_for_node

````

`````
