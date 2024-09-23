# {py:mod}`ros_sugar.core.component`

```{py:module} ros_sugar.core.component
```

```{autodoc2-docstring} ros_sugar.core.component
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`BaseComponent <ros_sugar.core.component.BaseComponent>`
  -
````

### API

`````{py:class} BaseComponent(component_name: str, inputs: typing.Optional[typing.Sequence[ros_sugar.io.topic.Topic]] = None, outputs: typing.Optional[typing.Sequence[ros_sugar.io.topic.Topic]] = None, config: typing.Optional[ros_sugar.config.base_config.BaseComponentConfig] = None, config_file: typing.Optional[str] = None, callback_group=None, enable_health_broadcast: bool = True, fallbacks: typing.Optional[ros_sugar.core.fallbacks.ComponentFallbacks] = None, main_action_type: typing.Optional[type] = None, main_srv_type: typing.Optional[type] = None, **kwargs)
:canonical: ros_sugar.core.component.BaseComponent

Bases: {py:obj}`ros_sugar.core.node.BaseNode`, {py:obj}`rclpy.lifecycle.Node`

````{py:method} rclpy_init_node(*args, **kwargs)
:canonical: ros_sugar.core.component.BaseComponent.rclpy_init_node

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.rclpy_init_node
```

````

````{py:method} attach_custom_callback(input_topic: ros_sugar.io.topic.Topic, callable: typing.Callable) -> None
:canonical: ros_sugar.core.component.BaseComponent.attach_custom_callback

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.attach_custom_callback
```

````

````{py:method} add_callback_postprocessor(input_topic: ros_sugar.io.topic.Topic, func: typing.Callable) -> None
:canonical: ros_sugar.core.component.BaseComponent.add_callback_postprocessor

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.add_callback_postprocessor
```

````

````{py:method} add_publisher_preprocessor(output_topic: ros_sugar.io.topic.Topic, func: typing.Callable) -> None
:canonical: ros_sugar.core.component.BaseComponent.add_publisher_preprocessor

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.add_publisher_preprocessor
```

````

````{py:method} create_all_subscribers()
:canonical: ros_sugar.core.component.BaseComponent.create_all_subscribers

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.create_all_subscribers
```

````

````{py:method} create_all_publishers()
:canonical: ros_sugar.core.component.BaseComponent.create_all_publishers

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.create_all_publishers
```

````

````{py:method} create_all_timers()
:canonical: ros_sugar.core.component.BaseComponent.create_all_timers

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.create_all_timers
```

````

````{py:method} create_all_action_servers()
:canonical: ros_sugar.core.component.BaseComponent.create_all_action_servers

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.create_all_action_servers
```

````

````{py:method} create_all_services()
:canonical: ros_sugar.core.component.BaseComponent.create_all_services

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.create_all_services
```

````

````{py:method} destroy_all_timers()
:canonical: ros_sugar.core.component.BaseComponent.destroy_all_timers

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.destroy_all_timers
```

````

````{py:method} destroy_all_subscribers()
:canonical: ros_sugar.core.component.BaseComponent.destroy_all_subscribers

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.destroy_all_subscribers
```

````

````{py:method} destroy_all_publishers()
:canonical: ros_sugar.core.component.BaseComponent.destroy_all_publishers

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.destroy_all_publishers
```

````

````{py:method} destroy_all_services()
:canonical: ros_sugar.core.component.BaseComponent.destroy_all_services

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.destroy_all_services
```

````

````{py:method} destroy_all_action_servers()
:canonical: ros_sugar.core.component.BaseComponent.destroy_all_action_servers

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.destroy_all_action_servers
```

````

````{py:method} got_all_inputs(inputs_to_check: typing.Optional[typing.List[str]] = None, inputs_to_exclude: typing.Optional[typing.List[str]] = None) -> bool
:canonical: ros_sugar.core.component.BaseComponent.got_all_inputs

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.got_all_inputs
```

````

````{py:method} get_missing_inputs() -> list[str]
:canonical: ros_sugar.core.component.BaseComponent.get_missing_inputs

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.get_missing_inputs
```

````

````{py:method} configure(config_file: str)
:canonical: ros_sugar.core.component.BaseComponent.configure

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.configure
```

````

````{py:property} run_type
:canonical: ros_sugar.core.component.BaseComponent.run_type
:type: ros_sugar.config.base_config.ComponentRunType

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.run_type
```

````

````{py:property} fallback_rate
:canonical: ros_sugar.core.component.BaseComponent.fallback_rate
:type: float

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.fallback_rate
```

````

````{py:property} loop_rate
:canonical: ros_sugar.core.component.BaseComponent.loop_rate
:type: float

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.loop_rate
```

````

````{py:property} events_actions
:canonical: ros_sugar.core.component.BaseComponent.events_actions
:type: typing.Dict[str, typing.List[ros_sugar.core.action.Action]]

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.events_actions
```

````

````{py:method} update_cmd_args_list()
:canonical: ros_sugar.core.component.BaseComponent.update_cmd_args_list

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.update_cmd_args_list
```

````

````{py:method} main_action_callback(goal_handle)
:canonical: ros_sugar.core.component.BaseComponent.main_action_callback
:abstractmethod:

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.main_action_callback
```

````

````{py:property} main_action_name
:canonical: ros_sugar.core.component.BaseComponent.main_action_name
:type: typing.Optional[str]

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.main_action_name
```

````

````{py:property} main_srv_name
:canonical: ros_sugar.core.component.BaseComponent.main_srv_name
:type: typing.Optional[str]

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.main_srv_name
```

````

````{py:method} main_service_callback(request, response)
:canonical: ros_sugar.core.component.BaseComponent.main_service_callback
:abstractmethod:

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.main_service_callback
```

````

````{py:method} get_change_parameters_msg_from_config(config: ros_sugar.config.base_config.BaseComponentConfig) -> ros_sugar_interfaces.srv.ChangeParameters.Request
:canonical: ros_sugar.core.component.BaseComponent.get_change_parameters_msg_from_config
:classmethod:

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.get_change_parameters_msg_from_config
```

````

````{py:method} is_topic_of_type(input, msg_type: type) -> bool
:canonical: ros_sugar.core.component.BaseComponent.is_topic_of_type

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.is_topic_of_type
```

````

````{py:method} attach_callbacks()
:canonical: ros_sugar.core.component.BaseComponent.attach_callbacks

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.attach_callbacks
```

````

````{py:property} available_actions
:canonical: ros_sugar.core.component.BaseComponent.available_actions
:type: typing.List[str]

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.available_actions
```

````

````{py:method} start() -> bool
:canonical: ros_sugar.core.component.BaseComponent.start

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.start
```

````

````{py:method} stop() -> bool
:canonical: ros_sugar.core.component.BaseComponent.stop

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.stop
```

````

````{py:method} reconfigure(new_config: typing.Any, keep_alive: bool = False) -> bool
:canonical: ros_sugar.core.component.BaseComponent.reconfigure

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.reconfigure
```

````

````{py:method} restart(wait_time: typing.Optional[float] = None) -> bool
:canonical: ros_sugar.core.component.BaseComponent.restart

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.restart
```

````

````{py:method} set_param(param_name: str, new_value: typing.Any, keep_alive: bool = True) -> bool
:canonical: ros_sugar.core.component.BaseComponent.set_param

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.set_param
```

````

````{py:method} set_params(params_names: typing.List[str], new_values: typing.List, keep_alive: bool = True) -> bool
:canonical: ros_sugar.core.component.BaseComponent.set_params

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.set_params
```

````

````{py:property} fallbacks
:canonical: ros_sugar.core.component.BaseComponent.fallbacks
:type: typing.List[str]

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.fallbacks
```

````

````{py:method} on_fail(action: typing.Union[typing.List[ros_sugar.core.action.Action], ros_sugar.core.action.Action], max_retries: typing.Optional[int] = None) -> None
:canonical: ros_sugar.core.component.BaseComponent.on_fail

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.on_fail
```

````

````{py:method} on_system_fail(action: typing.Union[typing.List[ros_sugar.core.action.Action], ros_sugar.core.action.Action], max_retries: typing.Optional[int] = None) -> None
:canonical: ros_sugar.core.component.BaseComponent.on_system_fail

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.on_system_fail
```

````

````{py:method} on_component_fail(action: typing.Union[typing.List[ros_sugar.core.action.Action], ros_sugar.core.action.Action], max_retries: typing.Optional[int] = None) -> None
:canonical: ros_sugar.core.component.BaseComponent.on_component_fail

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.on_component_fail
```

````

````{py:method} on_algorithm_fail(action: typing.Union[typing.List[ros_sugar.core.action.Action], ros_sugar.core.action.Action], max_retries: typing.Optional[int] = None) -> None
:canonical: ros_sugar.core.component.BaseComponent.on_algorithm_fail

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.on_algorithm_fail
```

````

````{py:method} broadcast_status() -> None
:canonical: ros_sugar.core.component.BaseComponent.broadcast_status

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.broadcast_status
```

````

````{py:property} lifecycle_state
:canonical: ros_sugar.core.component.BaseComponent.lifecycle_state
:type: typing.Optional[int]

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.lifecycle_state
```

````

````{py:method} on_configure(state: rclpy.lifecycle.State) -> rclpy.lifecycle.TransitionCallbackReturn
:canonical: ros_sugar.core.component.BaseComponent.on_configure

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.on_configure
```

````

````{py:method} on_activate(state: rclpy.lifecycle.State) -> rclpy.lifecycle.TransitionCallbackReturn
:canonical: ros_sugar.core.component.BaseComponent.on_activate

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.on_activate
```

````

````{py:method} on_deactivate(state: rclpy.lifecycle.State) -> rclpy.lifecycle.TransitionCallbackReturn
:canonical: ros_sugar.core.component.BaseComponent.on_deactivate

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.on_deactivate
```

````

````{py:method} on_shutdown(state: rclpy.lifecycle.State) -> rclpy.lifecycle.TransitionCallbackReturn
:canonical: ros_sugar.core.component.BaseComponent.on_shutdown

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.on_shutdown
```

````

````{py:method} on_cleanup(state: rclpy.lifecycle.State) -> rclpy.lifecycle.TransitionCallbackReturn
:canonical: ros_sugar.core.component.BaseComponent.on_cleanup

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.on_cleanup
```

````

````{py:method} on_error(state: rclpy.lifecycle.LifecycleState) -> rclpy.lifecycle.TransitionCallbackReturn
:canonical: ros_sugar.core.component.BaseComponent.on_error

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.on_error
```

````

````{py:method} custom_on_configure() -> None
:canonical: ros_sugar.core.component.BaseComponent.custom_on_configure

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.custom_on_configure
```

````

````{py:method} custom_on_activate() -> None
:canonical: ros_sugar.core.component.BaseComponent.custom_on_activate

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.custom_on_activate
```

````

````{py:method} custom_on_deactivate() -> None
:canonical: ros_sugar.core.component.BaseComponent.custom_on_deactivate

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.custom_on_deactivate
```

````

````{py:method} custom_on_shutdown() -> None
:canonical: ros_sugar.core.component.BaseComponent.custom_on_shutdown

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.custom_on_shutdown
```

````

````{py:method} custom_on_error() -> None
:canonical: ros_sugar.core.component.BaseComponent.custom_on_error

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.custom_on_error
```

````

````{py:method} add_execute_once(method: typing.Callable)
:canonical: ros_sugar.core.component.BaseComponent.add_execute_once

````

````{py:method} add_execute_in_loop(method: typing.Callable)
:canonical: ros_sugar.core.component.BaseComponent.add_execute_in_loop

````

````{py:method} get_ros_time() -> builtin_interfaces.msg.Time
:canonical: ros_sugar.core.component.BaseComponent.get_ros_time

````

````{py:method} get_secs_time() -> float
:canonical: ros_sugar.core.component.BaseComponent.get_secs_time

````

````{py:property} launch_cmd_args
:canonical: ros_sugar.core.component.BaseComponent.launch_cmd_args
:type: typing.List[str]

````

````{py:property} config_json
:canonical: ros_sugar.core.component.BaseComponent.config_json
:type: typing.Union[str, bytes]

````

````{py:method} activate()
:canonical: ros_sugar.core.component.BaseComponent.activate

````

````{py:method} deactivate()
:canonical: ros_sugar.core.component.BaseComponent.deactivate

````

````{py:method} setup_qos(qos_policy: ros_sugar.config.QoSConfig) -> rclpy.qos.QoSProfile
:canonical: ros_sugar.core.component.BaseComponent.setup_qos

````

````{py:method} init_flags()
:canonical: ros_sugar.core.component.BaseComponent.init_flags

````

````{py:method} init_variables()
:canonical: ros_sugar.core.component.BaseComponent.init_variables

````

````{py:method} create_tf_listener(tf_config: ros_sugar.tf.TFListenerConfig) -> ros_sugar.tf.TFListener
:canonical: ros_sugar.core.component.BaseComponent.create_tf_listener

````

````{py:method} create_client(*args, **kwargs) -> rclpy.client.Client
:canonical: ros_sugar.core.component.BaseComponent.create_client

````

````{py:method} create_all_service_clients()
:canonical: ros_sugar.core.component.BaseComponent.create_all_service_clients

````

````{py:method} create_all_action_clients()
:canonical: ros_sugar.core.component.BaseComponent.create_all_action_clients

````

````{py:method} destroy_all_action_clients()
:canonical: ros_sugar.core.component.BaseComponent.destroy_all_action_clients

````

````{py:method} destroy_all_service_clients()
:canonical: ros_sugar.core.component.BaseComponent.destroy_all_service_clients

````

````{py:attribute} PARAM_REL_TOL
:canonical: ros_sugar.core.component.BaseComponent.PARAM_REL_TOL
:value: >
   1e-06

````

````{py:property} publishers
:canonical: ros_sugar.core.component.BaseComponent.publishers
:type: typing.Iterator[rclpy.publisher.Publisher]

````

````{py:property} subscriptions
:canonical: ros_sugar.core.component.BaseComponent.subscriptions
:type: typing.Iterator[rclpy.subscription.Subscription]

````

````{py:property} clients
:canonical: ros_sugar.core.component.BaseComponent.clients
:type: typing.Iterator[rclpy.client.Client]

````

````{py:property} services
:canonical: ros_sugar.core.component.BaseComponent.services
:type: typing.Iterator[rclpy.service.Service]

````

````{py:property} timers
:canonical: ros_sugar.core.component.BaseComponent.timers
:type: typing.Iterator[rclpy.timer.Timer]

````

````{py:property} guards
:canonical: ros_sugar.core.component.BaseComponent.guards
:type: typing.Iterator[rclpy.guard_condition.GuardCondition]

````

````{py:property} waitables
:canonical: ros_sugar.core.component.BaseComponent.waitables
:type: typing.Iterator[rclpy.waitable.Waitable]

````

````{py:property} executor
:canonical: ros_sugar.core.component.BaseComponent.executor
:type: typing.Optional[rclpy.executors.Executor]

````

````{py:property} context
:canonical: ros_sugar.core.component.BaseComponent.context
:type: rclpy.context.Context

````

````{py:property} default_callback_group
:canonical: ros_sugar.core.component.BaseComponent.default_callback_group
:type: rclpy.callback_groups.CallbackGroup

````

````{py:property} handle
:canonical: ros_sugar.core.component.BaseComponent.handle

````

````{py:method} get_name() -> str
:canonical: ros_sugar.core.component.BaseComponent.get_name

````

````{py:method} get_namespace() -> str
:canonical: ros_sugar.core.component.BaseComponent.get_namespace

````

````{py:method} get_clock() -> rclpy.clock.Clock
:canonical: ros_sugar.core.component.BaseComponent.get_clock

````

````{py:method} get_logger()
:canonical: ros_sugar.core.component.BaseComponent.get_logger

````

````{py:method} declare_parameter(name: str, value: typing.Any = None, descriptor: typing.Optional[rcl_interfaces.msg.ParameterDescriptor] = None, ignore_override: bool = False) -> rclpy.parameter.Parameter
:canonical: ros_sugar.core.component.BaseComponent.declare_parameter

````

````{py:method} declare_parameters(namespace: str, parameters: typing.List[typing.Union[typing.Tuple[str], typing.Tuple[str, rclpy.parameter.Parameter.Type], typing.Tuple[str, typing.Any, rcl_interfaces.msg.ParameterDescriptor]]], ignore_override: bool = False) -> typing.List[rclpy.parameter.Parameter]
:canonical: ros_sugar.core.component.BaseComponent.declare_parameters

````

````{py:method} undeclare_parameter(name: str)
:canonical: ros_sugar.core.component.BaseComponent.undeclare_parameter

````

````{py:method} has_parameter(name: str) -> bool
:canonical: ros_sugar.core.component.BaseComponent.has_parameter

````

````{py:method} get_parameter_types(names: typing.List[str]) -> typing.List[rclpy.parameter.Parameter.Type]
:canonical: ros_sugar.core.component.BaseComponent.get_parameter_types

````

````{py:method} get_parameter_type(name: str) -> rclpy.parameter.Parameter.Type
:canonical: ros_sugar.core.component.BaseComponent.get_parameter_type

````

````{py:method} get_parameters(names: typing.List[str]) -> typing.List[rclpy.parameter.Parameter]
:canonical: ros_sugar.core.component.BaseComponent.get_parameters

````

````{py:method} get_parameter(name: str) -> rclpy.parameter.Parameter
:canonical: ros_sugar.core.component.BaseComponent.get_parameter

````

````{py:method} get_parameter_or(name: str, alternative_value: typing.Optional[rclpy.parameter.Parameter] = None) -> rclpy.parameter.Parameter
:canonical: ros_sugar.core.component.BaseComponent.get_parameter_or

````

````{py:method} get_parameters_by_prefix(prefix: str) -> typing.Dict[str, typing.Optional[typing.Union[bool, int, float, str, bytes, typing.Sequence[bool], typing.Sequence[int], typing.Sequence[float], typing.Sequence[str]]]]
:canonical: ros_sugar.core.component.BaseComponent.get_parameters_by_prefix

````

````{py:method} set_parameters(parameter_list: typing.List[rclpy.parameter.Parameter]) -> typing.List[rcl_interfaces.msg.SetParametersResult]
:canonical: ros_sugar.core.component.BaseComponent.set_parameters

````

````{py:method} set_parameters_atomically(parameter_list: typing.List[rclpy.parameter.Parameter]) -> rcl_interfaces.msg.SetParametersResult
:canonical: ros_sugar.core.component.BaseComponent.set_parameters_atomically

````

````{py:method} add_pre_set_parameters_callback(callback: typing.Callable[[typing.List[rclpy.parameter.Parameter]], typing.List[rclpy.parameter.Parameter]]) -> None
:canonical: ros_sugar.core.component.BaseComponent.add_pre_set_parameters_callback

````

````{py:method} add_on_set_parameters_callback(callback: typing.Callable[[typing.List[rclpy.parameter.Parameter]], rcl_interfaces.msg.SetParametersResult]) -> None
:canonical: ros_sugar.core.component.BaseComponent.add_on_set_parameters_callback

````

````{py:method} add_post_set_parameters_callback(callback: typing.Callable[[typing.List[rclpy.parameter.Parameter]], None]) -> None
:canonical: ros_sugar.core.component.BaseComponent.add_post_set_parameters_callback

````

````{py:method} remove_pre_set_parameters_callback(callback: typing.Callable[[typing.List[rclpy.parameter.Parameter]], typing.List[rclpy.parameter.Parameter]]) -> None
:canonical: ros_sugar.core.component.BaseComponent.remove_pre_set_parameters_callback

````

````{py:method} remove_on_set_parameters_callback(callback: typing.Callable[[typing.List[rclpy.parameter.Parameter]], rcl_interfaces.msg.SetParametersResult]) -> None
:canonical: ros_sugar.core.component.BaseComponent.remove_on_set_parameters_callback

````

````{py:method} remove_post_set_parameters_callback(callback: typing.Callable[[typing.List[rclpy.parameter.Parameter]], None]) -> None
:canonical: ros_sugar.core.component.BaseComponent.remove_post_set_parameters_callback

````

````{py:method} describe_parameter(name: str) -> rcl_interfaces.msg.ParameterDescriptor
:canonical: ros_sugar.core.component.BaseComponent.describe_parameter

````

````{py:method} describe_parameters(names: typing.List[str]) -> typing.List[rcl_interfaces.msg.ParameterDescriptor]
:canonical: ros_sugar.core.component.BaseComponent.describe_parameters

````

````{py:method} set_descriptor(name: str, descriptor: rcl_interfaces.msg.ParameterDescriptor, alternative_value: typing.Optional[rcl_interfaces.msg.ParameterValue] = None) -> rcl_interfaces.msg.ParameterValue
:canonical: ros_sugar.core.component.BaseComponent.set_descriptor

````

````{py:method} add_waitable(waitable: rclpy.waitable.Waitable) -> None
:canonical: ros_sugar.core.component.BaseComponent.add_waitable

````

````{py:method} remove_waitable(waitable: rclpy.waitable.Waitable) -> None
:canonical: ros_sugar.core.component.BaseComponent.remove_waitable

````

````{py:method} resolve_topic_name(topic: str, *, only_expand: bool = False) -> str
:canonical: ros_sugar.core.component.BaseComponent.resolve_topic_name

````

````{py:method} resolve_service_name(service: str, *, only_expand: bool = False) -> str
:canonical: ros_sugar.core.component.BaseComponent.resolve_service_name

````

````{py:method} create_publisher(msg_type, topic: str, qos_profile: typing.Union[rclpy.qos.QoSProfile, int], *, callback_group: typing.Optional[rclpy.callback_groups.CallbackGroup] = None, event_callbacks: typing.Optional[rclpy.event_handler.PublisherEventCallbacks] = None, qos_overriding_options: typing.Optional[rclpy.qos_overriding_options.QoSOverridingOptions] = None, publisher_class: typing.Type[rclpy.publisher.Publisher] = Publisher) -> rclpy.publisher.Publisher
:canonical: ros_sugar.core.component.BaseComponent.create_publisher

````

````{py:method} create_subscription(msg_type, topic: str, callback: typing.Callable[[rclpy.node.MsgType], None], qos_profile: typing.Union[rclpy.qos.QoSProfile, int], *, callback_group: typing.Optional[rclpy.callback_groups.CallbackGroup] = None, event_callbacks: typing.Optional[rclpy.event_handler.SubscriptionEventCallbacks] = None, qos_overriding_options: typing.Optional[rclpy.qos_overriding_options.QoSOverridingOptions] = None, raw: bool = False) -> rclpy.subscription.Subscription
:canonical: ros_sugar.core.component.BaseComponent.create_subscription

````

````{py:method} create_service(srv_type, srv_name: str, callback: typing.Callable[[rclpy.node.SrvTypeRequest, rclpy.node.SrvTypeResponse], rclpy.node.SrvTypeResponse], *, qos_profile: rclpy.qos.QoSProfile = qos_profile_services_default, callback_group: rclpy.callback_groups.CallbackGroup = None) -> rclpy.service.Service
:canonical: ros_sugar.core.component.BaseComponent.create_service

````

````{py:method} create_timer(timer_period_sec: float, callback: typing.Callable, callback_group: rclpy.callback_groups.CallbackGroup = None, clock: rclpy.clock.Clock = None) -> rclpy.timer.Timer
:canonical: ros_sugar.core.component.BaseComponent.create_timer

````

````{py:method} create_guard_condition(callback: typing.Callable, callback_group: rclpy.callback_groups.CallbackGroup = None) -> rclpy.guard_condition.GuardCondition
:canonical: ros_sugar.core.component.BaseComponent.create_guard_condition

````

````{py:method} create_rate(frequency: float, clock: rclpy.clock.Clock = None) -> rclpy.timer.Rate
:canonical: ros_sugar.core.component.BaseComponent.create_rate

````

````{py:method} destroy_publisher(publisher: rclpy.publisher.Publisher) -> bool
:canonical: ros_sugar.core.component.BaseComponent.destroy_publisher

````

````{py:method} destroy_subscription(subscription: rclpy.subscription.Subscription) -> bool
:canonical: ros_sugar.core.component.BaseComponent.destroy_subscription

````

````{py:method} destroy_client(client: rclpy.client.Client) -> bool
:canonical: ros_sugar.core.component.BaseComponent.destroy_client

````

````{py:method} destroy_service(service: rclpy.service.Service) -> bool
:canonical: ros_sugar.core.component.BaseComponent.destroy_service

````

````{py:method} destroy_timer(timer: rclpy.timer.Timer) -> bool
:canonical: ros_sugar.core.component.BaseComponent.destroy_timer

````

````{py:method} destroy_guard_condition(guard: rclpy.guard_condition.GuardCondition) -> bool
:canonical: ros_sugar.core.component.BaseComponent.destroy_guard_condition

````

````{py:method} destroy_rate(rate: rclpy.timer.Rate) -> bool
:canonical: ros_sugar.core.component.BaseComponent.destroy_rate

````

````{py:method} destroy_node()
:canonical: ros_sugar.core.component.BaseComponent.destroy_node

````

````{py:method} get_publisher_names_and_types_by_node(node_name: str, node_namespace: str, no_demangle: bool = False) -> typing.List[typing.Tuple[str, typing.List[str]]]
:canonical: ros_sugar.core.component.BaseComponent.get_publisher_names_and_types_by_node

````

````{py:method} get_subscriber_names_and_types_by_node(node_name: str, node_namespace: str, no_demangle: bool = False) -> typing.List[typing.Tuple[str, typing.List[str]]]
:canonical: ros_sugar.core.component.BaseComponent.get_subscriber_names_and_types_by_node

````

````{py:method} get_service_names_and_types_by_node(node_name: str, node_namespace: str) -> typing.List[typing.Tuple[str, typing.List[str]]]
:canonical: ros_sugar.core.component.BaseComponent.get_service_names_and_types_by_node

````

````{py:method} get_client_names_and_types_by_node(node_name: str, node_namespace: str) -> typing.List[typing.Tuple[str, typing.List[str]]]
:canonical: ros_sugar.core.component.BaseComponent.get_client_names_and_types_by_node

````

````{py:method} get_topic_names_and_types(no_demangle: bool = False) -> typing.List[typing.Tuple[str, typing.List[str]]]
:canonical: ros_sugar.core.component.BaseComponent.get_topic_names_and_types

````

````{py:method} get_service_names_and_types() -> typing.List[typing.Tuple[str, typing.List[str]]]
:canonical: ros_sugar.core.component.BaseComponent.get_service_names_and_types

````

````{py:method} get_node_names() -> typing.List[str]
:canonical: ros_sugar.core.component.BaseComponent.get_node_names

````

````{py:method} get_fully_qualified_node_names() -> typing.List[str]
:canonical: ros_sugar.core.component.BaseComponent.get_fully_qualified_node_names

````

````{py:method} get_node_names_and_namespaces() -> typing.List[typing.Tuple[str, str]]
:canonical: ros_sugar.core.component.BaseComponent.get_node_names_and_namespaces

````

````{py:method} get_node_names_and_namespaces_with_enclaves() -> typing.List[typing.Tuple[str, str, str]]
:canonical: ros_sugar.core.component.BaseComponent.get_node_names_and_namespaces_with_enclaves

````

````{py:method} get_fully_qualified_name() -> str
:canonical: ros_sugar.core.component.BaseComponent.get_fully_qualified_name

````

````{py:method} count_publishers(topic_name: str) -> int
:canonical: ros_sugar.core.component.BaseComponent.count_publishers

````

````{py:method} count_subscribers(topic_name: str) -> int
:canonical: ros_sugar.core.component.BaseComponent.count_subscribers

````

````{py:method} get_publishers_info_by_topic(topic_name: str, no_mangle: bool = False) -> typing.List[rclpy.topic_endpoint_info.TopicEndpointInfo]
:canonical: ros_sugar.core.component.BaseComponent.get_publishers_info_by_topic

````

````{py:method} get_subscriptions_info_by_topic(topic_name: str, no_mangle: bool = False) -> typing.List[rclpy.topic_endpoint_info.TopicEndpointInfo]
:canonical: ros_sugar.core.component.BaseComponent.get_subscriptions_info_by_topic

````

````{py:method} wait_for_node(fully_qualified_node_name: str, timeout: float) -> bool
:canonical: ros_sugar.core.component.BaseComponent.wait_for_node

````

`````
