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

`````{py:class} BaseComponent(component_name: str, inputs: typing.Optional[typing.Sequence[ros_sugar.io.topic.Topic]] = None, outputs: typing.Optional[typing.Sequence[ros_sugar.io.topic.Topic]] = None, config: typing.Optional[ros_sugar.config.base_config.BaseComponentConfig] = None, config_file: typing.Optional[str] = None, callback_group: typing.Optional[rclpy.callback_groups.CallbackGroup] = None, enable_health_broadcast: bool = True, fallbacks: typing.Optional[ros_sugar.core.fallbacks.ComponentFallbacks] = None, main_action_type: typing.Optional[type] = None, main_srv_type: typing.Optional[type] = None, **kwargs)
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

````{py:method} get_change_parameters_msg_from_config(config: ros_sugar.config.base_config.BaseComponentConfig) -> automatika_ros_sugar.srv.ChangeParameters.Request
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

````{py:method} custom_on_cleanup() -> None
:canonical: ros_sugar.core.component.BaseComponent.custom_on_cleanup

```{autodoc2-docstring} ros_sugar.core.component.BaseComponent.custom_on_cleanup
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

`````
