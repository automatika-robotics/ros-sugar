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

`````{py:class} Monitor(components_names: typing.List[str], enable_health_status_monitoring: bool = True, events_actions: typing.Optional[typing.Dict[ros_sugar.core.event.Event, typing.List[ros_sugar.core.action.Action]]] = None, events_to_emit: typing.Optional[typing.List[ros_sugar.core.event.Event]] = None, config: typing.Optional[ros_sugar.config.BaseConfig] = None, services_components: typing.Optional[typing.List[ros_sugar.core.component.BaseComponent]] = None, action_servers_components: typing.Optional[typing.List[ros_sugar.core.component.BaseComponent]] = None, activate_on_start: typing.Optional[typing.List[ros_sugar.core.component.BaseComponent]] = None, activation_timeout: typing.Optional[float] = None, activation_attempt_time: float = 1.0, start_on_init: bool = False, component_name: str = 'monitor', callback_group: typing.Optional[typing.Union[rclpy.callback_groups.MutuallyExclusiveCallbackGroup, rclpy.callback_groups.ReentrantCallbackGroup]] = None, *args, **kwargs)
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

`````
