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

`````
