# {py:mod}`ros_sugar.base_clients`

```{py:module} ros_sugar.base_clients
```

```{autodoc2-docstring} ros_sugar.base_clients
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`ServiceClientConfig <ros_sugar.base_clients.ServiceClientConfig>`
  - ```{autodoc2-docstring} ros_sugar.base_clients.ServiceClientConfig
    :summary:
    ```
* - {py:obj}`ActionClientConfig <ros_sugar.base_clients.ActionClientConfig>`
  - ```{autodoc2-docstring} ros_sugar.base_clients.ActionClientConfig
    :summary:
    ```
* - {py:obj}`ServiceClientHandler <ros_sugar.base_clients.ServiceClientHandler>`
  - ```{autodoc2-docstring} ros_sugar.base_clients.ServiceClientHandler
    :summary:
    ```
* - {py:obj}`ActionClientHandler <ros_sugar.base_clients.ActionClientHandler>`
  - ```{autodoc2-docstring} ros_sugar.base_clients.ActionClientHandler
    :summary:
    ```
````

### API

`````{py:class} ServiceClientConfig
:canonical: ros_sugar.base_clients.ServiceClientConfig

Bases: {py:obj}`ros_sugar.config.BaseAttrs`

```{autodoc2-docstring} ros_sugar.base_clients.ServiceClientConfig
```

````{py:method} asdict(filter: typing.Optional[typing.Callable] = None) -> typing.Dict
:canonical: ros_sugar.base_clients.ServiceClientConfig.asdict

````

````{py:method} from_dict(dict_obj: typing.Dict) -> None
:canonical: ros_sugar.base_clients.ServiceClientConfig.from_dict

````

````{py:method} from_yaml(file_path: str, nested_root_name: typing.Union[str, None] = None, get_common: bool = False) -> None
:canonical: ros_sugar.base_clients.ServiceClientConfig.from_yaml

````

````{py:method} to_json() -> typing.Union[str, bytes, bytearray]
:canonical: ros_sugar.base_clients.ServiceClientConfig.to_json

````

````{py:method} from_json(json_obj: typing.Union[str, bytes, bytearray]) -> None
:canonical: ros_sugar.base_clients.ServiceClientConfig.from_json

````

````{py:method} has_attribute(attr_name: str) -> bool
:canonical: ros_sugar.base_clients.ServiceClientConfig.has_attribute

````

````{py:method} get_attribute_type(attr_name: str) -> typing.Optional[type]
:canonical: ros_sugar.base_clients.ServiceClientConfig.get_attribute_type

````

````{py:method} update_value(attr_name: str, attr_value: typing.Any) -> bool
:canonical: ros_sugar.base_clients.ServiceClientConfig.update_value

````

`````

`````{py:class} ActionClientConfig
:canonical: ros_sugar.base_clients.ActionClientConfig

Bases: {py:obj}`ros_sugar.config.BaseAttrs`

```{autodoc2-docstring} ros_sugar.base_clients.ActionClientConfig
```

````{py:method} asdict(filter: typing.Optional[typing.Callable] = None) -> typing.Dict
:canonical: ros_sugar.base_clients.ActionClientConfig.asdict

````

````{py:method} from_dict(dict_obj: typing.Dict) -> None
:canonical: ros_sugar.base_clients.ActionClientConfig.from_dict

````

````{py:method} from_yaml(file_path: str, nested_root_name: typing.Union[str, None] = None, get_common: bool = False) -> None
:canonical: ros_sugar.base_clients.ActionClientConfig.from_yaml

````

````{py:method} to_json() -> typing.Union[str, bytes, bytearray]
:canonical: ros_sugar.base_clients.ActionClientConfig.to_json

````

````{py:method} from_json(json_obj: typing.Union[str, bytes, bytearray]) -> None
:canonical: ros_sugar.base_clients.ActionClientConfig.from_json

````

````{py:method} has_attribute(attr_name: str) -> bool
:canonical: ros_sugar.base_clients.ActionClientConfig.has_attribute

````

````{py:method} get_attribute_type(attr_name: str) -> typing.Optional[type]
:canonical: ros_sugar.base_clients.ActionClientConfig.get_attribute_type

````

````{py:method} update_value(attr_name: str, attr_value: typing.Any) -> bool
:canonical: ros_sugar.base_clients.ActionClientConfig.update_value

````

`````

`````{py:class} ServiceClientHandler(client_node: rclpy.node.Node, config: typing.Optional[ros_sugar.base_clients.ServiceClientConfig] = None, srv_name: typing.Optional[str] = None, srv_type: typing.Optional[type] = None)
:canonical: ros_sugar.base_clients.ServiceClientHandler

```{autodoc2-docstring} ros_sugar.base_clients.ServiceClientHandler
```

````{py:method} send_request(req_msg, executor: typing.Optional[rclpy.executors.Executor] = None)
:canonical: ros_sugar.base_clients.ServiceClientHandler.send_request

```{autodoc2-docstring} ros_sugar.base_clients.ServiceClientHandler.send_request
```

````

`````

`````{py:class} ActionClientHandler(client_node: rclpy.node.Node, config: typing.Optional[ros_sugar.base_clients.ActionClientConfig] = None, action_name: typing.Optional[str] = None, action_type: typing.Optional[type] = None)
:canonical: ros_sugar.base_clients.ActionClientHandler

```{autodoc2-docstring} ros_sugar.base_clients.ActionClientHandler
```

````{py:method} reset()
:canonical: ros_sugar.base_clients.ActionClientHandler.reset

```{autodoc2-docstring} ros_sugar.base_clients.ActionClientHandler.reset
```

````

````{py:method} send_request(request_msg: typing.Any, wait_until_first_feedback: bool = True) -> bool
:canonical: ros_sugar.base_clients.ActionClientHandler.send_request

```{autodoc2-docstring} ros_sugar.base_clients.ActionClientHandler.send_request
```

````

````{py:method} action_response_callback(future)
:canonical: ros_sugar.base_clients.ActionClientHandler.action_response_callback

```{autodoc2-docstring} ros_sugar.base_clients.ActionClientHandler.action_response_callback
```

````

````{py:method} action_result_callback(future)
:canonical: ros_sugar.base_clients.ActionClientHandler.action_result_callback

```{autodoc2-docstring} ros_sugar.base_clients.ActionClientHandler.action_result_callback
```

````

````{py:method} action_feedback_callback(feedback_msg: typing.Any)
:canonical: ros_sugar.base_clients.ActionClientHandler.action_feedback_callback

```{autodoc2-docstring} ros_sugar.base_clients.ActionClientHandler.action_feedback_callback
```

````

````{py:method} got_new_feedback() -> bool
:canonical: ros_sugar.base_clients.ActionClientHandler.got_new_feedback

```{autodoc2-docstring} ros_sugar.base_clients.ActionClientHandler.got_new_feedback
```

````

`````
