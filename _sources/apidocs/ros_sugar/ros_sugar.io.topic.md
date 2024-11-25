# {py:mod}`ros_sugar.io.topic`

```{py:module} ros_sugar.io.topic
```

```{autodoc2-docstring} ros_sugar.io.topic
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`Topic <ros_sugar.io.topic.Topic>`
  - ```{autodoc2-docstring} ros_sugar.io.topic.Topic
    :summary:
    ```
* - {py:obj}`AllowedTopic <ros_sugar.io.topic.AllowedTopic>`
  - ```{autodoc2-docstring} ros_sugar.io.topic.AllowedTopic
    :summary:
    ```
* - {py:obj}`RestrictedTopicsConfig <ros_sugar.io.topic.RestrictedTopicsConfig>`
  - ```{autodoc2-docstring} ros_sugar.io.topic.RestrictedTopicsConfig
    :summary:
    ```
````

### Functions

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`get_all_msg_types <ros_sugar.io.topic.get_all_msg_types>`
  - ```{autodoc2-docstring} ros_sugar.io.topic.get_all_msg_types
    :summary:
    ```
* - {py:obj}`get_msg_type <ros_sugar.io.topic.get_msg_type>`
  - ```{autodoc2-docstring} ros_sugar.io.topic.get_msg_type
    :summary:
    ```
* - {py:obj}`msg_type_validator <ros_sugar.io.topic.msg_type_validator>`
  - ```{autodoc2-docstring} ros_sugar.io.topic.msg_type_validator
    :summary:
    ```
````

### API

````{py:function} get_all_msg_types(msg_types_module: types.ModuleType = supported_types, additional_types: typing.Optional[typing.List[type[ros_sugar.io.supported_types.SupportedType]]] = None) -> typing.List[type[ros_sugar.io.supported_types.SupportedType]]
:canonical: ros_sugar.io.topic.get_all_msg_types

```{autodoc2-docstring} ros_sugar.io.topic.get_all_msg_types
```
````

````{py:function} get_msg_type(type_name: typing.Union[type[ros_sugar.io.supported_types.SupportedType], str], msg_types_module: typing.Optional[types.ModuleType] = supported_types, additional_types: typing.Optional[typing.List[type[ros_sugar.io.supported_types.SupportedType]]] = None) -> typing.Union[type[ros_sugar.io.supported_types.SupportedType], str]
:canonical: ros_sugar.io.topic.get_msg_type

```{autodoc2-docstring} ros_sugar.io.topic.get_msg_type
```
````

````{py:function} msg_type_validator(*_: typing.Any, value)
:canonical: ros_sugar.io.topic.msg_type_validator

```{autodoc2-docstring} ros_sugar.io.topic.msg_type_validator
```
````

`````{py:class} Topic
:canonical: ros_sugar.io.topic.Topic

Bases: {py:obj}`ros_sugar.config.BaseAttrs`

```{autodoc2-docstring} ros_sugar.io.topic.Topic
```

````{py:method} asdict(filter: typing.Optional[typing.Callable] = None) -> typing.Dict
:canonical: ros_sugar.io.topic.Topic.asdict

````

````{py:method} from_dict(dict_obj: typing.Dict) -> None
:canonical: ros_sugar.io.topic.Topic.from_dict

````

````{py:method} from_yaml(file_path: str, nested_root_name: str | None = None, get_common: bool = False) -> None
:canonical: ros_sugar.io.topic.Topic.from_yaml

````

````{py:method} to_json() -> typing.Union[str, bytes, bytearray]
:canonical: ros_sugar.io.topic.Topic.to_json

````

````{py:method} from_json(json_obj: typing.Union[str, bytes, bytearray]) -> None
:canonical: ros_sugar.io.topic.Topic.from_json

````

````{py:method} has_attribute(attr_name: str) -> bool
:canonical: ros_sugar.io.topic.Topic.has_attribute

````

````{py:method} get_attribute_type(attr_name: str) -> typing.Optional[type]
:canonical: ros_sugar.io.topic.Topic.get_attribute_type

````

````{py:method} update_value(attr_name: str, attr_value: typing.Any) -> bool
:canonical: ros_sugar.io.topic.Topic.update_value

````

`````

`````{py:class} AllowedTopic
:canonical: ros_sugar.io.topic.AllowedTopic

Bases: {py:obj}`ros_sugar.config.BaseAttrs`

```{autodoc2-docstring} ros_sugar.io.topic.AllowedTopic
```

````{py:method} asdict(filter: typing.Optional[typing.Callable] = None) -> typing.Dict
:canonical: ros_sugar.io.topic.AllowedTopic.asdict

````

````{py:method} from_dict(dict_obj: typing.Dict) -> None
:canonical: ros_sugar.io.topic.AllowedTopic.from_dict

````

````{py:method} from_yaml(file_path: str, nested_root_name: str | None = None, get_common: bool = False) -> None
:canonical: ros_sugar.io.topic.AllowedTopic.from_yaml

````

````{py:method} to_json() -> typing.Union[str, bytes, bytearray]
:canonical: ros_sugar.io.topic.AllowedTopic.to_json

````

````{py:method} from_json(json_obj: typing.Union[str, bytes, bytearray]) -> None
:canonical: ros_sugar.io.topic.AllowedTopic.from_json

````

````{py:method} has_attribute(attr_name: str) -> bool
:canonical: ros_sugar.io.topic.AllowedTopic.has_attribute

````

````{py:method} get_attribute_type(attr_name: str) -> typing.Optional[type]
:canonical: ros_sugar.io.topic.AllowedTopic.get_attribute_type

````

````{py:method} update_value(attr_name: str, attr_value: typing.Any) -> bool
:canonical: ros_sugar.io.topic.AllowedTopic.update_value

````

`````

`````{py:class} RestrictedTopicsConfig
:canonical: ros_sugar.io.topic.RestrictedTopicsConfig

```{autodoc2-docstring} ros_sugar.io.topic.RestrictedTopicsConfig
```

````{py:method} keys() -> typing.List[str]
:canonical: ros_sugar.io.topic.RestrictedTopicsConfig.keys
:classmethod:

```{autodoc2-docstring} ros_sugar.io.topic.RestrictedTopicsConfig.keys
```

````

````{py:method} types(key: str) -> typing.List[typing.Union[ros_sugar.io.supported_types.SupportedType, str]]
:canonical: ros_sugar.io.topic.RestrictedTopicsConfig.types
:classmethod:

```{autodoc2-docstring} ros_sugar.io.topic.RestrictedTopicsConfig.types
```

````

````{py:method} required_number(key: str) -> int
:canonical: ros_sugar.io.topic.RestrictedTopicsConfig.required_number
:classmethod:

```{autodoc2-docstring} ros_sugar.io.topic.RestrictedTopicsConfig.required_number
```

````

````{py:method} optional_number(key: str) -> int
:canonical: ros_sugar.io.topic.RestrictedTopicsConfig.optional_number
:classmethod:

```{autodoc2-docstring} ros_sugar.io.topic.RestrictedTopicsConfig.optional_number
```

````

`````
