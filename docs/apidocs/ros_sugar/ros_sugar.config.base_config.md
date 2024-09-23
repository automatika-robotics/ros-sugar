---
orphan: true
---

# {py:mod}`ros_sugar.config.base_config`

```{py:module} ros_sugar.config.base_config
```

```{autodoc2-docstring} ros_sugar.config.base_config
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`QoSConfig <ros_sugar.config.base_config.QoSConfig>`
  - ```{autodoc2-docstring} ros_sugar.config.base_config.QoSConfig
    :summary:
    ```
* - {py:obj}`BaseConfig <ros_sugar.config.base_config.BaseConfig>`
  - ```{autodoc2-docstring} ros_sugar.config.base_config.BaseConfig
    :summary:
    ```
* - {py:obj}`ComponentRunType <ros_sugar.config.base_config.ComponentRunType>`
  - ```{autodoc2-docstring} ros_sugar.config.base_config.ComponentRunType
    :summary:
    ```
* - {py:obj}`BaseComponentConfig <ros_sugar.config.base_config.BaseComponentConfig>`
  - ```{autodoc2-docstring} ros_sugar.config.base_config.BaseComponentConfig
    :summary:
    ```
````

### API

`````{py:class} QoSConfig
:canonical: ros_sugar.config.base_config.QoSConfig

Bases: {py:obj}`ros_sugar.config.base_attrs.BaseAttrs`

```{autodoc2-docstring} ros_sugar.config.base_config.QoSConfig
```

````{py:method} asdict(filter: typing.Optional[typing.Callable] = None) -> dict
:canonical: ros_sugar.config.base_config.QoSConfig.asdict

````

````{py:method} from_dict(dict_obj: typing.Dict) -> None
:canonical: ros_sugar.config.base_config.QoSConfig.from_dict

````

````{py:method} from_yaml(file_path: str, nested_root_name: str | None = None, get_common: bool = False) -> None
:canonical: ros_sugar.config.base_config.QoSConfig.from_yaml

````

````{py:method} to_json() -> typing.Union[str, bytes, bytearray]
:canonical: ros_sugar.config.base_config.QoSConfig.to_json

````

````{py:method} from_json(json_obj: typing.Union[str, bytes, bytearray]) -> None
:canonical: ros_sugar.config.base_config.QoSConfig.from_json

````

````{py:method} has_attribute(attr_name: str) -> bool
:canonical: ros_sugar.config.base_config.QoSConfig.has_attribute

````

````{py:method} get_attribute_type(attr_name: str) -> typing.Optional[type]
:canonical: ros_sugar.config.base_config.QoSConfig.get_attribute_type

````

````{py:method} update_value(attr_name: str, attr_value: typing.Any) -> bool
:canonical: ros_sugar.config.base_config.QoSConfig.update_value

````

`````

`````{py:class} BaseConfig
:canonical: ros_sugar.config.base_config.BaseConfig

Bases: {py:obj}`ros_sugar.config.base_attrs.BaseAttrs`

```{autodoc2-docstring} ros_sugar.config.base_config.BaseConfig
```

````{py:method} asdict(filter: typing.Optional[typing.Callable] = None) -> dict
:canonical: ros_sugar.config.base_config.BaseConfig.asdict

````

````{py:method} from_dict(dict_obj: typing.Dict) -> None
:canonical: ros_sugar.config.base_config.BaseConfig.from_dict

````

````{py:method} from_yaml(file_path: str, nested_root_name: str | None = None, get_common: bool = False) -> None
:canonical: ros_sugar.config.base_config.BaseConfig.from_yaml

````

````{py:method} to_json() -> typing.Union[str, bytes, bytearray]
:canonical: ros_sugar.config.base_config.BaseConfig.to_json

````

````{py:method} from_json(json_obj: typing.Union[str, bytes, bytearray]) -> None
:canonical: ros_sugar.config.base_config.BaseConfig.from_json

````

````{py:method} has_attribute(attr_name: str) -> bool
:canonical: ros_sugar.config.base_config.BaseConfig.has_attribute

````

````{py:method} get_attribute_type(attr_name: str) -> typing.Optional[type]
:canonical: ros_sugar.config.base_config.BaseConfig.get_attribute_type

````

````{py:method} update_value(attr_name: str, attr_value: typing.Any) -> bool
:canonical: ros_sugar.config.base_config.BaseConfig.update_value

````

`````

`````{py:class} ComponentRunType
:canonical: ros_sugar.config.base_config.ComponentRunType

Bases: {py:obj}`enum.Enum`

```{autodoc2-docstring} ros_sugar.config.base_config.ComponentRunType
```

````{py:method} to_str(enum_value) -> str
:canonical: ros_sugar.config.base_config.ComponentRunType.to_str
:classmethod:

```{autodoc2-docstring} ros_sugar.config.base_config.ComponentRunType.to_str
```

````

````{py:method} name()
:canonical: ros_sugar.config.base_config.ComponentRunType.name

````

````{py:method} value()
:canonical: ros_sugar.config.base_config.ComponentRunType.value

````

`````

`````{py:class} BaseComponentConfig
:canonical: ros_sugar.config.base_config.BaseComponentConfig

Bases: {py:obj}`ros_sugar.config.base_config.BaseConfig`

```{autodoc2-docstring} ros_sugar.config.base_config.BaseComponentConfig
```

````{py:method} asdict(filter: typing.Optional[typing.Callable] = None) -> dict
:canonical: ros_sugar.config.base_config.BaseComponentConfig.asdict

````

````{py:method} from_dict(dict_obj: typing.Dict) -> None
:canonical: ros_sugar.config.base_config.BaseComponentConfig.from_dict

````

````{py:method} from_yaml(file_path: str, nested_root_name: str | None = None, get_common: bool = False) -> None
:canonical: ros_sugar.config.base_config.BaseComponentConfig.from_yaml

````

````{py:method} to_json() -> typing.Union[str, bytes, bytearray]
:canonical: ros_sugar.config.base_config.BaseComponentConfig.to_json

````

````{py:method} from_json(json_obj: typing.Union[str, bytes, bytearray]) -> None
:canonical: ros_sugar.config.base_config.BaseComponentConfig.from_json

````

````{py:method} has_attribute(attr_name: str) -> bool
:canonical: ros_sugar.config.base_config.BaseComponentConfig.has_attribute

````

````{py:method} get_attribute_type(attr_name: str) -> typing.Optional[type]
:canonical: ros_sugar.config.base_config.BaseComponentConfig.get_attribute_type

````

````{py:method} update_value(attr_name: str, attr_value: typing.Any) -> bool
:canonical: ros_sugar.config.base_config.BaseComponentConfig.update_value

````

`````
