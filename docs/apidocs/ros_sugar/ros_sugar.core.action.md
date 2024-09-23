# {py:mod}`ros_sugar.core.action`

```{py:module} ros_sugar.core.action
```

```{autodoc2-docstring} ros_sugar.core.action
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`Action <ros_sugar.core.action.Action>`
  - ```{autodoc2-docstring} ros_sugar.core.action.Action
    :summary:
    ```
* - {py:obj}`LogInfo <ros_sugar.core.action.LogInfo>`
  - ```{autodoc2-docstring} ros_sugar.core.action.LogInfo
    :summary:
    ```
````

### API

`````{py:class} Action(method: typing.Callable, args: tuple = (), kwargs: typing.Optional[typing.Dict] = None)
:canonical: ros_sugar.core.action.Action

```{autodoc2-docstring} ros_sugar.core.action.Action
```

````{py:method} event_parser(method: callable, output_mapping: typing.Optional[str] = None, **new_kwargs)
:canonical: ros_sugar.core.action.Action.event_parser

```{autodoc2-docstring} ros_sugar.core.action.Action.event_parser
```

````

````{py:property} executable
:canonical: ros_sugar.core.action.Action.executable

```{autodoc2-docstring} ros_sugar.core.action.Action.executable
```

````

````{py:property} args
:canonical: ros_sugar.core.action.Action.args

```{autodoc2-docstring} ros_sugar.core.action.Action.args
```

````

````{py:property} kwargs
:canonical: ros_sugar.core.action.Action.kwargs

```{autodoc2-docstring} ros_sugar.core.action.Action.kwargs
```

````

````{py:property} parent_component
:canonical: ros_sugar.core.action.Action.parent_component

```{autodoc2-docstring} ros_sugar.core.action.Action.parent_component
```

````

````{py:property} action_name
:canonical: ros_sugar.core.action.Action.action_name
:type: str

```{autodoc2-docstring} ros_sugar.core.action.Action.action_name
```

````

````{py:property} component_action
:canonical: ros_sugar.core.action.Action.component_action
:type: bool

```{autodoc2-docstring} ros_sugar.core.action.Action.component_action
```

````

````{py:property} dictionary
:canonical: ros_sugar.core.action.Action.dictionary
:type: typing.Dict

```{autodoc2-docstring} ros_sugar.core.action.Action.dictionary
```

````

````{py:property} json
:canonical: ros_sugar.core.action.Action.json
:type: str

```{autodoc2-docstring} ros_sugar.core.action.Action.json
```

````

````{py:method} launch_action(monitor_node: typing.Optional[ros_sugar.core.node.BaseNode] = None) -> typing.Union[launch.actions.OpaqueCoroutine, launch.actions.OpaqueFunction]
:canonical: ros_sugar.core.action.Action.launch_action

```{autodoc2-docstring} ros_sugar.core.action.Action.launch_action
```

````

`````

`````{py:class} LogInfo(*, msg: str, logger_name: typing.Optional[str] = None, **kwargs)
:canonical: ros_sugar.core.action.LogInfo

Bases: {py:obj}`launch.actions.LogInfo`

```{autodoc2-docstring} ros_sugar.core.action.LogInfo
```

````{py:method} execute(context: launch.LaunchContext) -> None
:canonical: ros_sugar.core.action.LogInfo.execute

```{autodoc2-docstring} ros_sugar.core.action.LogInfo.execute
```

````

````{py:method} parse(entity: launch.frontend.Entity, parser: launch.frontend.Parser)
:canonical: ros_sugar.core.action.LogInfo.parse
:classmethod:

````

````{py:property} msg
:canonical: ros_sugar.core.action.LogInfo.msg
:type: typing.List[launch.substitution.Substitution]

````

````{py:property} condition
:canonical: ros_sugar.core.action.LogInfo.condition
:type: typing.Optional[launch.condition.Condition]

````

````{py:method} describe() -> typing.Text
:canonical: ros_sugar.core.action.LogInfo.describe

````

````{py:method} get_sub_entities() -> typing.List[launch.launch_description_entity.LaunchDescriptionEntity]
:canonical: ros_sugar.core.action.LogInfo.get_sub_entities

````

````{py:method} describe_sub_entities() -> typing.List[launch.launch_description_entity.LaunchDescriptionEntity]
:canonical: ros_sugar.core.action.LogInfo.describe_sub_entities

````

````{py:method} describe_conditional_sub_entities() -> typing.List[typing.Tuple[typing.Text, typing.Iterable[launch.launch_description_entity.LaunchDescriptionEntity]]]
:canonical: ros_sugar.core.action.LogInfo.describe_conditional_sub_entities

````

````{py:method} visit(context: launch.launch_context.LaunchContext) -> typing.Optional[typing.List[launch.launch_description_entity.LaunchDescriptionEntity]]
:canonical: ros_sugar.core.action.LogInfo.visit

````

````{py:method} get_asyncio_future() -> typing.Optional[asyncio.Future]
:canonical: ros_sugar.core.action.LogInfo.get_asyncio_future

````

`````
