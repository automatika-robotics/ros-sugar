---
orphan: true
---

# {py:mod}`ros_sugar.launch.launch_actions`

```{py:module} ros_sugar.launch.launch_actions
```

```{autodoc2-docstring} ros_sugar.launch.launch_actions
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`ComponentLaunchAction <ros_sugar.launch.launch_actions.ComponentLaunchAction>`
  - ```{autodoc2-docstring} ros_sugar.launch.launch_actions.ComponentLaunchAction
    :summary:
    ```
````

### API

`````{py:class} ComponentLaunchAction(*, node: typing.Union[ros_sugar.core.component.BaseComponent, ros_sugar.core.BaseNode], name: typing.Union[str, typing.List[launch.Substitution], None] = 'node_name', namespace: typing.Union[str, typing.List[launch.Substitution], None] = None, log_level: rclpy.impl.logging_severity.LoggingSeverity = LoggingSeverity.INFO, **kwargs)
:canonical: ros_sugar.launch.launch_actions.ComponentLaunchAction

Bases: {py:obj}`launch_ros.actions.Node`

```{autodoc2-docstring} ros_sugar.launch.launch_actions.ComponentLaunchAction
```

````{py:property} name
:canonical: ros_sugar.launch.launch_actions.ComponentLaunchAction.name
:type: typing.Union[str, typing.List[launch.Substitution], None]

```{autodoc2-docstring} ros_sugar.launch.launch_actions.ComponentLaunchAction.name
```

````

````{py:method} execute(context: launch.LaunchContext) -> typing.Optional[typing.List[launch.action.Action]]
:canonical: ros_sugar.launch.launch_actions.ComponentLaunchAction.execute

```{autodoc2-docstring} ros_sugar.launch.launch_actions.ComponentLaunchAction.execute
```

````

````{py:property} executor
:canonical: ros_sugar.launch.launch_actions.ComponentLaunchAction.executor

```{autodoc2-docstring} ros_sugar.launch.launch_actions.ComponentLaunchAction.executor
```

````

````{py:method} shutdown()
:canonical: ros_sugar.launch.launch_actions.ComponentLaunchAction.shutdown

```{autodoc2-docstring} ros_sugar.launch.launch_actions.ComponentLaunchAction.shutdown
```

````

````{py:method} parse_nested_parameters(params, parser)
:canonical: ros_sugar.launch.launch_actions.ComponentLaunchAction.parse_nested_parameters
:staticmethod:

````

````{py:method} parse(entity: launch.frontend.Entity, parser: launch.frontend.Parser)
:canonical: ros_sugar.launch.launch_actions.ComponentLaunchAction.parse
:classmethod:

````

````{py:property} node_package
:canonical: ros_sugar.launch.launch_actions.ComponentLaunchAction.node_package

````

````{py:property} node_executable
:canonical: ros_sugar.launch.launch_actions.ComponentLaunchAction.node_executable

````

````{py:property} node_name
:canonical: ros_sugar.launch.launch_actions.ComponentLaunchAction.node_name

````

````{py:property} expanded_node_namespace
:canonical: ros_sugar.launch.launch_actions.ComponentLaunchAction.expanded_node_namespace

````

````{py:property} expanded_remapping_rules
:canonical: ros_sugar.launch.launch_actions.ComponentLaunchAction.expanded_remapping_rules

````

````{py:property} cmd
:canonical: ros_sugar.launch.launch_actions.ComponentLaunchAction.cmd

````

````{py:property} cwd
:canonical: ros_sugar.launch.launch_actions.ComponentLaunchAction.cwd

````

````{py:property} env
:canonical: ros_sugar.launch.launch_actions.ComponentLaunchAction.env

````

````{py:property} additional_env
:canonical: ros_sugar.launch.launch_actions.ComponentLaunchAction.additional_env

````

````{py:property} process_description
:canonical: ros_sugar.launch.launch_actions.ComponentLaunchAction.process_description

````

````{py:property} shell
:canonical: ros_sugar.launch.launch_actions.ComponentLaunchAction.shell

````

````{py:property} emulate_tty
:canonical: ros_sugar.launch.launch_actions.ComponentLaunchAction.emulate_tty

````

````{py:property} sigkill_timeout
:canonical: ros_sugar.launch.launch_actions.ComponentLaunchAction.sigkill_timeout

````

````{py:property} sigterm_timeout
:canonical: ros_sugar.launch.launch_actions.ComponentLaunchAction.sigterm_timeout

````

````{py:property} output
:canonical: ros_sugar.launch.launch_actions.ComponentLaunchAction.output

````

````{py:property} process_details
:canonical: ros_sugar.launch.launch_actions.ComponentLaunchAction.process_details

````

````{py:method} get_sub_entities()
:canonical: ros_sugar.launch.launch_actions.ComponentLaunchAction.get_sub_entities

````

````{py:method} prepare(context: launch.launch_context.LaunchContext)
:canonical: ros_sugar.launch.launch_actions.ComponentLaunchAction.prepare

````

````{py:method} get_asyncio_future() -> typing.Optional[asyncio.Future]
:canonical: ros_sugar.launch.launch_actions.ComponentLaunchAction.get_asyncio_future

````

````{py:method} get_stdout()
:canonical: ros_sugar.launch.launch_actions.ComponentLaunchAction.get_stdout

````

````{py:method} get_stderr()
:canonical: ros_sugar.launch.launch_actions.ComponentLaunchAction.get_stderr

````

````{py:property} return_code
:canonical: ros_sugar.launch.launch_actions.ComponentLaunchAction.return_code

````

````{py:property} condition
:canonical: ros_sugar.launch.launch_actions.ComponentLaunchAction.condition
:type: typing.Optional[launch.condition.Condition]

````

````{py:method} describe() -> typing.Text
:canonical: ros_sugar.launch.launch_actions.ComponentLaunchAction.describe

````

````{py:method} describe_sub_entities() -> typing.List[launch.launch_description_entity.LaunchDescriptionEntity]
:canonical: ros_sugar.launch.launch_actions.ComponentLaunchAction.describe_sub_entities

````

````{py:method} describe_conditional_sub_entities() -> typing.List[typing.Tuple[typing.Text, typing.Iterable[launch.launch_description_entity.LaunchDescriptionEntity]]]
:canonical: ros_sugar.launch.launch_actions.ComponentLaunchAction.describe_conditional_sub_entities

````

````{py:method} visit(context: launch.launch_context.LaunchContext) -> typing.Optional[typing.List[launch.launch_description_entity.LaunchDescriptionEntity]]
:canonical: ros_sugar.launch.launch_actions.ComponentLaunchAction.visit

````

`````
