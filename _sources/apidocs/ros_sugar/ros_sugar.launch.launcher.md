# {py:mod}`ros_sugar.launch.launcher`

```{py:module} ros_sugar.launch.launcher
```

```{autodoc2-docstring} ros_sugar.launch.launcher
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`Launcher <ros_sugar.launch.launcher.Launcher>`
  - ```{autodoc2-docstring} ros_sugar.launch.launcher.Launcher
    :summary:
    ```
````

### API

`````{py:class} Launcher(namespace: str = '', config_file: typing.Optional[str] = None, enable_monitoring: bool = True, activation_timeout: typing.Optional[float] = None)
:canonical: ros_sugar.launch.launcher.Launcher

```{autodoc2-docstring} ros_sugar.launch.launcher.Launcher
```

````{py:method} add_pkg(components: typing.List[ros_sugar.core.component.BaseComponent], package_name: typing.Optional[str] = None, executable_entry_point: typing.Optional[str] = 'executable', events_actions: typing.Dict[ros_sugar.core.event.Event, typing.Union[ros_sugar.core.action.Action, launch.action.Action, typing.List[typing.Union[ros_sugar.core.action.Action, launch.action.Action]]]] | None = None, multiprocessing: bool = False, activate_all_components_on_start: bool = True, components_to_activate_on_start: typing.Optional[typing.List[ros_sugar.core.component.BaseComponent]] = None, ros_log_level: typing.Optional[str] = None)
:canonical: ros_sugar.launch.launcher.Launcher.add_pkg

```{autodoc2-docstring} ros_sugar.launch.launcher.Launcher.add_pkg
```

````

````{py:method} start(node_name: str, **_) -> ros_sugar.utils.SomeEntitiesType
:canonical: ros_sugar.launch.launcher.Launcher.start

```{autodoc2-docstring} ros_sugar.launch.launcher.Launcher.start
```

````

````{py:method} stop(node_name: str, **_) -> ros_sugar.utils.SomeEntitiesType
:canonical: ros_sugar.launch.launcher.Launcher.stop

```{autodoc2-docstring} ros_sugar.launch.launcher.Launcher.stop
```

````

````{py:method} restart(node_name: str, **_) -> ros_sugar.utils.SomeEntitiesType
:canonical: ros_sugar.launch.launcher.Launcher.restart

```{autodoc2-docstring} ros_sugar.launch.launcher.Launcher.restart
```

````

````{py:property} fallback_rate
:canonical: ros_sugar.launch.launcher.Launcher.fallback_rate
:type: typing.Dict

```{autodoc2-docstring} ros_sugar.launch.launcher.Launcher.fallback_rate
```

````

````{py:method} on_fail(action_name: str, max_retries: typing.Optional[int] = None) -> None
:canonical: ros_sugar.launch.launcher.Launcher.on_fail

```{autodoc2-docstring} ros_sugar.launch.launcher.Launcher.on_fail
```

````

````{py:method} configure(config_file: str, component_name: str | None = None)
:canonical: ros_sugar.launch.launcher.Launcher.configure

```{autodoc2-docstring} ros_sugar.launch.launcher.Launcher.configure
```

````

````{py:method} add_py_executable(path_to_executable: str, name: str = 'python3')
:canonical: ros_sugar.launch.launcher.Launcher.add_py_executable

```{autodoc2-docstring} ros_sugar.launch.launcher.Launcher.add_py_executable
```

````

````{py:method} add_method(method: typing.Callable | typing.Awaitable, args: typing.Iterable | None = None, kwargs: typing.Dict | None = None)
:canonical: ros_sugar.launch.launcher.Launcher.add_method

```{autodoc2-docstring} ros_sugar.launch.launcher.Launcher.add_method
```

````

````{py:method} bringup(config_file: str | None = None, introspect: bool = False, launch_debug: bool = False, ros_log_level: str = 'info')
:canonical: ros_sugar.launch.launcher.Launcher.bringup

```{autodoc2-docstring} ros_sugar.launch.launcher.Launcher.bringup
```

````

`````
