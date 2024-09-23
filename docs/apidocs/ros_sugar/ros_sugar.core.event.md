# {py:mod}`ros_sugar.core.event`

```{py:module} ros_sugar.core.event
```

```{autodoc2-docstring} ros_sugar.core.event
:allowtitles:
```

## Module Contents

### Classes

````{list-table}
:class: autosummary longtable
:align: left

* - {py:obj}`InternalEvent <ros_sugar.core.event.InternalEvent>`
  - ```{autodoc2-docstring} ros_sugar.core.event.InternalEvent
    :summary:
    ```
* - {py:obj}`OnInternalEvent <ros_sugar.core.event.OnInternalEvent>`
  - ```{autodoc2-docstring} ros_sugar.core.event.OnInternalEvent
    :summary:
    ```
* - {py:obj}`Operand <ros_sugar.core.event.Operand>`
  - ```{autodoc2-docstring} ros_sugar.core.event.Operand
    :summary:
    ```
* - {py:obj}`Event <ros_sugar.core.event.Event>`
  - ```{autodoc2-docstring} ros_sugar.core.event.Event
    :summary:
    ```
````

### API

`````{py:class} InternalEvent(event_name: str)
:canonical: ros_sugar.core.event.InternalEvent

Bases: {py:obj}`launch.event.Event`

```{autodoc2-docstring} ros_sugar.core.event.InternalEvent
```

````{py:property} event_name
:canonical: ros_sugar.core.event.InternalEvent.event_name

```{autodoc2-docstring} ros_sugar.core.event.InternalEvent.event_name
```

````

`````

`````{py:class} OnInternalEvent(*, internal_event_name: str, entities: ros_sugar.core.event.SomeEntitiesType, handle_once: bool = False)
:canonical: ros_sugar.core.event.OnInternalEvent

Bases: {py:obj}`launch.event_handler.EventHandler`

```{autodoc2-docstring} ros_sugar.core.event.OnInternalEvent
```

````{py:property} entities
:canonical: ros_sugar.core.event.OnInternalEvent.entities

````

````{py:method} describe() -> typing.Tuple[typing.Text, typing.List[launch.some_entities_type.SomeEntitiesType]]
:canonical: ros_sugar.core.event.OnInternalEvent.describe

````

````{py:method} handle(event: launch.event.Event, context: launch.launch_context.LaunchContext) -> typing.Optional[launch.some_entities_type.SomeEntitiesType]
:canonical: ros_sugar.core.event.OnInternalEvent.handle

````

````{py:property} handle_once
:canonical: ros_sugar.core.event.OnInternalEvent.handle_once

````

````{py:property} handler_description
:canonical: ros_sugar.core.event.OnInternalEvent.handler_description

````

````{py:property} matcher_description
:canonical: ros_sugar.core.event.OnInternalEvent.matcher_description

````

````{py:method} matches(event: launch.event.Event) -> bool
:canonical: ros_sugar.core.event.OnInternalEvent.matches

````

`````

````{py:class} Operand(ros_message: typing.Any, attributes: typing.List[str])
:canonical: ros_sugar.core.event.Operand

```{autodoc2-docstring} ros_sugar.core.event.Operand
```

````

`````{py:class} Event(event_name: str, event_source: typing.Union[ros_sugar.io.topic.Topic, str, typing.Dict], trigger_value: typing.Union[float, int, bool, str, None], nested_attributes: typing.Union[str, typing.List[str]], topic_template: typing.Optional[ros_sugar.io.topic.Topic] = None)
:canonical: ros_sugar.core.event.Event

```{autodoc2-docstring} ros_sugar.core.event.Event
```

````{py:property} under_processing
:canonical: ros_sugar.core.event.Event.under_processing
:type: bool

```{autodoc2-docstring} ros_sugar.core.event.Event.under_processing
```

````

````{py:property} name
:canonical: ros_sugar.core.event.Event.name
:type: str

```{autodoc2-docstring} ros_sugar.core.event.Event.name
```

````

````{py:method} clear() -> None
:canonical: ros_sugar.core.event.Event.clear

```{autodoc2-docstring} ros_sugar.core.event.Event.clear
```

````

````{py:method} trig() -> None
:canonical: ros_sugar.core.event.Event.trig

```{autodoc2-docstring} ros_sugar.core.event.Event.trig
```

````

````{py:property} dictionary
:canonical: ros_sugar.core.event.Event.dictionary
:type: typing.Dict

```{autodoc2-docstring} ros_sugar.core.event.Event.dictionary
```

````

````{py:method} set_dictionary(dict_obj, topic_template: ros_sugar.io.topic.Topic)
:canonical: ros_sugar.core.event.Event.set_dictionary

```{autodoc2-docstring} ros_sugar.core.event.Event.set_dictionary
```

````

````{py:property} json
:canonical: ros_sugar.core.event.Event.json
:type: str

```{autodoc2-docstring} ros_sugar.core.event.Event.json
```

````

````{py:method} callback(msg: typing.Any) -> None
:canonical: ros_sugar.core.event.Event.callback

```{autodoc2-docstring} ros_sugar.core.event.Event.callback
```

````

````{py:method} register_method(method_name: str, method: typing.Callable[..., typing.Any]) -> None
:canonical: ros_sugar.core.event.Event.register_method

```{autodoc2-docstring} ros_sugar.core.event.Event.register_method
```

````

````{py:method} register_actions(actions: typing.Union[ros_sugar.core.action.Action, typing.List[ros_sugar.core.action.Action]]) -> None
:canonical: ros_sugar.core.event.Event.register_actions

```{autodoc2-docstring} ros_sugar.core.event.Event.register_actions
```

````

````{py:method} clear_actions() -> None
:canonical: ros_sugar.core.event.Event.clear_actions

```{autodoc2-docstring} ros_sugar.core.event.Event.clear_actions
```

````

````{py:method} remove_method(method_name: str)
:canonical: ros_sugar.core.event.Event.remove_method

```{autodoc2-docstring} ros_sugar.core.event.Event.remove_method
```

````

`````
