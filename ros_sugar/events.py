"""Available Events"""

from typing import Union, Dict, Optional, List, Any
import json
from copy import deepcopy
from .io.topic import Topic
from .core.event import Event


def json_to_events_list(
    json_obj: Union[str, bytes, bytearray],
    topic_template: Optional[Topic] = None,
) -> List:
    """
    Loads a list of events from a JSON object

    :param json_obj: JSON object containing a set of events
    :type json_obj: str | bytes | bytearray

    :raises ValueError: If the provided json object cannot be converted to an events list

    :return: Events list
    :rtype: List[Event]
    """
    list_obj = json.loads(json_obj)
    events_list = []
    for event_serialized in list_obj:
        # Check if the serialized event contains a class name
        event_as_dict = json.loads(event_serialized)
        if "event_class" not in event_as_dict.keys():
            raise ValueError(
                "Cannot convert json object to Events Dictionary. Json item is not a valid serialized Event"
            )

        # Get and check event class
        event_class_name: str = event_as_dict["event_class"]
        events_classes = [event.__name__ for event in available_events]
        if event_class_name not in events_classes:
            raise ValueError(
                f"Cannot convert json object to Events Dictionary. Unknown event class '{event_class_name}'"
            )

        for event in available_events:
            if event.__name__ == event_class_name:
                # Construct new event
                new_event = event(
                    event_as_dict["event_name"],
                    event_as_dict,
                    event_as_dict["trigger_ref_value"],
                    nested_attributes=[],
                    topic_template=topic_template,
                )
                # Add to events dictionary
                events_list.append(deepcopy(new_event))

    return events_list


class OnAny(Event):
    def __init__(self, event_name: str, event_source: Union[Topic, str, Dict]) -> None:
        """__init__.

        :param event_name:
        :type event_name: str
        :param event_source:
        :type event_source: Union[Topic, str, Dict]
        :param attrs: Tuple of attributes to access in the topic message
        :rtype: None
        """
        # passing trigger_value as zero as it will not be used in this event
        super().__init__(event_name, event_source, None, [])

    def callback(self, msg: Any) -> None:
        """
        Event topic listener callback

        :param msg: Event trigger topic message
        :type msg: Any
        """
        self._event_value = msg

        self.trigger = True

        # Process event on any incoming message
        if not self.under_processing:
            self.under_processing = True
            self._call_on_trigger(msg=msg)
            self.under_processing = False


class OnChange(Event):
    """
    OnChange Event is triggered when a given topic attribute changes in value from any initial value to any new value. The target attribute value is registered on the first recept of a message on the target topic, then the event is triggered on a change in that value. After a change the new value is registered and the event is triggered again on any new change, ...etc.

    ## Example usage scenario:
    - Event on a change in the number of detected people of the robot by a vision system to play a friendly welcome message.
    """

    def __init__(
        self,
        event_name: str,
        event_source: Union[Topic, str, Dict],
        nested_attributes: Union[str, List[str]],
        **kwargs,
    ) -> None:
        """__init__.

        :param event_name:
        :type event_name: str
        :param event_source:
        :type event_source: Union[Topic, str, Dict]
        :param attrs: Tuple of attributes to access in the topic message
        :rtype: None
        """
        # passing trigger_value as zero as it will not be used in this event
        super().__init__(event_name, event_source, None, nested_attributes, **kwargs)

    def callback(self, msg) -> None:
        """
        Overrides Event callback to save previous event value for OnChange comparison

        :param msg: Event topic message
        :type msg: Any
        """
        if hasattr(self, "_event_value"):
            self._previous_event_value = self._event_value.value
        return super().callback(msg)

    def _update_trigger(self) -> None:
        """
        Set trigger  to True if event value is equal to reference value
        """
        if self._event_value != self._previous_event_value:
            self.trigger = True
        else:
            self.trigger = False


class OnChangeEqual(Event):
    """
    OnChangeEqual Event is a combination of OnChange and OnEqual events. OnChangeEqual is triggered when a given topic attribute changes in value from any initial value to <b>given trigger goal</b> value.

    :::{note} The difference between using OnChangeEqual as opposite to OnEqual or OnChange is that:
    - OnEqual will keep getting triggered every time a new message value is received that is equal to the trigger.
    - OnChange will keep getting triggered every time a new message value is received that is different from a previous value
    - OnChangeEqual will get triggered once when the topic message value reaches the trigger, making it convenient for many applications
    :::

    ## Some example usage scenarios:
    - Event on the robot reaching a navigation goal point: reach_end Boolean topic OnChangeEqual to True (triggered once when reaching, does not trigger again if the robot is static and staying in 'goal reaching' state)
    - Event on an Enum value of a message attribute to detect reaching a given state.
    - Event of reaching 100% charge level of a robot to end charging.
    """

    def __init__(
        self,
        event_name: str,
        event_source: Union[Topic, str, Dict],
        trigger_value: Union[float, int, bool, str, list],
        nested_attributes: Union[str, List[str]],
        **kwargs,
    ) -> None:
        """__init__.

        :param event_name:
        :type event_name: str
        :param event_source:
        :type event_source: Union[Topic, str, Dict]
        :param trigger_value:
        :type trigger_value: Union[float, int, bool, str, list]
        :param attrs:
        :rtype: None
        """
        # passing trigger_value as zero as it will not be used in this event
        super().__init__(
            event_name, event_source, trigger_value, nested_attributes, **kwargs
        )

    def callback(self, msg) -> None:
        """
        Overrides Event callback to save previous event value for OnChange comparison

        :param msg: Event topic message
        :type msg: Any
        """
        if hasattr(self, "_event_value"):
            self._previous_event_value = self._event_value.value
        return super().callback(msg)

    def _update_trigger(self) -> None:
        """
        Set trigger  to True if event value is equal to reference value
        """
        if hasattr(self, "_previous_event_value"):
            if (
                self._event_value != self._previous_event_value
                and self._event_value == self.trigger_ref_value
            ):
                self.trigger = True
            else:
                self.trigger = False


class OnEqual(Event):
    """
    OnEqual Event is triggered when a given topic attribute value is equal to a given trigger value.

    ## Example usage scenario:
    - Event when the detection id (object type) in an object detection topic is equal to a specific object (to raise an event on detecting another robot, a human, etc.)
    """

    def __init__(
        self,
        event_name: str,
        event_source: Union[Topic, str, Dict],
        trigger_value: Union[float, int, bool, str, list],
        nested_attributes: Union[str, List[str]],
        **kwargs,
    ) -> None:
        """__init__.

        :param event_name:
        :type event_name: str
        :param event_source:
        :type event_source: Union[Topic, str, Dict]
        :param trigger_value:
        :type trigger_value: Union[float, int, bool, str, list]
        :param attrs:
        :rtype: None
        """
        super().__init__(
            event_name, event_source, trigger_value, nested_attributes, **kwargs
        )

    def _update_trigger(self) -> None:
        """
        Set trigger  to True if event value is equal to reference value
        """
        self.trigger = self._event_value == self.trigger_ref_value


class OnContainsAll(Event):
    """
    OnContainsAll Event is triggered when a given topic attribute value contains all of the given trigger list value.
    """

    def __init__(
        self,
        event_name: str,
        event_source: Union[Topic, str, Dict],
        trigger_value: list,
        nested_attributes: Union[str, List[str]],
        **kwargs,
    ) -> None:
        """__init__.

        :param event_name:
        :type event_name: str
        :param event_source:
        :type event_source: Union[Topic, str, Dict]
        :param trigger_value:
        :type trigger_value: Union[float, int, bool, str, list]
        :param attrs:
        :rtype: None
        """
        super().__init__(
            event_name, event_source, trigger_value, nested_attributes, **kwargs
        )

    def _update_trigger(self) -> None:
        """
        Set trigger  to True if event value contains all of the reference values
        """
        self.trigger = self.trigger_ref_value in self._event_value


class OnContainsAny(Event):
    """
    OnContainsAny Event is triggered when a given topic attribute value contains one of the given trigger list value.
    """

    def __init__(
        self,
        event_name: str,
        event_source: Union[Topic, str, Dict],
        trigger_value: list,
        nested_attributes: Union[str, List[str]],
        **kwargs,
    ) -> None:
        """__init__.

        :param event_name:
        :type event_name: str
        :param event_source:
        :type event_source: Union[Topic, str, Dict]
        :param trigger_value:
        :type trigger_value: Union[float, int, bool, str, list]
        :param attrs:
        :rtype: None
        """
        super().__init__(
            event_name, event_source, trigger_value, nested_attributes, **kwargs
        )

    def _update_trigger(self) -> None:
        """
        Set trigger  to True if event value contains any of the reference values
        """
        if isinstance(self.trigger_ref_value, list):
            self.trigger = any(
                val in self._event_value for val in self.trigger_ref_value
            )
        else:
            self.trigger = self.trigger_ref_value in self._event_value


class OnDifferent(Event):
    """
    OnDifferent Event is triggered when a given topic attribute value is different from a given trigger value.
    """

    def __init__(
        self,
        event_name: str,
        event_source: Union[Topic, str, Dict],
        trigger_value: Union[float, int, bool, str, list],
        nested_attributes: Union[str, List[str]],
        **kwargs,
    ) -> None:
        """__init__.

        :param event_name:
        :type event_name: str
        :param event_source:
        :type event_source: Union[Topic, str, Dict]
        :param trigger_value:
        :type trigger_value: Union[float, int, bool, str, list]
        :param attrs:
        :rtype: None
        """
        super().__init__(
            event_name, event_source, trigger_value, nested_attributes, **kwargs
        )

    def _update_trigger(self) -> None:
        """
        Set trigger  to True if event value is not equal to reference value
        """
        self.trigger = self._event_value != self.trigger_ref_value


class OnGreater(Event):
    """
    OnGreater Event is triggered when a given topic attribute value is greater than a given trigger value.

    ## Example usage scenario:
    - Event when a drone is higher than a certain allowed elevation (location z coordinate > elevation level), to bring the drone down into allowed limits.
    """

    def __init__(
        self,
        event_name: str,
        event_source: Union[Topic, str, Dict],
        trigger_value: Union[float, int, bool, str],
        nested_attributes: Union[str, List[str]],
        or_equal: bool = False,
        **kwargs,
    ) -> None:
        """__init__.

        :param event_name:
        :type event_name: str
        :param event_source:
        :type event_source: Union[Topic, str, Dict]
        :param trigger_value:
        :type trigger_value: Union[float, int, bool, str]
        :param attrs:
        :param or_equal:
        :type or_equal: bool
        :rtype: None
        """
        super().__init__(
            event_name, event_source, trigger_value, nested_attributes, **kwargs
        )
        self._or_equal = or_equal

    def _update_trigger(self) -> None:
        """
        Set trigger  to True if event value is greater than reference value
        """
        if self._or_equal:
            self.trigger = self._event_value >= self.trigger_ref_value
        else:
            self.trigger = self._event_value > self.trigger_ref_value


class OnLess(Event):
    """
    OnGreater Event is triggered when a given topic attribute value is less than a given trigger value.

    ## Example usage scenario:
    - Event when the robot battery level falls under a certain low limit, to go back to the charging station, for example.
    """

    def __init__(
        self,
        event_name: str,
        event_source: Union[Topic, str, Dict],
        trigger_value: Union[float, int, bool, str],
        nested_attributes: Union[str, List[str]],
        or_equal: bool = False,
        **kwargs,
    ) -> None:
        """__init__.

        :param event_name:
        :type event_name: str
        :param event_source:
        :type event_source: Union[Topic, str, Dict]
        :param trigger_value:
        :type trigger_value: Union[float, int, bool, str]
        :param attrs:
        :param or_equal:
        :type or_equal: bool
        :rtype: None
        """
        super().__init__(
            event_name, event_source, trigger_value, nested_attributes, **kwargs
        )
        self._or_equal = or_equal

    def _update_trigger(self) -> None:
        """
        Set trigger  to True if event value is less than reference value
        """
        if self._or_equal:
            self.trigger = self._event_value <= self.trigger_ref_value
        else:
            self.trigger = self._event_value < self.trigger_ref_value


available_events: List[type] = [
    OnAny,
    OnChange,
    OnLess,
    OnGreater,
    OnChangeEqual,
    OnDifferent,
    OnEqual,
    OnContainsAll,
    OnContainsAny,
]
