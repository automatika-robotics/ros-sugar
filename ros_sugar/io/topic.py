"""ROS Topic Configuration"""

import inspect
from types import ModuleType
from typing import Any, List, Optional, Union, Dict
from attrs import Factory, define, field
from ..config import BaseAttrs, QoSConfig, base_validators

from . import supported_types


def get_all_msg_types(
    msg_types_module: ModuleType = supported_types,
) -> List[type[supported_types.SupportedType]]:
    """
    Gets all message types from supported data_types
    :return: Supported data types
    :rtype: list[type]
    """
    available_types = [
        type_obj
        for _, type_obj in inspect.getmembers(
            msg_types_module, predicate=inspect.isclass
        )
        if issubclass(type_obj, supported_types.SupportedType)
    ]
    return available_types + supported_types._additional_types


def __parse_name_without_class(type_name: str) -> str:
    """
    Parse a string into class name if exists

    :param type_name: Any name
    :type type_name: str
    :return: Class name
    :rtype: str
    """
    if type_name.startswith("<class") and type_name.endswith("'>"):
        end_pos = type_name.find("'>")
        dot_pos = type_name.rfind(".", 0, end_pos)
        if dot_pos != -1:
            return type_name[dot_pos + 1 : end_pos]
        # If it is not nested
        quote_pos = type_name.rfind("'", 0, end_pos)
        if quote_pos != -1:
            return type_name[quote_pos + 1 : end_pos]
    return type_name


def get_msg_type(
    type_name: Union[type[supported_types.SupportedType], str],
    msg_types_module: Optional[ModuleType] = supported_types,
) -> Union[type[supported_types.SupportedType], str]:
    """
    Gets a message type from supported data_types given a string name
    :param type_name: Message name
    :type type_name: str

    :return: Supported data type or None if not found
    :rtype: _type_
    """
    if isinstance(type_name, type):
        if issubclass(type_name, supported_types.SupportedType):
            return type_name
    if isinstance(type_name, str):
        type_name = __parse_name_without_class(type_name)
        available_types = inspect.getmembers(
            msg_types_module, predicate=inspect.isclass
        )
        if supported_types._additional_types:
            extra_types = {
                (a_type.__name__, a_type)
                for a_type in supported_types._additional_types
            }
            # Get new types
            extra_types = extra_types.difference(available_types)
            for name, obj in list(extra_types):
                if name == type_name and issubclass(obj, supported_types.SupportedType):
                    return obj
        for name, obj in available_types:
            if name == type_name and issubclass(obj, supported_types.SupportedType):
                return obj
    return type_name


def _get_msg_types(
    type_names: List[Union[type[supported_types.SupportedType], str]],
) -> List[Union[type[supported_types.SupportedType], str]]:
    """
    Gets a list of message types from supported data_types given a list of string names
    :param type_name: List of message names
    :type type_name: str

    :return: List of supported data types or None if not found
    :rtype: _type_
    """
    output_type_names = []
    for type_name in type_names:
        output = get_msg_type(type_name)
        output_type_names.append(output)
    return output_type_names


def msg_type_validator(*_: Any, value):
    """
    Validates that a string message type corresponds to a supported data type

    :param value: _description_
    :type value: _type_
    :raises ValueError: _description_
    """
    if not get_msg_type(value):
        raise ValueError(
            f"Cannot set message type to not supported data type '{value}'. Check config.get_supported_datatypes for more info"
        )


def _normalize_topic_name(name: str) -> str:
    """Removes the leading slash from the input string if it exists - Used to normalize topic names in ROS

    :return: Topic name without leading '/'
    :rtype: str
    """
    return name[1:] if name.startswith("/") else name


def _make_qos_config(qos_profile: Union[Dict, QoSConfig]) -> QoSConfig:
    if isinstance(qos_profile, QoSConfig):
        return qos_profile
    return QoSConfig(**qos_profile)


@define(kw_only=True)
class Topic(BaseAttrs):
    """
    Class for ROS topic configuration (name, type and QoS)
    """

    name: str = field(converter=_normalize_topic_name)
    msg_type: Union[type[supported_types.SupportedType], str] = field(
        converter=get_msg_type
    )
    qos_profile: Union[Dict, QoSConfig] = field(
        default=Factory(QoSConfig), converter=_make_qos_config
    )
    ros_msg_type: Any = field(default=None, init=False)

    @msg_type.validator
    def _msg_type_validator(self, _, val):
        msg_types = get_all_msg_types()
        if val not in msg_types:
            raise ValueError(
                f"Got value of 'msg_type': {val}, which is not in available datatypes. Topics can only be created with one of the following types: { {msg_t.__name__: msg_t for msg_t in msg_types} }"
            )
        # Set ros type
        self.ros_msg_type = self.msg_type.get_ros_type()


@define(kw_only=True)
class AllowedTopics(BaseAttrs):
    """Configure allowed types to restrict a component Topic"""

    types: List[Union[type[supported_types.SupportedType], str]] = field(
        converter=_get_msg_types
    )
    number_required: int = field(
        default=1, validator=base_validators.in_range(min_value=0, max_value=100)
    )
    # If number_of_optional is -1 -> unlimited number of optional inputs
    number_optional: int = field(
        default=0, validator=base_validators.in_range(min_value=-1, max_value=100)
    )

    @types.validator
    def _types_validator(self, _, vals):
        msg_types = get_all_msg_types()
        if any(v not in msg_types for v in vals):
            raise ValueError(
                f"Got value of 'msg_type': {vals}, which is not in available datatypes. Topics can only be created with one of the following types: { {msg_t.__name__: msg_t for msg_t in msg_types} }"
            )

    def __attrs_post_init__(self):
        """__attrs_post_init__."""
        if self.number_required == 0 and self.number_optional == 0:
            raise ValueError(
                "Logical error - Cannot define an AllowedTopic with zero optional and required streams"
            )
