from enum import Enum
from typing import Union

from attrs import define, field
from rclpy import qos

from . import base_validators
from .base_attrs import BaseAttrs


def _get_enum_value(enm_val):
    """
    Converter to get value from Enum
    """
    if isinstance(enm_val, Enum):
        return enm_val.value
    return enm_val


@define(kw_only=True)
class QoSConfig(BaseAttrs):
    """
    Class for quality of service (QoS) configuration in ROS2
    """

    # Keep last: only store up to N samples, configurable via the queue depth option.
    # Keep all: store all samples, subject to the configured resource limits of the underlying middleware.
    history: int = field(
        converter=_get_enum_value,
        default=qos.HistoryPolicy.KEEP_LAST,
        validator=base_validators.in_([
            qos.HistoryPolicy.KEEP_LAST,
            qos.HistoryPolicy.KEEP_ALL,
        ]),
    )

    # used only honored if the “history” policy was set to “keep last”
    queue_size: int = field(
        default=10, validator=base_validators.in_range(min_value=5, max_value=100)
    )

    # Best effort: attempt to deliver samples, but may lose them if the network is not robust
    # Reliable: guarantee that samples are delivered, may retry multiple times
    reliability: int = field(
        converter=_get_enum_value,
        default=qos.ReliabilityPolicy.RELIABLE,
        validator=base_validators.in_([
            qos.ReliabilityPolicy.BEST_EFFORT,
            qos.ReliabilityPolicy.RELIABLE,
        ]),
    )

    # Transient local: the publisher becomes responsible for persisting samples for “late-joining” subscriptions
    # Volatile: no attempt is made to persist samples
    durability: int = field(
        converter=_get_enum_value,
        default=qos.DurabilityPolicy.VOLATILE,
        validator=base_validators.in_([
            qos.DurabilityPolicy.TRANSIENT_LOCAL,
            qos.DurabilityPolicy.VOLATILE,
            # qos.DurabilityPolicy.BEST_AVAILABLE, # Only available in iron -> TODO: Get values from rclpy
            qos.DurabilityPolicy.UNKNOWN,
            qos.DurabilityPolicy.SYSTEM_DEFAULT,
        ]),
    )

    # TODO: Fix default values
    # the expected maximum amount of time between subsequent messages being published to a topic
    # deadline: Optional[qos.Duration] = field(default=None)

    # the maximum amount of time between the publishing and the reception of a message without the message being considered stale or expired (expired messages are silently dropped and are effectively never received).
    # lifespan: Optional[qos.Duration] = field(default=None)


@define(kw_only=True)
class BaseConfig(BaseAttrs):
    """
    Node General Parameters

    :param loop_rate: Rate (Hz) in which the node executes
    :type loop_rate: float
    :param visualization: To publish additional topics for visualization
    :type visualization: bool
    """

    # Runtime
    loop_rate: float = field(
        default=100.0, validator=base_validators.in_range(min_value=1e-4, max_value=1e9)
    )  # Hz

    # Debugging and publishing visualization topics
    visualization: bool = field(default=False)


class ComponentRunType(Enum):
    """
    Component run type:
    - Timed: Executes main in a timed loop
    - Event: Executes based on a trigger topic/event
    - Server: Executes based on a ROS service request from a client
    - ActionServer: Executes based on a ROS action server request from a client
    """

    TIMED = "Timed"
    EVENT = "Event"
    SERVER = "Server"
    ACTION_SERVER = "ActionServer"

    @classmethod
    def values(cls):
        return [member.value for member in cls]

    def __str__(self) -> str:
        """
        Gets value of enum

        :return: Enum value
        :rtype: str
        """
        return self.value

    def __repr__(self) -> str:
        """
        Gets value of enum

        :return: Enum value
        :rtype: str
        """
        return self.value

    @classmethod
    def to_str(cls, enum_value) -> str:
        """
        Return string value corresponding to enum value if exists

        :param enum_value: _description_
        :type enum_value: ComponentRunType | str
        :raises ValueError: If the enum value is not from this class

        :return: String value
        :rtype: str
        """
        if isinstance(enum_value, ComponentRunType):
            return enum_value.value
        # If the value is already given as a string check if it valid and return it
        elif isinstance(enum_value, str):
            if enum_value in cls.values():
                return enum_value
        raise ValueError(f"{enum_value} is not a valid ComponentRunType value")


def _convert_to_runtype_to_enum(
    value: Union[ComponentRunType, str],
) -> ComponentRunType:
    """
    Converter for ComponentRunType to set the value from strings

    :param value: Runtype value
    :type value: Union[ComponentRunType, str]

    :raises ValueError: If string value is not one of the ComponentRunType Enum values

    :return: Enum value
    :rtype: ComponentRunType
    """
    if isinstance(value, ComponentRunType):
        return value
    if isinstance(value, str):
        for value_enum in ComponentRunType:
            if value_enum.value == value:
                return value_enum
        raise ValueError(f"Unsupported ComponentRunTime value '{value}'")


@define(kw_only=True)
class BaseComponentConfig(BaseConfig):
    """
    Component configuration parameters

    :param use_without_launcher: To use the component without the Launcher. When True it initializes a ROS node on component init
    :type use_without_launcher: bool
    :param layer_id: Component layer ID. Refers to the component 'start' or 'activate' priority. Zero is equivalent to no-priority
    :type layer_id: int
    :param fallback_rate: Rate (Hz) in which the component checks for Fallbacks and executes actions if a failure is detected
    :type fallback_rate: float
    :param run_type: Component run type
    :type run_type: ComponentRunType
    """

    use_without_launcher: bool = field(default=False)

    # NOTE: Layer ID to be added in coming updates
    # layer_id: int = field(
    #     default=0, validator=base_validators.in_range(min_value=0, max_value=1e3)
    # )

    fallback_rate: float = field(
        default=100.0, validator=base_validators.in_range(min_value=1e-9, max_value=1e9)
    )

    run_type: Union[ComponentRunType, str] = field(
        default=ComponentRunType.TIMED, converter=_convert_to_runtype_to_enum
    )
