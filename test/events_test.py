import unittest
from functools import partial
from threading import Event, Thread
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import numpy as np

from ros_sugar import events
from ros_sugar.io import Topic
from ros_sugar.core import BaseComponent
from ros_sugar import Launcher
from automatika_ros_sugar.msg import ComponentStatus
from ros_sugar.actions import Action, LogInfo

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from ros_sugar.io.utils import numpy_to_multiarray

# Threading Events
on_any_py_event = Event()
on_equal_py_event = Event()
on_diff_py_event = Event()
on_greater_py_event = Event()
on_less_py_event = Event()
on_contains_any_py_event = Event()
on_contains_all_py_event = Event()
on_change_py_event = Event()
on_change_eq_py_event = Event()


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """
    Generate launch test description to test all event classes using the component health status topic
    Health status is published automatically after start and should be equal to 'STATUS_HEALTHY'
    """

    # health status topic
    status_topic = Topic(name="test_component_status", msg_type="ComponentStatus")

    # float array topic
    float_array_topic = Topic(name="float_array", msg_type="Float64MultiArray")

    # Component publishing to the event topic
    component = BaseComponent(
        component_name="test_component"
    )

    event_on_any = events.OnAny(
        event_name="on_any",
        event_source=status_topic,
    )

    event_on_equal = events.OnEqual(
        event_name="on_equal",
        event_source=status_topic,
        trigger_value=ComponentStatus.STATUS_HEALTHY,
        nested_attributes="status",
    )

    event_on_diff = events.OnDifferent(
        event_name="on_different",
        event_source=status_topic,
        trigger_value=ComponentStatus.STATUS_FAILURE_ALGORITHM_LEVEL,
        nested_attributes="status",
    )

    event_on_greater = events.OnGreater(
        event_name="on_greater",
        event_source=status_topic,
        trigger_value=-1,
        nested_attributes="status",
    )

    event_on_less = events.OnLess(
        event_name="on_less",
        event_source=status_topic,
        trigger_value=ComponentStatus.STATUS_FAILURE_COMPONENT_LEVEL,
        nested_attributes="status",
    )

    event_on_contains_any = events.OnContainsAny(
        event_name="on_contains_any",
        event_source=float_array_topic,
        trigger_value=[
            1.0, 10.0
        ],
        nested_attributes="data",
    )

    event_on_contains_any = events.OnContainsAny(
        event_name="on_contains_any",
        event_source=float_array_topic,
        trigger_value=[
            1.0, 10.0
        ],
        nested_attributes="data",
    )

    event_on_contains_all = events.OnContainsAll(
        event_name="on_contains_all",
        event_source=float_array_topic,
        trigger_value=[1.0, 2.0, 3.0],
        nested_attributes="data",
    )

    event_on_change = events.OnChange(
        event_name="on_change",
        event_source=float_array_topic,
        nested_attributes="data",
    )

    event_on_change_eq = events.OnChangeEqual(
        event_name="on_change_eq",
        event_source=float_array_topic,
        nested_attributes="data",
        trigger_value=[30.0, 40.0]
    )

    def trigger_event(on_event: Event):
        on_event.set()

    launcher = Launcher()

    launcher.add_pkg(
        components=[component],
        events_actions={
            event_on_any: [
                LogInfo(msg="Got OnAny Event"),
                Action(method=partial(trigger_event, on_any_py_event)),
            ],
            event_on_equal: [
                LogInfo(msg="Got OnEqual Event"),
                Action(method=partial(trigger_event, on_equal_py_event)),
            ],
            event_on_diff: [
                LogInfo(msg="Got OnDifferent Event"),
                Action(method=partial(trigger_event, on_diff_py_event)),
            ],
            event_on_less: [
                LogInfo(msg="Got OnLess Event"),
                Action(method=partial(trigger_event, on_less_py_event)),
            ],
            event_on_greater: [
                LogInfo(msg="Got OnGreater Event"),
                Action(method=partial(trigger_event, on_greater_py_event)),
            ],
            event_on_contains_any: [
                LogInfo(msg="Got OnContainsAny Event"),
                Action(method=partial(trigger_event, on_contains_any_py_event)),
            ],
            event_on_contains_all: [
                LogInfo(msg="Got OnContainsAll Event"),
                Action(method=partial(trigger_event, on_contains_all_py_event)),
            ],
            event_on_change: [
                LogInfo(msg="Got OnChange Event"),
                Action(method=partial(trigger_event, on_change_py_event)),
            ],
            event_on_change_eq: [
                LogInfo(msg="Got OnChangeEqual Event"),
                Action(method=partial(trigger_event, on_change_eq_py_event)),
            ],
        },
    )

    # Setup launch description without bringup for testing
    launcher.setup_launch_description()

    # Add ready for test action
    launcher._description.add_action(launch_testing.actions.ReadyToTest())

    # Return the launcher description for launch_testing
    return launcher._description


class TestingNode(Node):
    def __init__(self, name="publisher_test_node"):
        super().__init__(name)
        self._data = np.array([1.0, 2.0, 3.0, 4.0])

    def start_node(self, change_data: bool):
        # Create a publisher
        self.publisher = self.create_publisher(Float64MultiArray, "float_array", 10)
        if not change_data:
            self.timer = self.create_timer(0.1, self.publish_data)
        else:
            self._counter = 0
            self.timer = self.create_timer(0.1, self.publish_changed_data)

        # Add a spin thread
        self.ros_spin_thread = Thread(
            target=lambda node: rclpy.spin(node), args=(self,)
        )
        self.ros_spin_thread.start()

    def publish_data(self):
        self.publisher.publish(numpy_to_multiarray(self._data, Float64MultiArray))

    def publish_changed_data(self):
        self._counter += 1
        if self._counter % 2 == 0:
            self._data = np.array([30.0, 40.0])
        else:
            self._data = np.array([1.0, 2.0, 3.0, 4.0])
        self.publisher.publish(numpy_to_multiarray(self._data, Float64MultiArray))


class TestEvents(unittest.TestCase):
    """Tests will run concurrently with the dut process.  After all these tests are done,
       launch system will shut down the processes that it started up
    """
    wait_time = 10.0    # seconds

    def test_on_any(cls):
        global on_any_py_event
        assert on_any_py_event.wait(cls.wait_time)

    def test_on_equal(cls):
        global on_equal_py_event
        assert on_equal_py_event.wait(cls.wait_time)

    def test_on_different(cls):
        global on_diff_py_event
        assert on_diff_py_event.wait(cls.wait_time)

    def test_on_less(cls):
        global on_less_py_event
        assert on_less_py_event.wait(cls.wait_time)

    def test_on_greater(cls):
        global on_greater_py_event
        assert on_greater_py_event.wait(cls.wait_time)

    def test_contains_with_publisher_node(cls):
        global on_contains_any_py_event, on_contains_all_py_event
        node = TestingNode()
        node.start_node(change_data=False)
        event_any = on_contains_any_py_event.wait(cls.wait_time)
        event_all = on_contains_all_py_event.wait(cls.wait_time)
        node.destroy_node()
        assert event_any
        assert event_all

    def test_change_with_publisher_node(cls):
        global on_change_py_event, on_change_eq_py_event
        node = TestingNode()
        node.start_node(change_data=True)
        event_change = on_change_py_event.wait(cls.wait_time)
        event_change_eq = on_change_eq_py_event.wait(cls.wait_time)
        node.destroy_node()
        assert event_change
        assert event_change_eq
