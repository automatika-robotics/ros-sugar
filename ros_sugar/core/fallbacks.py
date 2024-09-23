"""Fallbacks"""

from typing import List, Optional, Union

from attrs import define, field

from .action import Action


@define
class Fallback:
    """Fallback action and execution tracking"""

    action: Union[List[Action], Action] = field()
    max_retries: Optional[int] = field(default=None)

    action_idx: int = field(default=0, init=False)
    retry_idx: int = field(default=0, init=False)

    def reset_current_idx(self):
        """Reset the current action index to zero"""
        self.action_idx = 0

    def reset_retries(self):
        """Reset the current retries index to zero"""
        self.retry_idx = 0

    def reset(self):
        """Reset the current action and the retries index to zero"""
        self.reset_current_idx()
        self.reset_retries()


class ComponentFallbacks:
    """
    Components Fallbacks contain the set of Actions to be executed when a failure status is detected (See Status class). Fallbacks are defines with an Action or a list of Actions to be executed in order. Each Action in the list is retried until max_retries is reached.

    # Default behavior in a Component:
    The default behavior in a Component is for the component to broadcast the status on any detected failure. By default the component sets 'on_any_fail' Fallback to 'Action(self.broadcast_status)' with 'max_retries=None'.

    Action(s) set for 'on_any_fail' is executed for *any* failure for which no fallback action is defined.

    # Usage in a Component:
    ```python
        from ros_sugar.component import BaseComponent
        from ros_sugar.action import Action

        my_component = BaseComponent(node_name='test_component')

        # Set fallback for component failure to restart the component
        my_component.on_component_fail(fallback=Action(my_component.restart))

        # Change fallback for any failure
        my_component.on_fail(fallback=Action(my_component.restart))

        # First broadcast status, if another failure happens -> restart
        my_component.on_fail(fallback=[Action(my_component.broadcast_status), Action(my_component.restart)])
    ```
    """

    def __init__(
        self,
        on_any_fail: Optional[Fallback] = None,
        on_component_fail: Optional[Fallback] = None,
        on_algorithm_fail: Optional[Fallback] = None,
        on_system_fail: Optional[Fallback] = None,
        on_giveup: Optional[Fallback] = None,
    ) -> None:
        """
        Setup component fallbacks

        :param max_retries: Maximum number of attempts when running a fallback action, defaults to 1
        :type max_retries: int, optional
        :param on_any_fail: Actions to be executed on any failure if a specific failure policy is not defined, defaults to None
        :type on_any_fail: Optional[Union[List[Action], Action]], optional
        :param on_component_fail:  Actions to be executed on component level fail, defaults to None
        :type on_component_fail: Optional[Union[List[Action], Action]], optional
        :param on_algorithm_fail: Actions to be executed on algorithm level fail, defaults to None
        :type on_algorithm_fail: Optional[Union[List[Action], Action]], optional
        :param on_system_fail: Actions to be executed on system level fail, defaults to None
        :type on_system_fail: Optional[Union[List[Action], Action]], optional
        :param on_giveup: Action to be executed when all fallbacks have failed, defaults to None
        :type on_giveup: Optional[Action], optional
        """
        self.on_any_fail = on_any_fail

        self.on_component_fail = on_component_fail
        self.on_algorithm_fail = on_algorithm_fail

        self.on_system_fail = on_system_fail

        self.on_giveup = on_giveup

        # Flag to indicate that all fallbacks failed and no more fallbacks are available
        self.__giveup: bool = False

    @property
    def giveup(self) -> bool:
        """
        Getter of component giveup, True if no more fallbacks are available for a type of failure

        :return: Giveup
        :rtype: bool
        """
        return self.__giveup

    def reset(self) -> None:
        """Reset all fallback execution tracking indices to 0 and the retries tracking indices to 0"""
        self.reset_execution_indices()
        self.reset_retries()

    def reset_execution_indices(self) -> None:
        """Reset all fallback execution tracking indices to 0"""
        # Indices to keep track of the executed fallback

        if self.on_component_fail:
            self.on_component_fail.reset_current_idx()
        if self.on_algorithm_fail:
            self.on_algorithm_fail.reset_current_idx()
        if self.on_system_fail:
            self.on_system_fail.reset_current_idx()

    def reset_retries(self):
        """Reset all fallback retries tracking indices to 0"""
        if self.on_component_fail:
            self.on_component_fail.reset_retries()
        if self.on_algorithm_fail:
            self.on_algorithm_fail.reset_retries()
        if self.on_system_fail:
            self.on_system_fail.reset_retries()

    def _execute_fallback(self, fallback: Optional[Fallback]) -> None:
        """
        Execute a fallback from given fallbacks methods

        :param idx: Index of last executed fallback from the list, sat to '-1' if no fallback was ever executed
        :type idx: int
        :param fallback_list: List of fallback methods
        :type fallback_list: List[Callable]

        :raises ValueError: If the list of fallback methods is None
        """
        if not fallback:
            # try executing the generic fallback
            if self.on_any_fail:
                self.execute_generic_fallback()
                return
            else:
                raise ValueError("No fallback actions are defined for detected failure")

        if not isinstance(fallback.action, List):
            # Only one fallback action is available

            # None max_retries == Never give up
            if not fallback.max_retries:
                fallback.action()
                fallback.retry_idx += 1
                self.__giveup = False
                return

            if fallback.retry_idx < fallback.max_retries:
                fallback.action()
                fallback.retry_idx += 1
                self.__giveup = False
            else:
                self.__giveup = True
            return

        # Fallback with a list of actions cannot have None max_retries as it will remain stuck on first action
        if not fallback.max_retries:
            fallback.max_retries = 1

        if fallback.retry_idx < fallback.max_retries:
            # If all retries are not consumed increase retry index
            fallback.retry_idx += 1
        else:
            # If all retries are consumed -> reset retries and execute the next fallback
            fallback.reset_retries()
            fallback.action_idx += 1

        if fallback.action_idx < len(fallback.action):
            fallback.action[fallback.action_idx]()
            self.__giveup = False
        else:
            self.__giveup = True

    def execute_giveup(self):
        """
        Execute the component giveup method when all fallbacks fail
        """
        self._execute_fallback(self.on_giveup)

    def execute_component_fallback(self) -> bool:
        """
        Execute the next component fallback method for component failure

        :return: Giveup: If no more fallbacks are available to be executed
        :rtype: bool
        """
        self._execute_fallback(self.on_component_fail)
        return self.__giveup

    def execute_algorithm_fallback(self) -> bool:
        """
        Execute the next algorithm fallback method for algorithm failure

        :return: Giveup: If no more fallbacks are available to be executed
        :rtype: bool
        """
        self._execute_fallback(self.on_algorithm_fail)
        return self.__giveup

    def execute_system_fallback(self) -> bool:
        """
        Execute the next system fallback method for system failure

        :return: Giveup: If no more fallbacks are available to be executed
        :rtype: bool
        """
        self._execute_fallback(self.on_system_fail)
        return self.__giveup

    def execute_generic_fallback(self) -> bool:
        """
        Execute the next algorithm fallback method for any failure

        :return: Giveup: If no more fallbacks are available to be executed
        :rtype: bool
        """
        self._execute_fallback(self.on_any_fail)
        return self.__giveup
