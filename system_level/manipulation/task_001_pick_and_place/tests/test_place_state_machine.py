import pytest
from unittest.mock import Mock

from ros2_code.place_state_machine_on_enter import PickAndPlaceStateMachine


def test_on_enter_triggers_pick_and_place_and_transition():
    """
    Unit test for on_enter TODO

    We verify:
    - controller.move_object() is called
    - state transition is triggered
    """

    sm = PickAndPlaceStateMachine.__new__(PickAndPlaceStateMachine)

    sm.controller = Mock()
    sm.controller.select_random_object.return_value = Mock()

    sm.send = Mock()

    sm.on_enter_picking_and_placing()

    sm.controller.move_object.assert_called_once()
    sm.send.assert_called_once()
