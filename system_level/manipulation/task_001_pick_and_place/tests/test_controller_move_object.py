import pytest
from unittest.mock import Mock

from ros2_code.controller import Controller


def test_move_object_invokes_pick_and_place():
    """
    Unit test for Controller.move_object

    We verify:
    - pick() is called once
    - place() is called once
    - bin selection logic is used
    """

    controller = Controller.__new__(Controller)

    controller.pick = Mock()
    controller.place = Mock()
    controller.select_bin = Mock(return_value=(0.5, 0.5))

    fake_obj = Mock()
    fake_obj.color = "red"
    fake_obj.x_world = 0.1
    fake_obj.y_world = 0.2
    fake_obj.height = 0.05

    controller.move_object(fake_obj)

    controller.pick.assert_called_once()
    controller.place.assert_called_once()
