# ****************************************************************************
#
# Copyright (c) 2014-2024 Fraunhofer FKIE
# Author: Alexander Tiderko
# License: MIT
#
# ****************************************************************************
#
# This is a top-level compatibility shim that re-exports everything from
# the actual module inside the package.
#

from task_005_KIE_multiagent_model.interface_finder import *  # noqa: F401,F403
from task_005_KIE_multiagent_model.interface_finder import _get_topic  # noqa: F401
from task_005_KIE_multiagent_model.interface_finder import _get_topic_from_node  # noqa: F401
from task_005_KIE_multiagent_model.interface_finder import _get_service  # noqa: F401