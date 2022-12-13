#!/usr/bin/env python3

#
# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#
#

from statemachine import StateMachine, State


class GlobalOrchestrationSM(StateMachine):
    """State machine implementation, contains list of states and valid transitions."""

    can_gather = False
    has_finished = True

    start = State('Start', initial=True)
    waiting = State('Waiting')  # Waiting for the simulation start
    spawning = State('Spawning')  # Spawning the robot
    gathering = State('Gathering')
    finishing = State('Finishing')
    # This state may be removed if the simulation is restarted upon completion.
    terminating = State('Terminating')

    run = start.to(waiting)
    await_start = waiting.to(waiting)
    started = waiting.to(spawning)
    spawned = spawning.to(gathering)
    gather = gathering.to(finishing)
    navigate = gathering.to(gathering)
    finish = finishing.to(gathering)
    finished = finishing.to(terminating)

    def __init__(self) -> None:
        super().__init__()
