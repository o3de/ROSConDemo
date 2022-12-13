#!/usr/bin/env python3

#
# Copyright (c) Contributors to the Open 3D Engine Project.
# For complete copyright and license terms please see the LICENSE at the root of this distribution.
#
# SPDX-License-Identifier: Apache-2.0 OR MIT
#
#

import rclpy
import time
from threading import Thread

from geometry_msgs.msg import PoseStamped

from .state_machine import GlobalOrchestrationSM
from .orchestration_node import KrakenOrchestrationNode


class GlobalOrchestration(GlobalOrchestrationSM, KrakenOrchestrationNode):
    def __init__(self):
        rclpy.init()

        GlobalOrchestrationSM.__init__(self)
        KrakenOrchestrationNode.__init__(self)
        self.logger = KrakenOrchestrationNode.get_logger(self)

        spin_thread = Thread(target=rclpy.spin, args=(self,))
        spin_thread.start()

        self.poses = []
        self.current_index = 0

    def get_next_pose(self):
        """Retrieve next goal from the Plan."""
        if self.current_index >= len(self.poses):
            self.logger.error('Invalid index!')
            return PoseStamped()

        pose = self.poses[self.current_index]
        self.logger.info("Current goal id is %d " % self.current_index)
        self.current_index += 1
        return pose

    def __del__(self):
        self.destroy_node()
        rclpy.shutdown()

    def on_enter_waiting(self):
        """Is called on entry to waiting state."""
        self.logger.info('Has the simulation started?')
        self.send_state("Wait for connection")
        # Perform a check whether the simulation has started or not.
        self.started()

    def on_enter_spawning(self):
        """Is called on entry to spawning state and spawns robot."""
        self.logger.info('Spawning the robot.')
        self.send_state("Spawning the robot")
        if self.spawn_kraken():
            self.spawned()
        else:
            self.logger.info('Robot spawning failed.')

    def on_spawned(self):
        """Is called on spawn, retrieves plan and position."""
        self.logger.info('Robot spawned successfully. Fetching info.')
        pose = self.get_spawn_point_pose()
        self.poses = self.get_plan_poses(pose)
        # Give nav stack a few seconds before we start first goal point
        self.logger.info('Waiting for robot sensors to latch...')
        time.sleep(3)

    def on_enter_gathering(self):
        """Is called on entry to gathering state."""
        self.logger.info('Is the robot in a gathering position?')
        # Perform a check whether the robot is in a gathering position or not.
        if self.can_gather:
            self.gather()
        else:
            self.navigate()

    def on_enter_finishing(self):
        """Is called on entry to finishing apple gathering state."""
        self.logger.info('Has the robot completed its job?')
        # Perform a check whether the robot has finished its job or not.
        if self.current_index >= len(self.poses):
            self.finished()
        else:
            self.finish()

    def on_navigate(self):
        """Is called on nav. state, sends goal position to nav2, monitors."""
        self.send_state("Navigating")
        self.logger.info('Navigating the robot to a closest gathering position?')
        self.can_gather = False
        # Instruct the robot to navigate to the closest gathering position.
        self.navigate_to_pose(self.get_next_pose())

        while not self.goal_pose_reached:
            time.sleep(1)

        self.goal_pose_reached = False
        self.can_gather = True

    def on_gather(self):
        """Is called on gathering state and monitor progress."""
        self.logger.info("Requesting gathering...")
        self.send_state("Gather %.1f " % self.apple_progress)
        trigger_res = self.trigger_apple_gathering()
        self.logger.info(trigger_res.message)

        if trigger_res.success:
            while not self.gathering_done:
                self.send_state("Gather %.0f %%" % self.apple_progress)
                time.sleep(0.2)

            self.logger.info("Done gathering apples.")
        else:
            self.logger.info("Freeing up the AppleKraken...")
            cancel_res = self.cancel_apple_gathering()
            self.logger.info(cancel_res.message)
            time.sleep(2)

    def on_finish(self):
        """Is called on finish state, navigates to last goal."""
        # Same action is performed.
        self.on_navigate()

    def on_finished(self):
        """Is called after finished."""
        self.send_state("Completed")
        self.logger.info('The robot has completed its job.')
        # The robot has finished its job.


def main():
    global_orch = GlobalOrchestration()
    global_orch.run()


if __name__ == '__main__':
    main()
