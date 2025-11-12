# SPDX-FileCopyrightText: Copyright (c) 2022-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from typing import Optional

import isaacsim.core.api.tasks as tasks
import numpy as np
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.robot.manipulators.manipulators import SingleManipulator



# Inheriting from the base class Follow Target
class FollowTarget(tasks.FollowTarget):
    def __init__(
        self,
        name: str = "ur10e_follow_target",
        target_prim_path: Optional[str] = None,
        target_name: Optional[str] = None,
        target_position: Optional[np.ndarray] = None,
        target_orientation: Optional[np.ndarray] = None,
        offset: Optional[np.ndarray] = None,
    ) -> None:
        tasks.FollowTarget.__init__(
            self,
            name=name,
            target_prim_path=target_prim_path,
            target_name=target_name,
            target_position=target_position,
            target_orientation=target_orientation,
            offset=offset,
        )
        return

    def set_robot(self) -> SingleManipulator:
        asset_path = (
            "/home/nvidia/Desktop/LOUSD_new/completed_warehouse/warehouse_complete.usd")

        add_reference_to_stage(usd_path=asset_path, prim_path="/World")
        # define the gripper
        gripper = ParallelGripper(
            # We chose the following values while inspecting the articulation
            end_effector_prim_path="/World/robot/robotiq_2f_140/robotiq_arg2f_base_link",
            joint_prim_names=["finger_joint"],
            joint_opened_positions=np.array([0]),
            joint_closed_positions=np.array([40]),
            action_deltas=np.array([-40]),
            use_mimic_joints=True,
        )

        self.robot_position = np.array([-3.98, -4.715, 1.175])
        self.robot_orientation = np.array([1, 0, 0, 0])
        # define the manipulator
        manipulator = SingleManipulator(
            prim_path="/World/robot",
            name="ur10_robot",
            end_effector_prim_path="/World/robot/robotiq_2f_140/robotiq_arg2f_base_link",
            gripper=gripper,
            position=self.robot_position,
            orientation=self.robot_orientation,
        )
        return manipulator
