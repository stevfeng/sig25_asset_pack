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

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import carb
import omni.appwindow  # Contains handle to keyboard
import numpy as np
from controller.ik_solver import KinematicsSolver
from isaacsim.core.api import World
from tasks.follow_target import FollowTarget


global gripper_open
gripper_open = True

def acquire_keyboard_input():
    appwindow = omni.appwindow.get_default_app_window()
    input = carb.input.acquire_input_interface()
    keyboard = appwindow.get_keyboard()
    sub_keyboard = input.subscribe_to_keyboard_events(keyboard, sub_keyboard_event)

def sub_keyboard_event(event, *args, **kwargs):
    if event.type == carb.input.KeyboardEventType.KEY_PRESS:
        if event.input.name == "N":
            global gripper_open
            gripper_open = not gripper_open
            print(f"Gripper open: {gripper_open}")

my_world = World(stage_units_in_meters=1.0, physics_dt=1.0/360.0)
# Initialize the Follow Target task with a target location for the cube to be followed by the end effector
my_task = FollowTarget(name="ur10e_follow_target", target_position=np.array([-2.8, -4.5, 0.8]))
my_world.add_task(my_task)
acquire_keyboard_input()

my_world.reset()
my_world.scene.remove_object("default_ground_plane")
task_params = my_world.get_task("ur10e_follow_target").get_params()
target_name = task_params["target_name"]["value"]
ur10e_name = task_params["robot_name"]["value"]
my_ur10e = my_world.scene.get_object(ur10e_name)

# initialize the ik solver
ik_solver = KinematicsSolver(my_ur10e)
articulation_controller = my_ur10e.get_articulation_controller()

robot_position = my_task.robot_position
robot_orientation = my_task.robot_orientation

# run the simulation
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()

        # Check if the gripper should be opened or closed
        if gripper_open:
            my_ur10e.gripper.open()
        else:
            my_ur10e.gripper.close()

        # Get the current observations
        observations = my_world.get_observations()
        actions, succ = ik_solver.compute_inverse_kinematics(
            target_position=observations[target_name]["position"]-robot_position,
            target_orientation=observations[target_name]["orientation"],
        )
        if succ:
            articulation_controller.apply_action(actions)
        else:
            print("IK did not converge to a solution.  No action is being taken.")
    elif my_world.is_stopped():
        my_world.reset()

simulation_app.close()
