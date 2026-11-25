#!/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import dataclasses
import logging
from functools import cached_property

from lerobot.lerobot_types import RobotAction, RobotObservation
from lerobot.utils.bimanual import BimanualMixin
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..nero_follower import NeroFollower
from ..nero_follower.config_nero_follower import NeroFollowerConfig
from ..robot import Robot
from .config_bi_nero_follower import BiNeroFollowerConfig

logger = logging.getLogger(__name__)


class BiNeroFollower(BimanualMixin, Robot):
    """Two 松灵 Nero 7-DOF follower arms in bimanual PC-control mode (Mode B).

    Each arm has its own USB-CAN adapter; the PC issues independent ``move_j``
    commands to both arms simultaneously.  Observations and actions are keyed
    with ``left_`` / ``right_`` prefixes (e.g. ``left_joint_1.pos``).

    Shared workspace cameras (defined in ``config.cameras``) are opened by the
    left arm and kept **unprefixed** in observations, matching the convention
    used by :class:`~lerobot.robots.bi_so_follower.BiSOFollower`.

    Instantiate via :func:`~lerobot.robots.utils.make_robot_from_config` with
    ``type="bi_nero_follower"``.
    """

    config_class = BiNeroFollowerConfig
    name = "bi_nero_follower"

    def __init__(self, config: BiNeroFollowerConfig):
        super().__init__(config)
        self.config = config

        # Shared/top-level camera keys stay unprefixed in observations.
        self._top_level_cam_keys: set[str] = set(config.cameras)

        # Collision check: shared camera names must not clash with per-arm names.
        _collisions = (self._top_level_cam_keys & set(config.left_arm_config.cameras)) | (
            self._top_level_cam_keys & set(config.right_arm_config.cameras)
        )
        if _collisions:
            raise ValueError(
                f"Top-level camera names collide with per-arm camera names: {sorted(_collisions)}"
            )

        # Merge top-level cameras into the left arm config so they are opened
        # together with the left arm's connection lifecycle.
        left_cfg = dataclasses.replace(
            config.left_arm_config,
            cameras={**config.left_arm_config.cameras, **config.cameras},
        )
        right_cfg = config.right_arm_config  # unchanged

        self.left_arm = NeroFollower(left_cfg)
        self.right_arm = NeroFollower(right_cfg)

        # Placeholder; filled in by connect() once cameras are instantiated.
        self.cameras: dict = {}

    # ------------------------------------------------------------------
    # Feature dicts
    # ------------------------------------------------------------------

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {
            **{f"left_{k}": v for k, v in self.left_arm._motors_ft.items()},
            **{f"right_{k}": v for k, v in self.right_arm._motors_ft.items()},
        }

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        out: dict[str, tuple] = {}
        # Top-level cameras (routed through left arm) stay unprefixed.
        for k, v in self.left_arm._cameras_ft.items():
            out[k if k in self._top_level_cam_keys else f"left_{k}"] = v
        for k, v in self.right_arm._cameras_ft.items():
            out[f"right_{k}"] = v
        return out

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    # ------------------------------------------------------------------
    # Lifecycle (connect/disconnect add camera bookkeeping on top of Mixin)
    # ------------------------------------------------------------------

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        """Connect both arms (and their cameras) in sequence."""
        # BimanualMixin.connect delegates to left_arm.connect / right_arm.connect.
        super().connect(calibrate=calibrate)
        # After connection, cameras are live — build the merged dict.
        self.cameras = {**self.left_arm.cameras, **self.right_arm.cameras}

    # BimanualMixin already provides disconnect(); nothing extra needed here.

    # ------------------------------------------------------------------
    # Data routing
    # ------------------------------------------------------------------

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        obs: RobotObservation = {}

        for key, value in self.left_arm.get_observation().items():
            # Top-level cameras stay unprefixed; arm joints and arm cameras get "left_".
            obs[key if key in self._top_level_cam_keys else f"left_{key}"] = value

        for key, value in self.right_arm.get_observation().items():
            obs[f"right_{key}"] = value

        return obs

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        # Strip the "left_" prefix, dispatch to left arm.
        left_action = {
            key.removeprefix("left_"): value
            for key, value in action.items()
            if key.startswith("left_")
        }
        # Strip the "right_" prefix, dispatch to right arm.
        right_action = {
            key.removeprefix("right_"): value
            for key, value in action.items()
            if key.startswith("right_")
        }

        sent_left = self.left_arm.send_action(left_action)
        sent_right = self.right_arm.send_action(right_action)

        # Re-add prefixes so the returned dict mirrors the input schema.
        return {
            **{f"left_{k}": v for k, v in sent_left.items()},
            **{f"right_{k}": v for k, v in sent_right.items()},
        }
