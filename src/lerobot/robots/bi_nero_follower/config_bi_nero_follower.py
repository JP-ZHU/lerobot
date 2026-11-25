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

from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig

from ..config import RobotConfig
from ..nero_follower.config_nero_follower import NeroFollowerConfig


@RobotConfig.register_subclass("bi_nero_follower")
@dataclass
class BiNeroFollowerConfig(RobotConfig):
    """Configuration for two Nero 7-DOF follower arms in bimanual PC-control mode.

    In Mode B (bimanual PC control) each arm has its own USB-CAN adapter.
    ``left_arm_config.channel`` and ``right_arm_config.channel`` must be
    different strings (e.g. ``"0"`` and ``"1"``).

    ``cameras`` holds any shared viewpoint cameras (e.g. a workspace overview
    camera) that are not tied to a specific arm.  Per-arm cameras are placed
    inside each arm's config.
    """

    left_arm_config: NeroFollowerConfig = field(
        default_factory=lambda: NeroFollowerConfig(channel="0")
    )
    right_arm_config: NeroFollowerConfig = field(
        default_factory=lambda: NeroFollowerConfig(channel="1")
    )
    # Shared / top-level cameras (workspace overview, etc.)
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
