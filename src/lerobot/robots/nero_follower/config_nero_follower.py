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


@RobotConfig.register_subclass("nero_follower")
@dataclass
class NeroFollowerConfig(RobotConfig):
    """Configuration for a single Nero 7-DOF follower arm connected via USB-CAN.

    Windows agx_cando channel numbers are strings: "0" for the first adapter, "1"
    for the second, and so on.  On a single USB-CAN shared between two arms (Mode A,
    leader-follower teaching), both the NeroFollower and NeroLeader use ``channel="0"``.
    In bimanual PC-control mode (Mode B), each arm uses its own channel.
    """

    # CAN channel index (Windows agx_cando: "0" = first adapter, "1" = second, …)
    channel: str = "0"
    # SDK driver profile for the arm mainboard firmware.  Pass the string directly
    # to create_agx_arm_config(firmeware_version=…).  Valid values correspond to the
    # NeroFW enum: "default" (≤1.10), "1.11", "1.12", "1.20", "1.21" (≥1.21).
    # Use "default" until you can confirm the actual profile with robot.get_firmware().
    # NOTE: This is NOT the joint-node firmware shown in the upper-computer UI (e.g.
    # "V2.0.7"); that is per-node firmware and is unrelated to this parameter.
    firmware_version: str = "default"
    # python-can interface name.  "agx_cando" is the Windows USB-CAN default.
    interface: str = "agx_cando"
    # Optional cameras mounted on or near this arm.
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
