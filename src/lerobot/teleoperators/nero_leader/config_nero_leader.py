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

from dataclasses import dataclass

from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("nero_leader")
@dataclass
class NeroLeaderConfig(TeleoperatorConfig):
    """Configuration for a single Nero 7-DOF leader arm in teaching mode.

    In Mode A (leader-follower teaching), the leader arm shares the same USB-CAN
    adapter as the follower arm.  Both use ``channel="0"``.  The leader is put into
    zero-force drag mode via ``set_leader_mode()``; joint angles are read from the
    0x501-0x507 CAN broadcast frames via ``get_leader_joint_angles()``.

    Windows agx_cando channel numbers are strings: "0" for the first adapter.
    """

    # CAN channel index (Windows agx_cando: "0" = first adapter)
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
