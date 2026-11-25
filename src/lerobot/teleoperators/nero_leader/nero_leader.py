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

import logging
from functools import cached_property

from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..teleoperator import Teleoperator
from .config_nero_leader import NeroLeaderConfig

logger = logging.getLogger(__name__)

NUM_JOINTS = 7
JOINT_NAMES = [f"joint_{i}" for i in range(1, NUM_JOINTS + 1)]


class NeroLeader(Teleoperator):
    """松灵 Nero 7-DOF leader arm in zero-force drag (teaching) mode.

    Used exclusively in **Mode A** (leader-follower teaching on a single USB-CAN
    adapter).  The arm is set to leader mode via ``set_leader_mode()``, which
    enables zero-force back-driving so a human operator can physically drag it.
    Joint angles are read from the 0x501-0x507 CAN broadcast frames via
    ``get_leader_joint_angles()``.

    ``send_feedback()`` is a no-op because the Nero leader arm has no built-in
    force-feedback actuators; haptic feedback is handled entirely by the follower
    arm's firmware in Mode A.
    """

    config_class = NeroLeaderConfig
    name = "nero_leader"

    def __init__(self, config: NeroLeaderConfig):
        super().__init__(config)
        self.config = config
        self._arm = None  # created in connect()

    @cached_property
    def action_features(self) -> dict[str, type]:
        """Joint angles (radians) reported by the leader arm."""
        return {f"{name}.pos": float for name in JOINT_NAMES}

    @cached_property
    def feedback_features(self) -> dict[str, type]:
        """No feedback channels — the Nero leader has no actuated feedback."""
        return {}

    @property
    def is_connected(self) -> bool:
        return self._arm is not None and self._arm.is_connected()

    @property
    def is_calibrated(self) -> bool:
        # Nero arms use absolute encoders; no calibration required.
        return True

    def calibrate(self) -> None:
        """No-op — Nero arms ship with absolute encoders."""
        pass

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        """Open the CAN bus and put the arm into zero-force leader mode."""
        from pyAgxArm import ArmModel, AgxArmFactory, create_agx_arm_config

        arm_cfg = create_agx_arm_config(
            robot=ArmModel.NERO,
            firmeware_version=self.config.firmware_version,  # NOTE: SDK typo — do not change to "firmware_version"
            channel=self.config.channel,
            interface=self.config.interface,
        )
        self._arm = AgxArmFactory.create_arm(arm_cfg)
        self._arm.connect()
        self._arm.enable()
        logger.info(
            "NeroLeader connected on CAN channel %s (firmware %s)",
            self.config.channel,
            self.config.firmware_version,
        )

    @check_if_not_connected
    def configure(self) -> None:
        """Enable zero-force drag mode on the leader arm."""
        self._arm.set_leader_mode()
        logger.debug("NeroLeader on channel %s set to leader mode", self.config.channel)

    @check_if_not_connected
    def get_action(self) -> dict[str, float]:
        """Read current joint angles from the leader arm's CAN broadcast.

        Returns angles in radians keyed as ``joint_1.pos`` … ``joint_7.pos``.
        """
        angles = self._arm.get_leader_joint_angles()  # list[float], radians, length 7
        return {f"{name}.pos": float(angles[i]) for i, name in enumerate(JOINT_NAMES)}

    def send_feedback(self, feedback: dict) -> None:
        """No-op — the Nero leader arm has no actuated feedback channel."""
        pass

    def disconnect(self) -> None:
        """Exit leader mode, disable, and close the CAN bus."""
        if not self.is_connected:
            return

        # Switch back to follower mode before disabling.
        # On firmware >= 1.12, disable() is ignored while in leader mode.
        try:
            self._arm.set_follower_mode()
        except Exception as exc:
            logger.warning("set_follower_mode() before leader disconnect raised: %s", exc)

        try:
            self._arm.disable()
        except Exception as exc:
            logger.warning("arm.disable() raised: %s", exc)

        try:
            self._arm.disconnect()
        except Exception as exc:
            logger.warning("arm.disconnect() raised: %s", exc)

        self._arm = None
        logger.info("NeroLeader on channel %s disconnected", self.config.channel)
