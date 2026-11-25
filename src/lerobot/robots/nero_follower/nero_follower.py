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

from lerobot.cameras import DepthCamera, make_cameras_from_configs
from lerobot.lerobot_types import RobotAction, RobotObservation
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..robot import Robot
from .config_nero_follower import NeroFollowerConfig

logger = logging.getLogger(__name__)

NUM_JOINTS = 7
JOINT_NAMES = [f"joint_{i}" for i in range(1, NUM_JOINTS + 1)]


class NeroFollower(Robot):
    """松灵 Nero 7-DOF follower arm connected via USB-CAN.

    Two operating modes are supported:

    **Mode A — leader-follower teaching (single USB-CAN):**
    One NeroFollower and one NeroLeader share the same USB-CAN adapter
    (``channel="0"``).  Firmware synchronises the arms; this class observes
    state and sends optional clamped position targets.

    **Mode B — bimanual PC control (two USB-CAN adapters):**
    Two NeroFollower instances each own a separate CAN adapter.  Use
    ``BiNeroFollower`` to manage both together under a single Robot interface.
    """

    config_class = NeroFollowerConfig
    name = "nero_follower"

    def __init__(self, config: NeroFollowerConfig):
        super().__init__(config)
        self.config = config
        self._arm = None   # created in connect()
        self.cameras = {}  # populated in connect()

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"{name}.pos": float for name in JOINT_NAMES}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        features: dict[str, tuple] = {}
        for cam_key, cam in self.cameras.items():
            if getattr(cam, "use_rgb", True):
                features[cam_key] = (cam.height, cam.width, 3)
            if isinstance(cam, DepthCamera) and cam.use_depth:
                features[f"{cam_key}_depth"] = (cam.height, cam.width, 1)
        return features

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return self._arm is not None and self._arm.is_connected()

    @property
    def is_calibrated(self) -> bool:
        # Nero arms use absolute encoders; no encoder-offset calibration needed.
        return True

    def calibrate(self) -> None:
        """No-op — Nero arms ship with absolute encoders and need no calibration."""
        pass

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        """Open the CAN bus, enable the arm, and connect cameras."""
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
            "NeroFollower connected on CAN channel %s (firmware %s)",
            self.config.channel,
            self.config.firmware_version,
        )

        # Cameras
        self.cameras = make_cameras_from_configs(self.config.cameras)
        for cam in self.cameras.values():
            cam.connect()

        if calibrate and not self.is_calibrated:
            self.calibrate()

    @check_if_not_connected
    def configure(self) -> None:
        """Put the arm into follower / normal-PC-control mode.

        In both Mode A (leader-follower) and Mode B (bimanual PC control) the
        arm should accept ``move_j`` commands, so we always call
        ``set_follower_mode()``.  On firmware ≥ 1.12 the arm boots in follower
        mode already, but calling this again is a safe no-op.
        """
        self._arm.set_follower_mode()
        logger.debug("NeroFollower on channel %s set to follower mode", self.config.channel)

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        """Return joint angles (rad) plus any camera frames."""
        obs: RobotObservation = {}

        # Joint positions
        angles = self._arm.get_joint_angles()  # list[float], radians, length 7
        for i, name in enumerate(JOINT_NAMES):
            obs[f"{name}.pos"] = float(angles[i])

        # Camera frames
        for cam_key, cam in self.cameras.items():
            frame = cam.async_read()
            if isinstance(frame, dict):
                # DepthCamera returns {"color": ndarray, "depth": ndarray}
                obs[cam_key] = frame["color"]
                depth_key = f"{cam_key}_depth"
                if depth_key in self._cameras_ft:
                    obs[depth_key] = frame["depth"]
            else:
                obs[cam_key] = frame

        return obs

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        """Send a joint-position command to the arm.

        ``action`` must contain keys ``joint_1.pos`` … ``joint_7.pos`` in
        radians.  Uses ``move_j`` which applies on-arm trajectory planning and
        is safe for PC-initiated motion.
        """
        joints = [float(action[f"{name}.pos"]) for name in JOINT_NAMES]
        self._arm.move_j(joints)
        return action

    def disconnect(self) -> None:
        """Disable the arm and close the CAN bus and camera connections."""
        if not self.is_connected:
            return

        # On firmware ≥ 1.12 the arm ignores disable() while in leader mode,
        # so explicitly switch to follower mode first.
        try:
            self._arm.set_follower_mode()
        except Exception as exc:
            logger.warning("set_follower_mode() before disconnect raised: %s", exc)

        try:
            self._arm.disable()
        except Exception as exc:
            logger.warning("arm.disable() raised: %s", exc)

        try:
            self._arm.disconnect()
        except Exception as exc:
            logger.warning("arm.disconnect() raised: %s", exc)

        self._arm = None

        for cam_key, cam in self.cameras.items():
            try:
                cam.disconnect()
            except Exception as exc:
                logger.warning("Camera %s disconnect raised: %s", cam_key, exc)

        self.cameras = {}
        logger.info("NeroFollower on channel %s disconnected", self.config.channel)
