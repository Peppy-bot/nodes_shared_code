from __future__ import annotations

import logging
from typing import Optional

logger = logging.getLogger(__name__)

_DEFAULT_TRANSLATION: list[float] = [0.0, 0.0, 0.0]
_DEFAULT_ORIENTATION: list[float] = [1.0, 0.0, 0.0, 0.0]  # wxyz identity
_DEFAULT_FREQUENCY: float = 0.0  # 0 = match physics rate


class IsaacImuSensor:
    """Reads IMU data from an Isaac Sim IMU sensor prim.

    Uses isaacsim.sensor.imu.IMUSensor which reads orientation, angular
    velocity, and linear acceleration directly from the PhysX physics engine
    — no finite-difference approximation needed.

    The prim_path must point to an existing IMU sensor prim in the USD stage
    (typically placed as a child of the rigid body you want to measure).

    translation — sensor offset from parent body frame (xyz, metres).
    orientation — sensor mount orientation relative to parent body (wxyz quaternion).
    frequency   — sampling rate in Hz; 0 matches the physics step rate.
    """

    def __init__(
        self,
        prim_path: str,
        translation: Optional[list[float]] = None,
        orientation: Optional[list[float]] = None,
        frequency: float = _DEFAULT_FREQUENCY,
    ) -> None:
        if translation is None:
            translation = list(_DEFAULT_TRANSLATION)
        if orientation is None:
            orientation = list(_DEFAULT_ORIENTATION)
        if len(translation) != 3:
            raise ValueError(
                "IsaacImuSensor: translation must have 3 elements,"
                f" got {len(translation)}"
            )
        if len(orientation) != 4:
            raise ValueError(
                "IsaacImuSensor: orientation must have 4 elements (wxyz),"
                f" got {len(orientation)}"
            )
        if frequency < 0:
            raise ValueError(f"IsaacImuSensor: frequency must be >= 0, got {frequency}")
        self._prim_path = prim_path
        self._translation = translation
        self._orientation = orientation
        self._frequency = frequency
        self._sensor = None
        self._ready: bool = False

    def setup(self) -> bool:
        """Initialise the IMUSensor against the live USD stage."""
        if self._sensor is not None and self._ready:
            return True
        try:
            import numpy as np  # pylint: disable=E0401
            from isaacsim.sensors.physics import IMUSensor  # pylint: disable=E0401

            sensor_kwargs: dict = {
                "prim_path": self._prim_path,
                "name": "peppy_imu_sensor",
                "translation": np.array(self._translation, dtype=float),
                "orientation": np.array(self._orientation, dtype=float),
            }
            if self._frequency > 0:
                sensor_kwargs["frequency"] = self._frequency
            self._sensor = IMUSensor(**sensor_kwargs)
            self._sensor.initialize()
            self._ready = True
        except Exception as exc:
            logger.error(
                f"Failed to setup IsaacImuSensor at '{self._prim_path}': {exc}"
            )
            self._sensor = None
            self._ready = False
            return False

        logger.info(
            f"IsaacImuSensor ready — prim='{self._prim_path}'"
            f" translation={self._translation} orientation={self._orientation}"
            f" frequency={self._frequency}"
        )
        return True

    def teardown(self) -> None:
        """Reset sensor state."""
        self._sensor = None
        self._ready = False

    def get_imu_data(self) -> Optional[dict]:
        """Read orientation (wxyz), angular velocity, and linear acceleration."""
        if not self._ready or self._sensor is None:
            return None

        try:
            frame = self._sensor.get_current_frame()
            return {
                "orientation": [float(v) for v in frame["orientation"]],
                "angular_velocity": [float(v) for v in frame["ang_vel"]],
                "linear_acceleration": [float(v) for v in frame["lin_acc"]],
            }
        except Exception as exc:
            logger.warning(f"Could not read IMU data: {exc}")
            return None

    @property
    def is_ready(self) -> bool:
        """True when the IMUSensor has been initialised."""
        return self._ready
