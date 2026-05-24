from __future__ import annotations

import logging
from typing import Optional

logger = logging.getLogger(__name__)


class MujocoOdometrySensor:
    """Reads world-frame pose and velocity of a named MuJoCo body (base link).

    Uses data.xpos / data.xquat for position/orientation and data.cvel for
    velocity. cvel layout is [ang_vel_x, ang_vel_y, ang_vel_z, lin_vel_x,
    lin_vel_y, lin_vel_z] — angular comes first in MuJoCo convention.
    """

    def __init__(self, model, data, body_name: str) -> None:
        self._model = model
        self._data = data
        self._body_name = body_name
        self._body_id: Optional[int] = None
        self._ready: bool = False

    def setup(self) -> bool:
        """Resolve body ID from the MuJoCo model."""
        try:
            import mujoco  # pylint: disable=E0401

            body_id = mujoco.mj_name2id(
                self._model, mujoco.mjtObj.mjOBJ_BODY, self._body_name
            )
            if body_id < 0:
                logger.error(
                    f"Odometry body '{self._body_name}' not found in model."
                    " Check the 'prim' field matches a body name in your MJCF."
                )
                return False
            self._body_id = body_id
            self._ready = True
        except Exception as exc:
            logger.error(f"Failed to setup MujocoOdometrySensor: {exc}")
            return False

        logger.info(
            f"MujocoOdometrySensor ready — body='{self._body_name}' id={self._body_id}"
        )
        return True

    def teardown(self) -> None:
        """Reset sensor state."""
        self._ready = False
        self._body_id = None

    def get_odometry_data(self) -> Optional[dict]:
        """Return position, orientation (wxyz), linear/angular velocity."""
        if not self._ready:
            return None

        try:
            cvel = self._data.cvel[self._body_id]
            return {
                "position": [
                    float(self._data.xpos[self._body_id][i]) for i in range(3)
                ],
                "orientation": [
                    float(self._data.xquat[self._body_id][i]) for i in range(4)
                ],
                # cvel = [ang_vel(3), lin_vel(3)]
                "linear_velocity": [float(cvel[i]) for i in range(3, 6)],
                "angular_velocity": [float(cvel[i]) for i in range(3)],
            }
        except Exception as exc:
            logger.warning(f"Could not read odometry data: {exc}")
            return None

    @property
    def is_ready(self) -> bool:
        """True when body ID has been resolved."""
        return self._ready
