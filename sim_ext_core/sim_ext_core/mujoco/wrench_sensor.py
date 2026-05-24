from __future__ import annotations

import logging
from typing import Optional

logger = logging.getLogger(__name__)


class MujocoWrenchSensor:
    """Reads total external contact force/torque on a named MuJoCo body.

    Uses data.cfrc_ext which gives the net external force and torque acting on
    each body in world frame. Layout: [torque_x, torque_y, torque_z, force_x,
    force_y, force_z] — torque comes first in MuJoCo convention.
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
                    f"Wrench body '{self._body_name}' not found in model."
                    " Check the 'prim' field matches a body name in your MJCF."
                )
                return False
            self._body_id = body_id
            self._ready = True
        except Exception as exc:
            logger.error(f"Failed to setup MujocoWrenchSensor: {exc}")
            return False

        logger.info(
            f"MujocoWrenchSensor ready — body='{self._body_name}' id={self._body_id}"
        )
        return True

    def teardown(self) -> None:
        """Reset sensor state."""
        self._ready = False
        self._body_id = None

    def get_wrench_data(self) -> Optional[dict]:
        """Return net external force [fx, fy, fz] and torque [tx, ty, tz]."""
        if not self._ready:
            return None

        try:
            cfrc = self._data.cfrc_ext[self._body_id]
            # cfrc_ext layout: [torque(3), force(3)]
            return {
                "force": [float(cfrc[i]) for i in range(3, 6)],
                "torque": [float(cfrc[i]) for i in range(3)],
            }
        except Exception as exc:
            logger.warning(f"Could not read wrench data: {exc}")
            return None

    @property
    def is_ready(self) -> bool:
        """True when body ID has been resolved."""
        return self._ready
