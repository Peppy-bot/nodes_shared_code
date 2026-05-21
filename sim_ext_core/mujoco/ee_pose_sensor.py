from __future__ import annotations

import logging
from typing import Optional

logger = logging.getLogger(__name__)


class MujocoEePoseSensor:
    """Reads world-frame pose of a named MuJoCo body (end-effector link).

    prim is the last path component of the config entry and must match the
    body name in the MuJoCo XML model exactly.
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
                    f"EE body '{self._body_name}' not found in model."
                    " Check the 'prim' field matches a body name in your MJCF."
                )
                return False
            self._body_id = body_id
            self._ready = True
        except Exception as exc:
            logger.error(f"Failed to setup MujocoEePoseSensor: {exc}")
            return False

        logger.info(
            f"MujocoEePoseSensor ready — body='{self._body_name}' id={self._body_id}"
        )
        return True

    def teardown(self) -> None:
        """Reset sensor state."""
        self._ready = False
        self._body_id = None

    def get_ee_pose(self) -> Optional[dict]:
        """Return position (xyz) and orientation (wxyz) in world frame."""
        if not self._ready:
            return None

        try:
            return {
                "position": [
                    float(self._data.xpos[self._body_id][i]) for i in range(3)
                ],
                "orientation": [
                    float(self._data.xquat[self._body_id][i]) for i in range(4)
                ],
            }
        except Exception as exc:
            logger.warning(f"Could not read EE pose: {exc}")
            return None

    @property
    def is_ready(self) -> bool:
        """True when body ID has been resolved."""
        return self._ready
