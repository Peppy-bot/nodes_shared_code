from __future__ import annotations

import logging
from typing import Optional

logger = logging.getLogger(__name__)


class MujocoTransformTree:
    """Reads world-frame transforms for every body in the MuJoCo model."""

    def __init__(self, model, data) -> None:
        self._model = model
        self._data = data
        self._bodies: list[tuple[int, str, str]] = []  # (id, name, parent_name)
        self._ready: bool = False

    def setup(self) -> bool:
        """Build static body list from the model (body 0 = world, skipped)."""
        if self._ready:
            return True
        try:
            import mujoco  # pylint: disable=E0401

            bodies = []
            for body_id in range(1, self._model.nbody):
                name = mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_BODY, body_id)
                if name is None:
                    name = f"body_{body_id}"
                parent_id = self._model.body_parentid[body_id]
                if parent_id == 0:
                    parent_name = "world"
                else:
                    parent_name = (
                        mujoco.mj_id2name(
                            self._model, mujoco.mjtObj.mjOBJ_BODY, parent_id
                        )
                        or f"body_{parent_id}"
                    )
                bodies.append((body_id, name, parent_name))

            self._bodies = bodies
            self._ready = True
        except Exception as exc:
            logger.error(f"Failed to setup MujocoTransformTree: {exc}")
            return False

        logger.info(f"MujocoTransformTree ready — {len(self._bodies)} bodies")
        return True

    def teardown(self) -> None:
        """Reset sensor state."""
        self._bodies = []
        self._ready = False

    def get_tf_data(self) -> Optional[list[dict]]:
        """Return world-frame position and orientation for every body."""
        if not self._ready:
            return None

        try:
            frames = []
            for body_id, name, parent_name in self._bodies:
                frames.append(
                    {
                        "name": name,
                        "parent": parent_name,
                        "position": [float(v) for v in self._data.xpos[body_id]],
                        "orientation": [float(v) for v in self._data.xquat[body_id]],
                    }
                )
            return frames
        except Exception as exc:
            logger.warning(f"Could not read TF tree data: {exc}")
            return None

    @property
    def is_ready(self) -> bool:
        """True when the body list has been built."""
        return self._ready
