from __future__ import annotations

import logging
from typing import Optional

logger = logging.getLogger(__name__)


class IsaacOdometrySensor:
    """Reads world-frame pose and velocity of a rigid body prim (base link)."""

    def __init__(self, prim_path: str) -> None:
        self._prim_path = prim_path
        self._prim = None
        self._ready: bool = False

    def setup(self) -> bool:
        """Initialise the RigidPrim against the live USD stage."""
        if self._prim is not None and self._ready:
            return True
        try:
            from isaacsim.core.prims import RigidPrim  # pylint: disable=E0401

            self._prim = RigidPrim(prim_paths_expr=self._prim_path)
            self._prim.initialize()
            self._ready = True
        except Exception as exc:
            logger.error(
                f"Failed to setup IsaacOdometrySensor at '{self._prim_path}': {exc}"
            )
            self._prim = None
            self._ready = False
            return False

        logger.info(f"IsaacOdometrySensor ready — prim='{self._prim_path}'")
        return True

    def teardown(self) -> None:
        """Reset sensor state."""
        self._prim = None
        self._ready = False

    def get_odometry_data(self) -> Optional[dict]:
        """Return position, orientation (wxyz), linear/angular velocity."""
        if not self._ready or self._prim is None:
            return None

        try:
            positions, orientations = self._prim.get_world_poses()
            linear_vel = self._prim.get_linear_velocities()[0]
            angular_vel = self._prim.get_angular_velocities()[0]
            return {
                "position": positions[0].tolist(),
                "orientation": orientations[0].tolist(),
                "linear_velocity": linear_vel.tolist(),
                "angular_velocity": angular_vel.tolist(),
            }
        except Exception as exc:
            logger.warning(f"Could not read odometry data: {exc}")
            return None

    @property
    def is_ready(self) -> bool:
        """True when the RigidPrim has been initialised."""
        return self._ready
