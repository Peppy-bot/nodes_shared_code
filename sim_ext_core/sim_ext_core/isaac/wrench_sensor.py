from __future__ import annotations

import logging
from typing import Optional

logger = logging.getLogger(__name__)


class IsaacWrenchSensor:
    """Reads joint reaction force/torque from an Isaac Sim articulation.

    Uses Articulation.get_measured_joint_forces() which returns PhysX joint
    reaction forces as (num_envs, num_dof, 6) where the 6 values are
    [fx, fy, fz, tx, ty, tz] in world frame.

    joint_index=-1 selects the last joint (end-effector), which is the default.
    """

    def __init__(self, prim_path: str, joint_index: int = -1) -> None:
        self._prim_path = prim_path
        self._joint_index = joint_index
        self._articulation = None
        self._ready: bool = False

    def setup(self) -> bool:
        """Initialise the Articulation against the live USD stage."""
        if self._articulation is not None and self._ready:
            return True
        try:
            from isaacsim.core.prims import Articulation  # pylint: disable=E0401

            self._articulation = Articulation(prim_paths_expr=self._prim_path)
            self._articulation.initialize()

            num_dof = self._articulation.num_dof
            in_range = -num_dof <= self._joint_index <= num_dof - 1
            if not in_range:
                logger.error(
                    f"IsaacWrenchSensor: joint_index={self._joint_index}"
                    f" out of range for '{self._prim_path}' ({num_dof} DOF)."
                )
                self._articulation = None
                return False

            # Normalise negative index once so get_wrench_data() never errors.
            if self._joint_index < 0:
                self._joint_index = num_dof + self._joint_index

            self._ready = True
        except Exception as exc:
            logger.error(
                f"Failed to setup IsaacWrenchSensor at '{self._prim_path}': {exc}"
            )
            self._articulation = None
            self._ready = False
            return False

        logger.info(
            f"IsaacWrenchSensor ready — prim='{self._prim_path}'"
            f" joint_index={self._joint_index}"
        )
        return True

    def teardown(self) -> None:
        """Reset sensor state."""
        self._articulation = None
        self._ready = False

    def get_wrench_data(self) -> Optional[dict]:
        """Return [fx, fy, fz] force and [tx, ty, tz] torque at the selected joint."""
        if not self._ready or self._articulation is None:
            return None

        try:
            # Shape: (1, num_dof, 6) — [fx, fy, fz, tx, ty, tz]
            forces = self._articulation.get_measured_joint_forces()
            joint_wrench = forces[0][self._joint_index]
            return {
                "force": [float(v) for v in joint_wrench[:3]],
                "torque": [float(v) for v in joint_wrench[3:]],
            }
        except Exception as exc:
            logger.warning(f"Could not read wrench data: {exc}")
            return None

    @property
    def is_ready(self) -> bool:
        """True when the Articulation has been initialised."""
        return self._ready
