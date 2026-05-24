from __future__ import annotations

import logging
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .articulation import ArticulationBridge

logger = logging.getLogger(__name__)


class IsaacSimControl:
    """Isaac Sim implementation of simulator control operations.

    Uses the shared ArticulationBridge for set_joint_positions.
    Pause and reset delegate to the Isaac Sim timeline interface.
    step_sim is not supported in the callback-driven Isaac Sim model.
    """

    def __init__(self, articulation: "ArticulationBridge", timeline) -> None:
        self._articulation = articulation
        self._timeline = timeline

    @property
    def is_paused(self) -> bool:
        try:
            return self._timeline.is_stopped() or not self._timeline.is_playing()
        except Exception:
            return False

    def reset(self) -> dict:
        try:
            from omni.isaac.core import World  # pylint: disable=E0401

            world = World.instance()
            if world is not None:
                world.reset()
            else:
                self._timeline.stop()
                self._timeline.play()
            logger.info("IsaacSimControl: simulation reset.")
            return {"success": True, "message": "Simulation reset"}
        except Exception as exc:
            logger.error(f"IsaacSimControl reset failed: {exc}")
            return {"success": False, "message": str(exc)}

    def pause(self, paused: bool) -> dict:
        try:
            if paused:
                self._timeline.pause()
            else:
                self._timeline.play()
            state = "paused" if paused else "resumed"
            logger.info(f"IsaacSimControl: simulation {state}.")
            return {"success": True, "paused": paused}
        except Exception as exc:
            logger.error(f"IsaacSimControl pause failed: {exc}")
            return {"success": False, "paused": self.is_paused}

    def step(self, _num_steps: int) -> dict:
        logger.warning("IsaacSimControl.step: not supported in callback-driven mode.")
        return {"success": False, "steps_executed": 0, "sim_time": 0.0}

    def set_joint_positions(  # pylint: disable=R0911
        self, arm: str, positions: list[float]
    ) -> dict:
        if arm not in ("left", "right"):
            return {
                "success": False,
                "message": f"arm must be 'left' or 'right', got '{arm}'",
            }
        joint_names = [f"openarm_{arm}_joint{i}" for i in range(1, 8)]
        if len(positions) != len(joint_names):
            return {
                "success": False,
                "message": (
                    f"expected 7 positions for '{arm}' arm, got {len(positions)}"
                ),
            }

        if not self._articulation.is_ready and not self._articulation.setup():
            return {"success": False, "message": "Articulation not ready"}

        states = self._articulation.get_joint_states()
        if states is None:
            return {"success": False, "message": "Articulation not ready"}

        current_positions, _ = states
        new_positions = list(current_positions)

        dof_names = self._articulation.get_dof_names()
        if not dof_names:
            return {
                "success": False,
                "message": "Could not read DOF names from articulation",
            }

        try:
            for joint_name, pos in zip(joint_names, positions):
                idx = dof_names.index(joint_name)
                new_positions[idx] = pos
        except ValueError as exc:
            return {"success": False, "message": f"Joint not found: {exc}"}

        if not self._articulation.apply_command(new_positions):
            return {"success": False, "message": "apply_command failed"}

        logger.info(
            f"IsaacSimControl: set joint positions for '{arm}' arm — {positions}"
        )
        return {"success": True, "message": f"Joint positions set for '{arm}' arm"}
