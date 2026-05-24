from __future__ import annotations

import logging
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    pass

logger = logging.getLogger(__name__)


class MujocoSimControl:
    """MuJoCo implementation of simulator control operations.

    Wraps a MuJoCo model/data pair and exposes reset, pause, step, and
    set_joint_positions operations. Intended to be used with SimControlBridge.
    """

    def __init__(self, model, data) -> None:
        self._model = model
        self._data = data
        self._paused: bool = False

    @property
    def is_paused(self) -> bool:
        return self._paused

    def reset(self) -> dict:
        """Reset simulator to initial state."""
        try:
            import mujoco  # pylint: disable=E0401

            mujoco.mj_resetData(self._model, self._data)
            logger.info("MuJoCo: simulation reset.")
            return {"success": True, "message": "Simulation reset"}
        except Exception as exc:
            logger.error(f"MuJoCo reset failed: {exc}")
            return {"success": False, "message": str(exc)}

    def pause(self, paused: bool) -> dict:
        """Pause or resume physics stepping."""
        self._paused = paused
        state = "paused" if paused else "resumed"
        logger.info(f"MuJoCo: simulation {state}.")
        return {"success": True, "paused": self._paused}

    def step(self, num_steps: int) -> dict:
        """Step physics num_steps times regardless of pause state.

        This is an explicit step request from a user node — it overrides pause.
        Returns sim_time from data.time so the caller is in sync with the clock topic.
        """
        if num_steps <= 0:
            return {
                "success": True,
                "steps_executed": 0,
                "sim_time": float(self._data.time),
            }
        try:
            import mujoco  # pylint: disable=E0401

            for _ in range(num_steps):
                mujoco.mj_step(self._model, self._data)
            sim_time = float(self._data.time)
            return {"success": True, "steps_executed": num_steps, "sim_time": sim_time}
        except Exception as exc:
            logger.error(f"MuJoCo step failed: {exc}")
            return {"success": False, "steps_executed": 0, "sim_time": 0.0}

    def set_joint_positions(self, arm: str, positions: list[float]) -> dict:
        """Set joint angles for the specified arm of the bimanual robot.

        Looks up each joint by name (openarm_{arm}_joint1..7) using MuJoCo's
        joint name index so the mapping is robust to XML ordering changes.
        Calls mj_forward after writing qpos to update kinematics immediately.
        """
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
                    f"expected {len(joint_names)} positions for '{arm}' arm, "
                    f"got {len(positions)}"
                ),
            }
        try:
            import mujoco  # pylint: disable=E0401

            for name, pos in zip(joint_names, positions):
                jid = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_JOINT, name)
                if jid == -1:
                    return {
                        "success": False,
                        "message": f"Joint '{name}' not found in MuJoCo model",
                    }
                self._data.qpos[self._model.jnt_qposadr[jid]] = pos
            mujoco.mj_forward(self._model, self._data)
            logger.info(f"MuJoCo: set joint positions for '{arm}' arm — {positions}")
            return {"success": True, "message": f"Joint positions set for '{arm}' arm"}
        except Exception as exc:
            logger.error(f"MuJoCo set_joint_positions failed: {exc}")
            return {"success": False, "message": str(exc)}
