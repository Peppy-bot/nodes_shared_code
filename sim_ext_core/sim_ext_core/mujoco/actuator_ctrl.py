from __future__ import annotations

import logging

logger = logging.getLogger(__name__)


class MujocoActuatorCtrl:
    """Resolves MJCF actuator names to ctrl indices and writes targets into data.ctrl[].

    Used by ActuatorCtrlBridge to translate raw set_ctrl messages
    ({actuator_values: {name: value, ...}}) into in-process MuJoCo ctrl writes
    before mj_step.
    """

    def __init__(self, model, data) -> None:
        self._model = model
        self._data = data
        self._name_to_id: dict[str, int] = {}
        self._ready: bool = False

    def setup(self) -> bool:
        """Build the name → ctrl-id map.

        Accepts both the MJCF actuator name and the joint name the actuator
        drives, so callers can address ctrl indices by either. The joint name
        is the cross-engine canonical key (Isaac's dof_names are joint names),
        which lets components ship one payload shape that works on both engines.
        """
        try:
            import mujoco  # pylint: disable=E0401

            name_to_id: dict[str, int] = {}
            joint_aliases = 0
            for i in range(self._model.nu):
                actuator_name = (
                    mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
                    or ""
                )
                if actuator_name:
                    name_to_id[actuator_name] = i
                if int(self._model.actuator_trntype[i]) != int(
                    mujoco.mjtTrn.mjTRN_JOINT
                ):
                    continue
                trnid = int(self._model.actuator_trnid[i, 0])
                joint_name = (
                    mujoco.mj_id2name(self._model, mujoco.mjtObj.mjOBJ_JOINT, trnid)
                    or ""
                )
                if joint_name and joint_name not in name_to_id:
                    name_to_id[joint_name] = i
                    joint_aliases += 1
            self._name_to_id = name_to_id
            self._ready = True
        except Exception as exc:
            logger.error(f"Failed to setup MujocoActuatorCtrl: {exc}")
            return False

        logger.info(
            f"MujocoActuatorCtrl ready — {self._model.nu} actuator(s) resolved"
            f" ({joint_aliases} joint-name aliases)"
        )
        return True

    def teardown(self) -> None:
        self._ready = False
        self._name_to_id = {}

    def write_targets(self, actuator_values: dict) -> int:
        """Write each {name: value} pair into data.ctrl[ctrl_id[name]].

        Returns the count of values actually applied. Unknown actuator names
        are logged and dropped — they should not stop the rest of the batch.
        """
        if not self._ready:
            return 0
        if not isinstance(actuator_values, dict):
            logger.warning(
                f"actuator_values must be a dict, got {type(actuator_values).__name__}"
            )
            return 0
        applied = 0
        for name, value in actuator_values.items():
            ctrl_id = self._name_to_id.get(name)
            if ctrl_id is None:
                logger.warning(f"unknown actuator '{name}' — dropped")
                continue
            try:
                self._data.ctrl[ctrl_id] = float(value)
                applied += 1
            except Exception as exc:
                logger.warning(f"failed to write ctrl[{ctrl_id}] for '{name}': {exc}")
        return applied

    @property
    def is_ready(self) -> bool:
        return self._ready
