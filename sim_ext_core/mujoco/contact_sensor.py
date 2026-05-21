from __future__ import annotations

import logging
from typing import Optional

logger = logging.getLogger(__name__)


class MujocoContactSensor:
    """Reads active contact forces involving a named MuJoCo body.

    Iterates data.contact for active contacts where either contacting geom
    belongs to the target body.  Forces are reported in world frame via
    mj_contactForce + the contact rotation matrix.

    Returns an empty list when no contacts are active — not an error.
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
                    f"Contact body '{self._body_name}' not found in model."
                    " Check the 'prim' field matches a body name in your MJCF."
                )
                return False
            self._body_id = body_id
            self._ready = True
        except Exception as exc:
            logger.error(f"Failed to setup MujocoContactSensor: {exc}")
            return False

        logger.info(
            f"MujocoContactSensor ready — body='{self._body_name}' id={self._body_id}"
        )
        return True

    def teardown(self) -> None:
        """Reset sensor state."""
        self._ready = False
        self._body_id = None

    def get_contact_data(self) -> Optional[list[dict]]:
        """Return active contacts involving the target body.

        Each entry: body1 (geom name), body2 (geom name), position (3),
        force (3, world frame).
        """
        if not self._ready:
            return None

        try:
            import mujoco  # pylint: disable=E0401
            import numpy as np  # pylint: disable=E0401

            contacts = []
            for i in range(self._data.ncon):
                contact = self._data.contact[i]
                geom1_body = int(self._model.geom_bodyid[contact.geom1])
                geom2_body = int(self._model.geom_bodyid[contact.geom2])
                if self._body_id not in (geom1_body, geom2_body):
                    continue

                # mj_contactForce returns 6-DOF wrench in contact frame.
                # contact.frame is a (3,3) rotation matrix (contact → world).
                force_contact = np.zeros(6)
                mujoco.mj_contactForce(self._model, self._data, i, force_contact)
                frame = contact.frame.reshape(3, 3)
                force_world = frame @ force_contact[:3]

                geom1_name = (
                    mujoco.mj_id2name(
                        self._model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom1
                    )
                    or f"geom_{contact.geom1}"
                )
                geom2_name = (
                    mujoco.mj_id2name(
                        self._model, mujoco.mjtObj.mjOBJ_GEOM, contact.geom2
                    )
                    or f"geom_{contact.geom2}"
                )

                contacts.append(
                    {
                        "body1": geom1_name,
                        "body2": geom2_name,
                        "position": contact.pos.tolist(),
                        "force": force_world.tolist(),
                    }
                )
            return contacts
        except Exception as exc:
            logger.warning(f"Could not read contact data: {exc}")
            return None

    @property
    def is_ready(self) -> bool:
        """True when body ID has been resolved."""
        return self._ready
