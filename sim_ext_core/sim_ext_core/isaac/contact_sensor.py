from __future__ import annotations

import logging
from typing import Optional

logger = logging.getLogger(__name__)

_DEFAULT_MIN_THRESHOLD = 0
_DEFAULT_MAX_THRESHOLD = 10000
_DEFAULT_RADIUS = 1.0


class IsaacContactSensor:
    """Reads active contact forces at an Isaac Sim rigid body prim.

    Uses isaacsim.sensors.physics.ContactSensor — creates the sensor prim
    programmatically if it does not exist. Parent prim must have CollisionAPI.

    Returns an empty contact list when no contacts are active — this is not
    an error condition.
    """

    def __init__(
        self,
        prim_path: str,
        min_threshold: float = _DEFAULT_MIN_THRESHOLD,
        max_threshold: float = _DEFAULT_MAX_THRESHOLD,
        radius: float = _DEFAULT_RADIUS,
    ) -> None:
        self._prim_path = prim_path
        self._min_threshold = min_threshold
        self._max_threshold = max_threshold
        self._radius = radius
        self._sensor = None
        self._ready: bool = False

    def setup(self) -> bool:
        """Initialise the contact sensor against the live USD stage."""
        if self._sensor is not None and self._ready:
            return True
        try:
            import numpy as np  # pylint: disable=E0401
            import omni.usd  # pylint: disable=E0401
            from isaacsim.sensors.physics import ContactSensor  # pylint: disable=E0401

            stage = omni.usd.get_context().get_stage()
            if stage and stage.GetPrimAtPath(self._prim_path).IsValid():
                stage.RemovePrim(self._prim_path)

            self._sensor = ContactSensor(
                prim_path=self._prim_path,
                name="peppy_contact_sensor",
                translation=np.zeros(3),
                min_threshold=self._min_threshold,
                max_threshold=self._max_threshold,
                radius=self._radius,
            )
            self._sensor.initialize()
            self._sensor.add_raw_contact_data_to_frame()
            self._ready = True
        except Exception as exc:
            logger.error(
                f"Failed to setup IsaacContactSensor at '{self._prim_path}': {exc}"
            )
            self._sensor = None
            self._ready = False
            return False

        logger.info(f"IsaacContactSensor ready — prim='{self._prim_path}'")
        return True

    def teardown(self) -> None:
        self._sensor = None
        self._ready = False

    def get_contact_data(self) -> Optional[list[dict]]:
        """Return list of active contacts, each with body1, body2, position, force."""
        if not self._ready or self._sensor is None:
            return None

        try:
            frame = self._sensor.get_current_frame()
            if not frame.get("in_contact", False):
                return []

            contacts = []
            for entry in frame.get("contacts", []):
                contacts.append(
                    {
                        "body1": str(entry.get("body0", "")),
                        "body2": str(entry.get("body1", "")),
                        "position": [
                            float(v) for v in entry.get("position", [0.0, 0.0, 0.0])
                        ],
                        "force": [
                            float(v) for v in entry.get("impulse", [0.0, 0.0, 0.0])
                        ],
                    }
                )
            return contacts
        except Exception as exc:
            logger.warning(f"Could not read contact data: {exc}")
            return None

    @property
    def is_ready(self) -> bool:
        """True when the contact sensor has been initialised."""
        return self._ready
