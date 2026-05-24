from __future__ import annotations

import logging
from typing import Optional

logger = logging.getLogger(__name__)


class IsaacClockSensor:
    """Reads the current simulation time from the Isaac Sim timeline."""

    def __init__(self) -> None:
        self._ready: bool = False

    def setup(self) -> bool:
        """Verify the timeline interface is accessible."""
        try:
            import omni.timeline  # pylint: disable=E0401

            omni.timeline.get_timeline_interface()
            self._ready = True
        except Exception as exc:
            logger.error(f"Failed to setup IsaacClockSensor: {exc}")
            self._ready = False
            return False

        logger.info("IsaacClockSensor ready")
        return True

    def teardown(self) -> None:
        """Reset sensor state."""
        self._ready = False

    def get_clock_data(self) -> Optional[dict]:
        """Return the current simulation time in seconds."""
        if not self._ready:
            return None

        try:
            import omni.timeline  # pylint: disable=E0401

            sim_time = float(omni.timeline.get_timeline_interface().get_current_time())
            return {"sim_time": sim_time}
        except Exception as exc:
            logger.warning(f"Could not read clock data: {exc}")
            return None

    @property
    def is_ready(self) -> bool:
        """True when the timeline interface has been verified accessible."""
        return self._ready
