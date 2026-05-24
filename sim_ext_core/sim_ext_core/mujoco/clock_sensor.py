from __future__ import annotations

import logging
from typing import Optional

logger = logging.getLogger(__name__)


class MujocoClockSensor:
    """Reads the current simulation time from MuJoCo data.time."""

    def __init__(self, model, data) -> None:
        self._model = model
        self._data = data
        self._ready: bool = False

    def setup(self) -> bool:
        """Always succeeds — data.time is unconditionally available."""
        self._ready = True
        logger.info("MujocoClockSensor ready")
        return True

    def teardown(self) -> None:
        """Reset sensor state."""
        self._ready = False

    def get_clock_data(self) -> Optional[dict]:
        """Return the current simulation time in seconds."""
        if not self._ready:
            return None

        try:
            return {"sim_time": float(self._data.time)}
        except Exception as exc:
            logger.warning(f"Could not read clock data: {exc}")
            return None

    @property
    def is_ready(self) -> bool:
        """True once setup() has been called."""
        return self._ready
