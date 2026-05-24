from __future__ import annotations

from abc import ABC, abstractmethod
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from typing import Any

    PeppylibIO = Any


class BridgePlugin(ABC):

    @abstractmethod
    def setup(self) -> bool: ...

    @abstractmethod
    def on_step(self, step: int, io: "PeppylibIO") -> None: ...

    @abstractmethod
    def teardown(self) -> None: ...

    def try_setup(self) -> bool:
        if self.is_ready:
            return True
        return self.setup()

    def subscriptions(self) -> list[tuple[str, str, str]]:
        return []

    @property
    @abstractmethod
    def is_ready(self) -> bool: ...
