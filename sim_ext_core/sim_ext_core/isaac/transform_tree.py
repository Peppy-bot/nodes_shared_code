from __future__ import annotations

import logging
from typing import Optional

logger = logging.getLogger(__name__)


class IsaacTransformTree:
    """Reads world-frame transforms for every body in an Isaac Sim articulation."""

    def __init__(self, prim_path: str) -> None:
        self._prim_path = prim_path
        self._articulation = None
        self._body_parents: list[str] = []
        self._ready: bool = False

    def setup(self) -> bool:
        """Initialise articulation and build static parent name list from USD stage."""
        if self._articulation is not None and self._ready:
            return True
        try:
            import omni.usd  # pylint: disable=E0401
            from isaacsim.core.prims import Articulation  # pylint: disable=E0401

            self._articulation = Articulation(prim_paths_expr=self._prim_path)
            self._articulation.initialize()

            stage = omni.usd.get_context().get_stage()
            self._body_parents = []
            for name in self._articulation.body_names:
                prim = stage.GetPrimAtPath(f"{self._prim_path}/{name}")
                if prim.IsValid():
                    parent_name = prim.GetParent().GetName()
                else:
                    parent_name = "world"
                self._body_parents.append(parent_name)

            self._ready = True
        except Exception as exc:
            logger.error(
                f"Failed to setup IsaacTransformTree at '{self._prim_path}': {exc}"
            )
            self._articulation = None
            self._ready = False
            return False

        logger.info(
            f"IsaacTransformTree ready — prim='{self._prim_path}'"
            f" bodies={len(self._articulation.body_names)}"
        )
        return True

    def teardown(self) -> None:
        """Reset transform tree state."""
        self._articulation = None
        self._body_parents = []
        self._ready = False

    def _body_frame(self, stage, time_code, name: str, parent_name: str) -> dict:
        """Return a single body's world-frame position and orientation dict."""
        from pxr import UsdGeom  # pylint: disable=E0401

        prim = stage.GetPrimAtPath(f"{self._prim_path}/{name}")
        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(time_code)
        translation = matrix.ExtractTranslation()
        rotation = matrix.ExtractRotationQuat()
        img = rotation.GetImaginary()
        return {
            "name": name,
            "parent": parent_name,
            "position": [translation[0], translation[1], translation[2]],
            "orientation": [rotation.GetReal(), img[0], img[1], img[2]],
        }

    def get_tf_data(self) -> Optional[list[dict]]:
        """Return world-frame position and orientation for every body."""
        if not self._ready or self._articulation is None:
            return None

        try:
            import omni.usd  # pylint: disable=E0401
            from pxr import Usd  # pylint: disable=E0401

            stage = omni.usd.get_context().get_stage()
            time_code = Usd.TimeCode.Default()
            frames = []
            for i, name in enumerate(self._articulation.body_names):
                prim = stage.GetPrimAtPath(f"{self._prim_path}/{name}")
                if not prim.IsValid():
                    continue
                frames.append(
                    self._body_frame(stage, time_code, name, self._body_parents[i])
                )
            return frames
        except Exception as exc:
            logger.warning(f"Could not read transform tree data: {exc}")
            return None

    @property
    def is_ready(self) -> bool:
        """True when the articulation has been initialised."""
        return self._ready
