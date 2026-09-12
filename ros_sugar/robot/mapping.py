"""Mapping capability descriptors.

A robot plugin declares *how* maps of its environment get built.
Mapping is a one-off, operator-driven activity that produces a file
on disk, so it is driven from the EMOS CLI.

A declaration is read through ``python -m ros_sugar.robot inspect``.

Two kinds:

- `VendorMapping` -- the robot ships its own SLAM, reached by running a vendor
  tool. The plugin supplies the argv and where the results land.
- `NativeMapping` -- EMOS maps the environment itself from the plugin's own
  LiDAR and IMU feedbacks.

A plugin that declares neither simply cannot be mapped, and the CLI says so.
"""

from __future__ import annotations

from typing import Any, Dict, List, Optional

from attrs import define, field

from ..config import BaseAttrs


@define(kw_only=True)
class VendorMapping(BaseAttrs):
    """Mapping performed by the robot's own software.

    The commands are argv lists rather than shell strings. ``{name}`` is
    substituted with the map name at call time.

    :param start: Begin a mapping session.
    :param stop: End it and save. Expect to run this more than once -- some
        vendor tools need repeating before the finishing step completes, so
        the caller retries and then verifies the map directory appeared rather
        than trusting one exit code.
    :param store: Directory holding every map on the robot.
    :param grid: Occupancy-grid YAML filename inside a map directory. The
        default is the ROS ``map_server`` convention, which is also what
        Kompass's ``MapServer`` loads.
    :param cloud: Point-cloud filename inside a map directory.
    :param apply: Make a map the active one. ``None`` if the vendor offers no
        such command.
    :param after_apply: Run after ``apply`` -- typically restarting the
        vendor's localization service, without which the switch silently does
        not take effect.
    :param export: Package the active map for copying off the robot.
    :param active_link: Name of the symlink in ``store`` pointing at the
        active map.
    :param requires_root: Whether the commands need privilege escalation. When
        true the dashboard cannot drive this provider -- its daemon has no
        terminal for a password prompt -- and directs the operator to the CLI.
    :param host: ``"local"`` when the commands run on the machine EMOS is
        installed on, otherwise ``"ssh://user@host"``.
    :param area_limit_m: Largest square area the vendor supports, in metres,
        if documented. Advisory only; shown to the operator before they start.
    """

    start: List[str] = field()
    stop: List[str] = field()
    store: str = field()
    grid: str = field(default="occ_grid.yaml")
    cloud: str = field(default="full_cloud.pcd")
    apply: Optional[List[str]] = field(default=None)
    after_apply: Optional[List[str]] = field(default=None)
    export: Optional[List[str]] = field(default=None)
    active_link: str = field(default="active")
    requires_root: bool = field(default=True)
    host: str = field(default="local")
    area_limit_m: Optional[float] = field(default=None)

    @property
    def kind(self) -> str:
        """Discriminator for consumers reading a `describe` tree."""
        return "vendor"

    def spec(self) -> Dict[str, Any]:
        """JSON-serializable introspection record, tagged with `kind`."""
        record = self.asdict()
        record["kind"] = self.kind
        return record


@define(kw_only=True)
class NativeMapping(BaseAttrs):
    """Mapping performed by EMOS from the plugin's own sensor feedbacks.

    The inputs are named by **feedback key**, not by topic, so the plugin stays
    the single source of truth for what the topic actually is and a topic
    rename does not invalidate this declaration.

    :param cloud: Feedback key of the LiDAR point cloud.
    :param imu: Feedback key of an IMU mounted with the LiDAR, at a rate
        suitable for LiDAR-inertial odometry -- typically the one inside the
        LiDAR itself. A robot-state IMU arriving at 10 Hz over telemetry is
        far too slow. ``None`` means no IMU is available, which limits EMOS to
        a LiDAR-only backend and degrades the result on a robot whose gait
        pitches the sensor.
    :param z_min: Bottom of the height band kept when flattening the 3D map to
        an occupancy grid, in metres above the base frame's ground plane.
        Anything below is floor.
    :param z_max: Top of that band. Anything above cannot obstruct the robot.
    :param resolution: Grid cell size in metres.
    """

    cloud: str = field()
    imu: Optional[str] = field(default=None)
    z_min: float = field(default=0.15)
    z_max: float = field(default=0.80)
    resolution: float = field(default=0.05)

    @property
    def kind(self) -> str:
        """Discriminator for consumers reading a `describe` tree."""
        return "native"

    def spec(self) -> Dict[str, Any]:
        """JSON-serializable introspection record, tagged with `kind`."""
        record = self.asdict()
        record["kind"] = self.kind
        return record


__all__ = ["VendorMapping", "NativeMapping"]
