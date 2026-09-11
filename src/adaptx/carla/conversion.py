"""CARLA-to-ADAPT-X conversion (Phase 9).

**The single coordinate boundary.** CARLA uses a left-handed frame with +y to
the **right**; ADAPT-X uses a right-handed frame with +y to the **left**
(ADR-009). Every value crossing from the simulator is converted here and
nowhere else, so no downstream module ever has to ask which convention a
number is in (ADR-043).

::

    CARLA (left-handed, +y right)
              |
              v   this module, exactly once
    ADAPT-X (right-handed, +y left)
              |
              v
    preprocessing -> detection -> tracking -> prediction
                  -> mapping -> risk -> adaptive resolution

Deliberately free of ``import carla``
-------------------------------------
Nothing here touches the CARLA package. Every function takes plain floats,
bytes or arrays, which is what makes the conversion - the part where a sign
error would silently mirror the world - fully testable on a machine with no
simulator installed. :mod:`adaptx.carla.session` does the talking to CARLA and
calls into here.

Time is simulation time
-----------------------
Frame timestamps come from the simulator clock, never from the wall clock
(ADR-044). Simulation time is reported by CARLA as seconds elapsed since the
server started, which is a duration rather than a date, so it is anchored to a
fixed epoch to produce the absolute, ordered, reproducible timestamps the
frame contract requires.
"""

from __future__ import annotations

import math
from datetime import UTC, datetime, timedelta

import numpy as np

from adaptx.core.exceptions import InvalidPointCloudError
from adaptx.models.common import CoordinateFrame, DataSource, Vector3
from adaptx.models.point_cloud import XYZ_FIELDS, XYZI_FIELDS, RawPointCloudFrame

#: Fixed anchor for simulation time.
#:
#: CARLA reports elapsed seconds since the server started, so it carries no
#: date. This epoch turns that duration into an absolute instant. The choice of
#: date is arbitrary and carries no meaning - what matters is that it never
#: changes, so two runs of the same scenario produce identical timestamps and
#: `frame N < frame N+1` always holds.
SIMULATION_EPOCH = datetime(2000, 1, 1, 0, 0, 0, tzinfo=UTC)

#: Floats CARLA packs per LiDAR point: x, y, z, intensity.
CARLA_LIDAR_STRIDE = 4

#: Stage name used in conversion errors, matching the pipeline convention.
STAGE = "carla_ingest"


# ---------------------------------------------------------------------------
# geometry
# ---------------------------------------------------------------------------
def flip_y(x: float, y: float, z: float) -> tuple[float, float, float]:
    """Convert one CARLA position to the ADAPT-X frame.

    The whole conversion is a sign flip on y: both frames put +x forward and
    +z up, and they differ only in handedness. Negating exactly one axis is
    what turns a left-handed frame into a right-handed one.
    """
    return (x, -y, z)


def carla_location_to_vector3(x: float, y: float, z: float) -> Vector3:
    """Convert a CARLA location to an ADAPT-X :class:`Vector3`."""
    converted = flip_y(x, y, z)
    return Vector3(x=converted[0], y=converted[1], z=converted[2])


def carla_yaw_to_heading_rad(yaw_deg: float) -> float:
    """Convert a CARLA yaw in degrees to an ADAPT-X heading in radians.

    ADAPT-X measures heading counter-clockwise from +x in a right-handed frame
    (ADR-009). CARLA measures yaw in degrees in a left-handed frame, so a
    positive CARLA yaw turns towards +y-right, which is *negative* in ADAPT-X.
    Mirroring the y axis therefore mirrors the sense of rotation too - the same
    single handedness change, expressed as an angle.

    A CARLA yaw of +90 degrees points along CARLA +y (right), which is ADAPT-X
    -y, which is -pi/2. Hence the negation.
    """
    if not math.isfinite(yaw_deg):
        raise InvalidPointCloudError(
            f"{STAGE}: CARLA yaw must be finite, got {yaw_deg}",
            details={"stage": STAGE, "yaw_deg": yaw_deg},
        )
    return math.radians(-yaw_deg)


def carla_points_to_adaptx(points: np.ndarray) -> np.ndarray:
    """Convert an ``(N, 3)`` or ``(N, 4)`` CARLA point array to the ADAPT-X frame.

    Vectorised: one sign flip over a column, never a Python loop over points.
    The input is not modified, so a caller may keep the CARLA-frame array.

    A fourth column is intensity, not a coordinate, and is passed through
    untouched.
    """
    if points.ndim != 2 or points.shape[1] not in (3, CARLA_LIDAR_STRIDE):
        raise InvalidPointCloudError(
            f"{STAGE}: expected an (N, 3) or (N, 4) array, got shape {points.shape}",
            details={"stage": STAGE, "shape": list(points.shape)},
        )
    converted = np.array(points, dtype=np.float64, copy=True)
    converted[:, 1] *= -1.0
    return converted


def carla_world_to_ego(
    point: tuple[float, float, float],
    ego_position: tuple[float, float, float],
    ego_yaw_deg: float,
) -> Vector3:
    """Express a CARLA world point in the ego frame, in ADAPT-X coordinates.

    Both points are flipped into the ADAPT-X frame **first**, and the rotation
    is then an ordinary right-handed one. Doing it in that order means there is
    exactly one handedness change in the whole function; rotating first and
    flipping afterwards would need the sign of the rotation reversed as well,
    which is precisely the kind of double negative that hides a bug.

    Args:
        point: World position in CARLA coordinates.
        ego_position: Ego world position in CARLA coordinates.
        ego_yaw_deg: Ego yaw in CARLA degrees.

    Returns:
        The point relative to the ego, in the ADAPT-X frame: +x ahead of the
        vehicle, +y to its left.
    """
    px, py, pz = flip_y(*point)
    ex, ey, ez = flip_y(*ego_position)
    heading = carla_yaw_to_heading_rad(ego_yaw_deg)

    dx, dy, dz = px - ex, py - ey, pz - ez
    cos_h, sin_h = math.cos(heading), math.sin(heading)
    return Vector3(
        x=dx * cos_h + dy * sin_h,
        y=-dx * sin_h + dy * cos_h,
        z=dz,
    )


def carla_vector_to_ego(vector: tuple[float, float, float], ego_yaw_deg: float) -> Vector3:
    """Rotate a CARLA world *direction* into the ego frame (no translation).

    For velocities and other free vectors, which have direction but no
    position.
    """
    vx, vy, vz = flip_y(*vector)
    heading = carla_yaw_to_heading_rad(ego_yaw_deg)
    cos_h, sin_h = math.cos(heading), math.sin(heading)
    return Vector3(x=vx * cos_h + vy * sin_h, y=-vx * sin_h + vy * cos_h, z=vz)


def ego_offset_to_carla_world(
    offset: tuple[float, float, float],
    ego_position: tuple[float, float, float],
    ego_yaw_deg: float,
) -> tuple[float, float, float]:
    """Place an ADAPT-X ego-frame offset back into CARLA world coordinates.

    The exact inverse of :func:`carla_world_to_ego`, and the function a
    scenario needs: "put a vehicle 30 m ahead and 3 m to the left of the ego"
    is a statement in the ego frame, but CARLA spawns actors in world
    coordinates.

    Args:
        offset: Position relative to the ego, ADAPT-X frame (+x ahead, +y left).
        ego_position: Ego world position in CARLA coordinates.
        ego_yaw_deg: Ego yaw in CARLA degrees.

    Returns:
        World position in CARLA coordinates, ready to hand to the simulator.
    """
    ox, oy, oz = offset
    heading = carla_yaw_to_heading_rad(ego_yaw_deg)
    cos_h, sin_h = math.cos(heading), math.sin(heading)

    # Body -> world is the transpose of the world -> body rotation.
    wx = ox * cos_h - oy * sin_h
    wy = ox * sin_h + oy * cos_h

    ex, ey, ez = flip_y(*ego_position)
    return flip_y(ex + wx, ey + wy, ez + oz)


def adaptx_offset_to_carla(x: float, y: float, z: float) -> tuple[float, float, float]:
    """Convert an ADAPT-X offset **back** into CARLA's frame.

    The only direction that travels outward: sensor mount offsets are
    configured in the ADAPT-X convention (so configuration never mixes frames)
    and have to be handed to CARLA in its own. The conversion is its own
    inverse, which is why it is the same sign flip.
    """
    return flip_y(x, y, z)


# ---------------------------------------------------------------------------
# time
# ---------------------------------------------------------------------------
def simulation_timestamp(elapsed_seconds: float) -> datetime:
    """Convert CARLA simulation seconds to an absolute UTC timestamp.

    Simulation time is authoritative: this never consults the wall clock, so a
    replay of the same scenario produces byte-identical timestamps and the
    interval between frames is exactly ``fixed_delta_seconds``.
    """
    if not math.isfinite(elapsed_seconds) or elapsed_seconds < 0.0:
        raise InvalidPointCloudError(
            f"{STAGE}: simulation time must be finite and non-negative, got {elapsed_seconds}",
            details={"stage": STAGE, "elapsed_seconds": elapsed_seconds},
        )
    return SIMULATION_EPOCH + timedelta(seconds=elapsed_seconds)


# ---------------------------------------------------------------------------
# LiDAR
# ---------------------------------------------------------------------------
def decode_lidar_buffer(buffer: bytes | bytearray | memoryview) -> np.ndarray:
    """Decode a CARLA LiDAR ``raw_data`` buffer into an ``(N, 4)`` array.

    CARLA packs each point as four little-endian float32 values - x, y, z,
    intensity - so the buffer is reinterpreted rather than parsed. This is one
    ``frombuffer`` over the whole scan, not a loop over tens of thousands of
    points.

    The result is still in **CARLA coordinates**; pass it through
    :func:`carla_points_to_adaptx`.
    """
    raw = np.frombuffer(buffer, dtype=np.float32)
    if raw.size % CARLA_LIDAR_STRIDE != 0:
        raise InvalidPointCloudError(
            f"{STAGE}: LiDAR buffer holds {raw.size} floats, which is not a whole "
            f"number of {CARLA_LIDAR_STRIDE}-float points; the measurement is malformed",
            details={"stage": STAGE, "float_count": int(raw.size)},
        )
    return raw.reshape(-1, CARLA_LIDAR_STRIDE)


def build_raw_frame(
    *,
    points_carla: np.ndarray,
    frame_id: int,
    sensor_id: str,
    elapsed_seconds: float,
    include_intensity: bool = True,
) -> RawPointCloudFrame:
    """Assemble a :class:`RawPointCloudFrame` from a decoded CARLA scan.

    This is the moment simulator data becomes ADAPT-X data. Three things are
    fixed here and never revisited downstream:

    * coordinates are converted to the ADAPT-X frame;
    * the timestamp is simulation time;
    * ``source`` is :attr:`~adaptx.models.common.DataSource.SIMULATION` — never
      ``LIVE_SENSOR``. A simulated scan must never be presentable as a
      measurement from real hardware.

    ``RawPointCloudFrame`` rather than ``PointCloudFrame`` on purpose: raw is
    the type the ingest path expects, it permits the non-finite values a sensor
    model may produce, and Phase 2A is what turns it into a validated frame
    (ADR-010).
    """
    converted = carla_points_to_adaptx(points_carla)
    if not include_intensity and converted.shape[1] == CARLA_LIDAR_STRIDE:
        converted = np.ascontiguousarray(converted[:, :3])

    return RawPointCloudFrame(
        timestamp=simulation_timestamp(elapsed_seconds),
        frame_id=frame_id,
        sensor_id=sensor_id,
        points=converted,
        fields=XYZI_FIELDS if converted.shape[1] == CARLA_LIDAR_STRIDE else XYZ_FIELDS,
        coordinate_frame=CoordinateFrame.LIDAR,
        source=DataSource.SIMULATION,
    )
