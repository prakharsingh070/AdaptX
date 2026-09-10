"""Baseline geometric classification (Phase 3).

**This is a geometric heuristic, not semantic recognition.** It compares a
cluster's measured size against dimension bands for a few common road users. It
has no appearance model, no learned features and no training data. It cannot
tell a pedestrian from a post of the same size, a parked motorcycle from a
bicycle, or a van from a skip.

Every rule below is stated explicitly so a reader can predict the output, and
anything that does not fit exactly one band comes back as ``UNKNOWN``.

Rotation
--------
Bounding boxes are axis-aligned (Phase 3 does not fit oriented boxes), so a
vehicle at 45 degrees to the sensor measures wider and shorter than it is. The
rules therefore use **footprint length** = the larger horizontal extent and
**footprint width** = the smaller, rather than x and y directly. That makes the
comparison invariant to a 90-degree rotation, though a diagonal object still
measures larger than it is - a documented weakness, not a solved problem.

Bands
-----
Height is the z extent; length and width are the larger and smaller horizontal
extents. All in metres.

======================  ============  ================  ===============
Class                   Height        Footprint length  Footprint width
======================  ============  ================  ===============
``PEDESTRIAN``          0.90 - 2.20   0.20 - 1.20       0.20 - 1.20
``BICYCLE``             0.60 - 2.00   1.20 - 2.60       0.20 - 1.00
``VEHICLE``             1.00 - 2.60   2.60 - 6.50       1.30 - 2.60
``OBSTACLE``            0.15 - 1.00   0.20 - 3.00       0.20 - 3.00
======================  ============  ================  ===============

The project's existing :class:`~adaptx.models.common.ObjectClass` names the
bicycle class ``CYCLIST``; that contract is reused rather than duplicated.

Bands deliberately overlap - a bicycle and a narrow pedestrian occupy similar
space. Where a cluster fits **more than one** band it is ``UNKNOWN``, because
picking the "best" match would manufacture a distinction the geometry does not
support.

Confidence
----------
The reported confidence is a **geometric fit score**, not a probability that the
label is right. For the matched band it is the smallest per-dimension fit, where
each dimension scores 1.0 at the centre of its band and falls linearly to
:data:`EDGE_FIT` at the edges. ``UNKNOWN`` scores 0.0, since nothing matched.

No labelled dataset exists in this project, so no calibrated probability can be
produced. Reporting one would be an invention.

These bands are baseline values for a generic road scene. They are not tuned,
not validated, and not suitable for autonomous-driving decisions.
"""

from __future__ import annotations

from dataclasses import dataclass

from adaptx.models.common import ObjectClass

#: Fit score assigned exactly at the edge of a band. Above zero because a
#: cluster on the boundary still matched; below one because it matched weakly.
EDGE_FIT = 0.35


@dataclass(frozen=True)
class DimensionBand:
    """Inclusive dimension ranges that describe one class."""

    object_class: ObjectClass
    min_height_m: float
    max_height_m: float
    min_length_m: float
    max_length_m: float
    min_width_m: float
    max_width_m: float

    def contains(self, height: float, length: float, width: float) -> bool:
        """True when all three measurements fall inside this band."""
        return (
            self.min_height_m <= height <= self.max_height_m
            and self.min_length_m <= length <= self.max_length_m
            and self.min_width_m <= width <= self.max_width_m
        )

    def fit(self, height: float, length: float, width: float) -> float:
        """Weakest per-dimension fit, in ``[EDGE_FIT, 1.0]``."""
        return min(
            _axis_fit(height, self.min_height_m, self.max_height_m),
            _axis_fit(length, self.min_length_m, self.max_length_m),
            _axis_fit(width, self.min_width_m, self.max_width_m),
        )


def _axis_fit(value: float, low: float, high: float) -> float:
    """1.0 at the centre of ``[low, high]``, falling to EDGE_FIT at the edges."""
    half = (high - low) / 2.0
    if half <= 0.0:
        return 1.0
    centre = (low + high) / 2.0
    offset = min(abs(value - centre) / half, 1.0)
    return 1.0 - (1.0 - EDGE_FIT) * offset


#: The bands, in the order they are reported. Order does not affect the result:
#: a cluster matching several is UNKNOWN regardless.
BANDS: tuple[DimensionBand, ...] = (
    DimensionBand(ObjectClass.PEDESTRIAN, 0.90, 2.20, 0.20, 1.20, 0.20, 1.20),
    DimensionBand(ObjectClass.CYCLIST, 0.60, 2.00, 1.20, 2.60, 0.20, 1.00),
    DimensionBand(ObjectClass.VEHICLE, 1.00, 2.60, 2.60, 6.50, 1.30, 2.60),
    DimensionBand(ObjectClass.OBSTACLE, 0.15, 1.00, 0.20, 3.00, 0.20, 3.00),
)


class GeometricClassifier:
    """Assigns a class from cluster dimensions alone. Baseline, not learned."""

    name = "geometric_bands_v1"
    #: True while classification is a heuristic rather than a trained model.
    is_baseline = True

    def classify(
        self, extent_x_m: float, extent_y_m: float, extent_z_m: float
    ) -> tuple[ObjectClass, float]:
        """Classify a cluster from its axis-aligned extents.

        Args:
            extent_x_m: Extent along +x (forward).
            extent_y_m: Extent along +y (left).
            extent_z_m: Extent along +z (up).

        Returns:
            The class and its geometric fit score. ``(UNKNOWN, 0.0)`` when no
            band matches, or when more than one does.
        """
        height = extent_z_m
        length = max(extent_x_m, extent_y_m)
        width = min(extent_x_m, extent_y_m)

        matches = [band for band in BANDS if band.contains(height, length, width)]
        if len(matches) != 1:
            # Nothing matched, or the geometry is genuinely ambiguous between
            # classes. Either way the shape does not identify the object.
            return ObjectClass.UNKNOWN, 0.0

        band = matches[0]
        return band.object_class, round(band.fit(height, length, width), 4)
