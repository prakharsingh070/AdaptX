"""Temporal tracking layer.

Phase 4 implements a deterministic geometric baseline: gated nearest-neighbour
association, measured velocity, and a track lifecycle. No learned model, no
appearance features, no re-identification.
"""

from adaptx.tracking.association import AssociationOutcome, Candidate, associate
from adaptx.tracking.interfaces import ObjectTracker
from adaptx.tracking.tracker import GeometricObjectTracker, build_tracker

__all__ = [
    "AssociationOutcome",
    "Candidate",
    "GeometricObjectTracker",
    "ObjectTracker",
    "associate",
    "build_tracker",
]
