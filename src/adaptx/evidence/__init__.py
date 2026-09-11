"""Stored evidence served read-only to the dashboard (Phase 12).

This package sits DOWNSTREAM of the evaluation layer, beside it rather than
inside the perception pipeline: it reads Phase 10 run records and Phase 11
evaluation reports from disk and hands them to the API unchanged. No
pipeline package imports it, and it computes nothing (ADR-055).
"""

from adaptx.evidence.service import EvidenceService, RunFrame, RunSummary, StoredFile

__all__ = ["EvidenceService", "RunFrame", "RunSummary", "StoredFile"]
