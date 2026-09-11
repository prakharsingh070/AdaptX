"""Stored-evidence endpoints for the dashboard (Phase 12).

Read-only access to evaluation reports and recorded runs in the configured
directories. A report is served whole; a run is served as a summary plus one
frame at a time. Two reports are compared with the Phase 11 contract. Nothing
is written, nothing is recomputed, nothing reaches a simulator.
"""

from __future__ import annotations

from typing import Any

from fastapi import APIRouter, Query, status

from adaptx.api.dependencies import ContextDep
from adaptx.api.schemas import ErrorResponse
from adaptx.evaluation.compare import ComparisonReport
from adaptx.evaluation.models import EvaluationReport
from adaptx.evidence.service import RunFrame, RunSummary, StoredFile
from adaptx.models.common import AdaptXModel

router = APIRouter(tags=["evidence"])

_NOT_FOUND: dict[int | str, dict[str, Any]] = {
    404: {"model": ErrorResponse, "description": "Missing or invalid file"}
}


class StoredFileList(AdaptXModel):
    """The files in one evidence directory."""

    directory: str
    files: list[StoredFile]


@router.get(
    "/reports",
    response_model=StoredFileList,
    status_code=status.HTTP_200_OK,
    summary="List stored evaluation reports",
)
def list_reports(context: ContextDep) -> StoredFileList:
    """Every ``*.json`` in the report directory. Listing does not validate."""
    return StoredFileList(
        directory=str(context.evidence.report_dir), files=context.evidence.list_reports()
    )


@router.get(
    "/reports/compare",
    response_model=ComparisonReport,
    status_code=status.HTTP_200_OK,
    responses=_NOT_FOUND,
    summary="Compare two stored reports with the Phase 11 contract",
)
def compare_reports(
    context: ContextDep,
    first: str = Query(min_length=1),
    second: str = Query(min_length=1),
) -> ComparisonReport:
    """Deterministic-content comparison, timings excluded, exactly as the CLI does it."""
    return context.evidence.compare(first, second)


@router.get(
    "/reports/{name}",
    response_model=EvaluationReport,
    status_code=status.HTTP_200_OK,
    responses=_NOT_FOUND,
    summary="One stored evaluation report, validated",
)
def load_report(name: str, context: ContextDep) -> EvaluationReport:
    """The report as written by ``python -m adaptx.evaluation``; 404 with the
    validation error if the file is not one."""
    return context.evidence.load_report(name)


@router.get(
    "/runs",
    response_model=StoredFileList,
    status_code=status.HTTP_200_OK,
    summary="List stored scenario runs",
)
def list_runs(context: ContextDep) -> StoredFileList:
    """Every ``*.json`` in the run directory. Listing does not validate."""
    return StoredFileList(
        directory=str(context.evidence.run_dir), files=context.evidence.list_runs()
    )


@router.get(
    "/runs/{name}",
    response_model=RunSummary,
    status_code=status.HTTP_200_OK,
    responses=_NOT_FOUND,
    summary="One stored run without its frames",
)
def run_summary(name: str, context: ContextDep) -> RunSummary:
    """Metadata, sensor configuration and the frame index; frames come one at a time."""
    return context.evidence.run_summary(name)


@router.get(
    "/runs/{name}/frames/{index}",
    response_model=RunFrame,
    status_code=status.HTTP_200_OK,
    responses=_NOT_FOUND,
    summary="One frame of a stored run, with its ground truth",
)
def run_frame(name: str, index: int, context: ContextDep) -> RunFrame:
    """The frame's record, pipeline outputs and - labelled - its ground truth.

    Ground truth is served here and nowhere else: this is evaluation playback,
    not the live channel.
    """
    return context.evidence.run_frame(name, index)
