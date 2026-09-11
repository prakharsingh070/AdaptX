"""Live scene endpoints for the dashboard (Phase 12).

``GET /scene/latest`` returns the most recent :class:`SceneSnapshot` the
pipeline produced or was handed; ``POST /scene/frame`` lets a local
producer - the scenario CLI with ``--publish`` - hand one over for display.
The POST computes nothing: the snapshot is validated, stored and broadcast
(ADR-055). There is no endpoint here, or anywhere, that spawns, moves, starts
or stops anything.
"""

from __future__ import annotations

from fastapi import APIRouter, Response, status

from adaptx.api.dependencies import ContextDep
from adaptx.api.schemas import ErrorResponse
from adaptx.models.common import AdaptXModel
from adaptx.models.scene import SceneSnapshot

router = APIRouter(prefix="/scene", tags=["scene"])


class ScenePublishResponse(AdaptXModel):
    """Acknowledgement of a published snapshot."""

    accepted: bool
    sequence: int
    frame_id: int


class SceneLatestResponse(AdaptXModel):
    """The current scene, or an explicit statement that there is none."""

    sequence: int
    has_scene: bool
    scene: SceneSnapshot | None = None
    detail: str


@router.get(
    "/latest",
    response_model=SceneLatestResponse,
    status_code=status.HTTP_200_OK,
    summary="Most recent scene the pipeline produced",
)
def latest_scene(context: ContextDep) -> SceneLatestResponse:
    """The last frame's tracks, paths, assessments, tiles and point sample.

    ``has_scene`` is false until something has run the pipeline - a request
    to the LiDAR endpoints or a scenario run publishing its frames. The
    dashboard shows that state plainly rather than an empty scene that looks
    like an empty road.
    """
    sequence, scene = context.scene.latest()
    return SceneLatestResponse(
        sequence=sequence,
        has_scene=scene is not None,
        scene=scene,
        detail=(
            "no frame has been processed yet"
            if scene is None
            else f"frame {scene.frame_id} from {scene.origin}"
        ),
    )


@router.post(
    "/frame",
    response_model=ScenePublishResponse,
    status_code=status.HTTP_202_ACCEPTED,
    responses={422: {"model": ErrorResponse, "description": "Malformed snapshot"}},
    summary="Publish a scene snapshot a pipeline already produced",
)
def publish_scene(
    snapshot: SceneSnapshot, context: ContextDep, response: Response
) -> ScenePublishResponse:
    """Accept one frame's pipeline outputs for display.

    Used by ``python -m adaptx.scenarios run --publish``: the scenario's own
    pipeline processes the frame and the outputs are handed here as-is. The
    backend stores and broadcasts them; it runs no stage, changes no value and
    never sees ground truth (the snapshot contract has no field for it).
    """
    sequence = context.scene.publish(snapshot)
    response.headers["X-Scene-Sequence"] = str(sequence)
    return ScenePublishResponse(accepted=True, sequence=sequence, frame_id=snapshot.frame_id)
