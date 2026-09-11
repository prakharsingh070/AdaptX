"""ADAPT-X exception hierarchy.

Every error raised deliberately by ADAPT-X derives from :class:`AdaptXError` so
the API layer can translate it into an explicit HTTP response instead of a
generic 500. Errors are never swallowed to hide missing functionality.
"""

from __future__ import annotations

from typing import Any


class AdaptXError(Exception):
    """Base class for all ADAPT-X errors."""

    #: Machine-readable code returned to API clients.
    code: str = "adaptx_error"
    #: HTTP status used when this error escapes to the API layer.
    http_status: int = 500

    def __init__(self, message: str, *, details: dict[str, Any] | None = None) -> None:
        super().__init__(message)
        self.message = message
        self.details: dict[str, Any] = details or {}

    def to_dict(self) -> dict[str, Any]:
        """Serialise the error for an API response body."""
        return {"code": self.code, "message": self.message, "details": self.details}


class ConfigurationError(AdaptXError):
    """Invalid or inconsistent configuration."""

    code = "configuration_error"
    http_status = 500


class ValidationError(AdaptXError):
    """Input rejected because it violates a data contract."""

    code = "validation_error"
    http_status = 422


class InvalidPointCloudError(ValidationError):
    """A point-cloud frame is malformed or violates the configured limits."""

    code = "invalid_point_cloud"
    http_status = 422


class SimulatorUnavailableError(AdaptXError):
    """The simulator (CARLA) is not installed, not enabled, or not reachable."""

    code = "simulator_unavailable"
    http_status = 503


class ModuleNotReadyError(AdaptXError):
    """A subsystem exists as an interface but has no implementation yet.

    Raised by placeholder code paths so an unimplemented module can never be
    mistaken for a working one.
    """

    code = "module_not_ready"
    http_status = 501


class StoredEvidenceError(AdaptXError):
    """A stored run or report could not be found or did not validate."""

    code = "stored_evidence_error"
    http_status = 404
