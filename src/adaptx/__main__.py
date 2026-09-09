"""Console entry point: ``python -m adaptx`` or ``adaptx``."""

from __future__ import annotations

import uvicorn

from adaptx.config.settings import get_settings
from adaptx.core.logging import configure_logging


def main() -> None:
    """Start the ADAPT-X backend with the configured host, port and log level."""
    settings = get_settings()
    configure_logging(settings.logging.level, json_format=settings.logging.json_format)
    uvicorn.run(
        "adaptx.api.app:create_app",
        factory=True,
        host=settings.api.host,
        port=settings.api.port,
        reload=settings.api.reload,
        log_config=None,
    )


if __name__ == "__main__":
    main()
