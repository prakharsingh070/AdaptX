"""Structured logging tests."""

from __future__ import annotations

import json
import logging

from adaptx.core.logging import JsonFormatter, TextFormatter, configure_logging, get_logger


def _record(message: str = "hello", **context: object) -> logging.LogRecord:
    record = logging.LogRecord(
        name="adaptx.test",
        level=logging.INFO,
        pathname=__file__,
        lineno=1,
        msg=message,
        args=(),
        exc_info=None,
    )
    if context:
        record.context = context  # type: ignore[attr-defined]
    return record


def test_json_formatter_emits_the_required_fields() -> None:
    payload = json.loads(JsonFormatter().format(_record()))

    assert payload["level"] == "INFO"
    assert payload["logger"] == "adaptx.test"
    assert payload["message"] == "hello"
    assert payload["timestamp"].endswith("+00:00")


def test_json_formatter_includes_context() -> None:
    payload = json.loads(JsonFormatter().format(_record(frame_id=7, sensor_id="lidar_0")))
    assert payload["context"] == {"frame_id": 7, "sensor_id": "lidar_0"}


def test_json_formatter_omits_empty_context() -> None:
    assert "context" not in json.loads(JsonFormatter().format(_record()))


def test_text_formatter_is_single_line_with_context() -> None:
    line = TextFormatter().format(_record(frame_id=3))

    assert "\n" not in line
    assert "INFO" in line
    assert "adaptx.test" in line
    assert "frame_id=3" in line


def test_json_formatter_records_exceptions() -> None:
    try:
        raise ValueError("boom")
    except ValueError:
        import sys

        record = _record("failed")
        record.exc_info = sys.exc_info()

    payload = json.loads(JsonFormatter().format(record))
    assert "ValueError: boom" in payload["exception"]


def test_configure_logging_is_idempotent() -> None:
    configure_logging("INFO")
    first = len(logging.getLogger().handlers)
    configure_logging("DEBUG", json_format=True)

    root = logging.getLogger()
    assert len(root.handlers) == first
    assert root.level == logging.DEBUG

    configure_logging("WARNING")  # restore a quiet level for the rest of the suite


def test_get_logger_returns_a_named_logger() -> None:
    assert get_logger("adaptx.example").name == "adaptx.example"
