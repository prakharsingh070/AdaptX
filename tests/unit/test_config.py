"""Configuration system tests."""

from __future__ import annotations

import pytest
from pydantic import ValidationError

from adaptx.config.settings import (
    Environment,
    LiDARSettings,
    MapSettings,
    RiskSettings,
    Settings,
    get_settings,
    reload_settings,
)


def test_defaults_are_development_safe() -> None:
    settings = Settings()
    assert settings.app.environment is Environment.DEVELOPMENT
    assert settings.api.port == 8000
    assert settings.carla.enabled is False
    assert settings.carla.use_mock is False
    assert settings.is_production is False


def test_environment_variables_override_nested_fields(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("ADAPTX_API__PORT", "9123")
    monkeypatch.setenv("ADAPTX_CARLA__HOST", "carla.internal")
    monkeypatch.setenv("ADAPTX_APP__ENVIRONMENT", "production")

    settings = Settings(_env_file=None)

    assert settings.api.port == 9123
    assert settings.carla.host == "carla.internal"
    assert settings.is_production is True


def test_log_level_is_normalised(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setenv("ADAPTX_LOGGING__LEVEL", "debug")
    assert Settings(_env_file=None).logging.level == "DEBUG"


def test_invalid_port_is_rejected() -> None:
    with pytest.raises(ValidationError):
        Settings(api={"port": 0})


def test_lidar_point_bounds_must_be_ordered() -> None:
    with pytest.raises(ValidationError, match="min_points"):
        LiDARSettings(min_points=100, max_points=10)


def test_map_resolutions_must_go_coarse_to_fine() -> None:
    with pytest.raises(ValidationError, match="coarse"):
        MapSettings(resolution_low_m=0.1, resolution_critical_m=1.0)


def test_risk_thresholds_must_increase() -> None:
    with pytest.raises(ValidationError, match="medium < high < critical"):
        RiskSettings(threshold_medium=0.9, threshold_high=0.5, threshold_critical=0.95)


def test_get_settings_is_cached_and_reloadable(monkeypatch: pytest.MonkeyPatch) -> None:
    first = get_settings()
    assert get_settings() is first

    monkeypatch.setenv("ADAPTX_API__PORT", "9999")
    reloaded = reload_settings()
    try:
        assert reloaded is not first
        assert reloaded.api.port == 9999
    finally:
        monkeypatch.delenv("ADAPTX_API__PORT", raising=False)
        reload_settings()


def test_no_secret_fields_are_declared() -> None:
    """The settings surface must not carry credentials in Phase 1."""
    forbidden = {"password", "secret", "token", "api_key", "credential"}
    for section in Settings().model_dump().values():
        if not isinstance(section, dict):
            continue
        for field in section:
            assert not any(word in field.lower() for word in forbidden)
