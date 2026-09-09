"""CARLA boundary tests.

The whole suite runs without CARLA installed. These tests assert that the real
client degrades explicitly, and that the mock is deterministic and clearly
labelled synthetic.
"""

from __future__ import annotations

import pytest

from adaptx.carla.client import CarlaClient, carla_package_available
from adaptx.carla.mock import MockCarlaSimulatorClient
from adaptx.config.settings import CarlaSettings
from adaptx.core.exceptions import SimulatorUnavailableError
from adaptx.models.common import DataSource
from adaptx.models.system import CarlaStatus
from adaptx.services.carla_service import CarlaService, build_client


class TestRealClientWithoutCarla:
    def test_disabled_client_refuses_to_connect(self) -> None:
        client = CarlaClient(CarlaSettings(enabled=False))
        with pytest.raises(SimulatorUnavailableError, match="disabled"):
            client.connect()
        assert client.is_connected is False

    @pytest.mark.skipif(carla_package_available(), reason="the optional carla package is installed")
    def test_enabled_client_reports_the_missing_package(self) -> None:
        client = CarlaClient(CarlaSettings(enabled=True))
        with pytest.raises(SimulatorUnavailableError, match="not installed"):
            client.connect()

    def test_operations_require_a_connection(self) -> None:
        client = CarlaClient(CarlaSettings(enabled=True))
        with pytest.raises(SimulatorUnavailableError, match="not connected"):
            client.get_world()
        with pytest.raises(SimulatorUnavailableError, match="not connected"):
            client.get_vehicle_state()


class TestMockClient:
    @pytest.fixture
    def client(self) -> MockCarlaSimulatorClient:
        return MockCarlaSimulatorClient()

    def test_declares_itself_a_mock(self, client: MockCarlaSimulatorClient) -> None:
        assert client.is_mock is True

    def test_requires_connection_before_use(self, client: MockCarlaSimulatorClient) -> None:
        with pytest.raises(SimulatorUnavailableError, match="not connected"):
            client.get_world()

    def test_connect_and_disconnect(self, client: MockCarlaSimulatorClient) -> None:
        world = client.connect()
        assert client.is_connected is True
        assert world.map_name == "MockTown"

        client.disconnect()
        assert client.is_connected is False

    def test_sensor_data_is_labelled_synthetic(self, client: MockCarlaSimulatorClient) -> None:
        client.connect()
        frame = client.get_sensor_data()
        assert frame.source is DataSource.SYNTHETIC_TEST
        assert frame.point_count == 512

    def test_frame_ids_increase(self, client: MockCarlaSimulatorClient) -> None:
        client.connect()
        assert [client.get_sensor_data().frame_id for _ in range(3)] == [0, 1, 2]

    def test_generated_clouds_are_deterministic(self) -> None:
        first = MockCarlaSimulatorClient()
        second = MockCarlaSimulatorClient()
        first.connect()
        second.connect()
        assert (first.get_sensor_data().points == second.get_sensor_data().points).all()

    def test_vehicle_state_is_labelled_synthetic(self, client: MockCarlaSimulatorClient) -> None:
        client.connect()
        assert client.get_vehicle_state().source is DataSource.SYNTHETIC_TEST

    def test_actor_lifecycle(self, client: MockCarlaSimulatorClient) -> None:
        client.connect()
        vehicle = client.spawn_vehicle("vehicle.tesla.model3", role="ego")
        walker = client.spawn_actor("walker.pedestrian.0001")

        assert vehicle.role == "ego"
        assert walker.actor_id != vehicle.actor_id
        assert client.get_world().actor_count == 2

        assert client.destroy_actor(vehicle.actor_id) is True
        assert client.destroy_actor(vehicle.actor_id) is False
        assert client.get_world().actor_count == 1

    def test_load_world_clears_actors(self, client: MockCarlaSimulatorClient) -> None:
        client.connect()
        client.spawn_vehicle("vehicle.audi.tt")
        world = client.load_world("OtherTown")
        assert world.map_name == "OtherTown"
        assert world.actor_count == 0


class TestCarlaService:
    def test_disabled_service_reports_disconnected(self) -> None:
        service = CarlaService(CarlaSettings(enabled=False))
        assert service.connect() is False

        status = service.status()
        assert status.status is CarlaStatus.DISCONNECTED
        assert status.enabled is False
        assert "disabled" in status.detail

    def test_failed_connection_does_not_raise(self) -> None:
        """A missing or unreachable simulator must never crash the backend."""
        service = CarlaService(CarlaSettings(enabled=True, host="127.0.0.1", port=1))
        assert service.connect() is False
        assert service.status().status is CarlaStatus.DISCONNECTED
        assert service.status().detail != ""

    def test_mock_is_selected_only_by_configuration(self) -> None:
        assert build_client(CarlaSettings(use_mock=False)).is_mock is False
        assert build_client(CarlaSettings(use_mock=True)).is_mock is True

    def test_mock_service_reports_connected_and_flagged(self) -> None:
        settings = CarlaSettings(enabled=True, use_mock=True)
        service = CarlaService(settings)
        assert service.connect() is True

        status = service.status()
        assert status.status is CarlaStatus.CONNECTED
        assert status.is_mock is True
        assert status.world == "MockTown"

        service.disconnect()
        assert service.status().status is CarlaStatus.DISCONNECTED
