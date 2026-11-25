import pytest
from pymavlink import mavutil
from pymavlink.dialects.v20 import ardupilotmega as mavlink

from aeac_telemetry.vehicle import VehicleState, flight_mode

ARMED = mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
GUIDED, LOITER = 4, 5


def heartbeat(*, armed=True, custom_mode=GUIDED, status=mavutil.mavlink.MAV_STATE_ACTIVE):
    return mavlink.MAVLink_heartbeat_message(
        mavutil.mavlink.MAV_TYPE_QUADROTOR,
        mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | (ARMED if armed else 0),
        custom_mode,
        status,
        3,
    )


def flying_vehicle() -> VehicleState:
    vehicle = VehicleState()
    for msg in (
        heartbeat(),
        mavlink.MAVLink_global_position_int_message(
            0, 501022871, -1107373276, 765000, 15000, 0, 0, 0, 0
        ),
        mavlink.MAVLink_gps_raw_int_message(
            0, 3, 0, 0, 0, 80, 150, 0, 0, 14, h_acc=800, v_acc=1500
        ),
        mavlink.MAVLink_sys_status_message(0, 0, 0, 0, 0, 0, 87, 0, 0, 0, 0, 0, 0),
        mavlink.MAVLink_system_time_message(1_800_000_000_000_000, 0),
        mavlink.MAVLink_rc_channels_message(0, 8, *([1500] * 18), 127),
    ):
        vehicle.handle(msg, 10.0)
    return vehicle


def test_packet_from_mavlink() -> None:
    packet = flying_vehicle().packet("WARG-01", 10.5)

    assert packet == {
        "uavId": "WARG-01",
        "unixTime": 1_800_000_000.5,
        "latitude": pytest.approx(50.1022871),
        "longitude": pytest.approx(-110.7373276),
        "altitudeAGL": 15.0,
        "horizontalPositionAccuracy": 0.8,
        "verticalPositionAccuracy": 1.5,
        "batteryPercentage": 87.0,
        "mode": "armed-automatic",
        "telemetryLinkStatus": 1.0,
        "rcLinkStatus": 0.5,
    }


def test_no_packet_before_first_heartbeat() -> None:
    vehicle = VehicleState()
    vehicle.handle(
        mavlink.MAVLink_global_position_int_message(0, 1, 1, 0, 0, 0, 0, 0, 0), 0.0
    )

    assert vehicle.packet("WARG-01", 0.0) is None


def no_fix(packet: dict) -> bool:
    return (packet["latitude"], packet["longitude"]) == (0.0, 0.0) and packet[
        "horizontalPositionAccuracy"
    ] == 9999.0


def test_heartbeat_without_position_sends_no_fix_packet() -> None:
    vehicle = VehicleState()
    vehicle.handle(heartbeat(armed=False), 0.0)

    packet = vehicle.packet("WARG-01", 0.0)

    assert packet["mode"] == "idle"
    assert no_fix(packet)


def test_lost_gps_fix_sends_no_fix_packet_not_last_position() -> None:
    vehicle = flying_vehicle()
    vehicle.handle(
        mavlink.MAVLink_gps_raw_int_message(0, 1, 0, 0, 0, 9999, 9999, 0, 0, 0), 10.2
    )

    packet = vehicle.packet("WARG-01", 10.5)

    assert packet["mode"] == "armed-automatic"
    assert no_fix(packet)


def test_stale_position_is_no_fix() -> None:
    vehicle = flying_vehicle()
    vehicle.handle(heartbeat(), 12.5)

    assert no_fix(vehicle.packet("WARG-01", 12.5))


def test_lost_heartbeat_reports_link_lost() -> None:
    packet = flying_vehicle().packet("WARG-01", 13.0)

    assert packet["mode"] == "link-lost"
    assert packet["telemetryLinkStatus"] == 0.0
    assert packet["rcLinkStatus"] == 0.0


def test_gcs_heartbeats_are_ignored() -> None:
    vehicle = flying_vehicle()
    gcs = mavlink.MAVLink_heartbeat_message(
        mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0, 3
    )
    vehicle.handle(gcs, 12.9)

    assert vehicle.packet("WARG-01", 13.0)["mode"] == "link-lost"


@pytest.mark.parametrize(
    ("kwargs", "mode"),
    [
        ({"link_ok": False, "armed": True, "mode": "GUIDED", "system_status": 4}, "link-lost"),
        ({"link_ok": True, "armed": True, "mode": "GUIDED", "system_status": 5}, "failsafe"),
        ({"link_ok": True, "armed": False, "mode": "GUIDED", "system_status": 3}, "idle"),
        ({"link_ok": True, "armed": True, "mode": "LAND", "system_status": 4}, "armed-automatic"),
        ({"link_ok": True, "armed": True, "mode": "LOITER", "system_status": 4}, "armed-pilot"),
    ],
)
def test_flight_mode(kwargs, mode) -> None:
    assert flight_mode(**kwargs) == mode


def test_pilot_mode_from_heartbeat() -> None:
    vehicle = flying_vehicle()
    vehicle.handle(heartbeat(custom_mode=LOITER), 10.0)

    assert vehicle.packet("WARG-01", 10.0)["mode"] == "armed-pilot"
