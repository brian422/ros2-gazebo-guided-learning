"""Unit tests for deterministic inspection-mission state transitions."""

from hazard_inspection_control.mission_state_machine import (
    MissionStateCode,
    MissionStateMachine,
)


def test_new_mission_starts_idle() -> None:
    """Construction must produce a known, stopped initial state."""
    machine = MissionStateMachine()

    assert machine.state == MissionStateCode.IDLE
    assert machine.waypoint_index == 0
    assert not machine.is_active


def test_start_enters_waypoint_motion() -> None:
    """A valid start request activates waypoint zero."""
    machine = MissionStateMachine()

    result = machine.request_start()

    assert result.accepted
    assert machine.state == MissionStateCode.MOVING_TO_WAYPOINT
    assert machine.waypoint_index == 0


def test_hazard_threshold_event_changes_state() -> None:
    """A reported threshold crossing must interrupt normal movement."""
    machine = MissionStateMachine()
    machine.request_start()

    result = machine.report_hazard(6.25)

    assert result.accepted
    assert result.previous_state == MissionStateCode.MOVING_TO_WAYPOINT
    assert machine.state == MissionStateCode.HAZARD_DETECTED
    assert '6.250' in machine.detail


def test_hazard_retreat_ends_aborted() -> None:
    """The educational safety sequence ends stopped pending reset."""
    machine = MissionStateMachine()
    machine.request_start()
    machine.report_hazard(7.0)

    assert machine.begin_retreat().accepted
    assert machine.state == MissionStateCode.RETREATING
    assert machine.report_retreat_complete().accepted
    assert machine.state == MissionStateCode.ABORTED


def test_invalid_sensor_abort_reaches_safe_state() -> None:
    """The ROS node can route invalid input through an explicit abort event."""
    machine = MissionStateMachine()
    machine.request_start()

    result = machine.request_abort(
        'Mission aborted because hazard data is invalid.'
    )

    assert result.accepted
    assert machine.state == MissionStateCode.ABORTED
    assert 'invalid' in machine.detail


def test_transit_waypoint_advances_without_inspection() -> None:
    """A non-final transit point advances the route while staying in motion."""
    machine = MissionStateMachine()
    machine.request_start()

    result = machine.report_transit_waypoint_reached(more_waypoints=True)

    assert result.accepted
    assert machine.state == MissionStateCode.MOVING_TO_WAYPOINT
    assert machine.waypoint_index == 1


def test_final_inspection_completes_mission() -> None:
    """A final inspection must terminate in MISSION_COMPLETE."""
    machine = MissionStateMachine()
    machine.request_start()
    machine.report_waypoint_reached()

    result = machine.report_inspection_complete(more_waypoints=False)

    assert result.accepted
    assert machine.state == MissionStateCode.MISSION_COMPLETE


def test_start_is_rejected_when_already_active() -> None:
    """Repeated start commands must not silently restart route progress."""
    machine = MissionStateMachine()
    machine.request_start()

    result = machine.request_start()

    assert not result.accepted
    assert machine.state == MissionStateCode.MOVING_TO_WAYPOINT
