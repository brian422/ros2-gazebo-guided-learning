"""Pure finite-state-machine logic for the inspection mission."""

from dataclasses import dataclass
from enum import IntEnum


class MissionStateCode(IntEnum):
    """Numeric state codes shared with MissionState.msg."""

    IDLE = 0
    MOVING_TO_WAYPOINT = 1
    INSPECTING = 2
    HAZARD_DETECTED = 3
    RETREATING = 4
    MISSION_COMPLETE = 5
    ABORTED = 6


ACTIVE_STATES = {
    MissionStateCode.MOVING_TO_WAYPOINT,
    MissionStateCode.INSPECTING,
    MissionStateCode.HAZARD_DETECTED,
    MissionStateCode.RETREATING,
}


@dataclass(frozen=True)
class TransitionResult:
    """Describe whether an event caused an accepted state transition."""

    accepted: bool
    previous_state: MissionStateCode
    current_state: MissionStateCode
    reason: str


class MissionStateMachine:
    """Readable mission-state model independent of ROS middleware."""

    def __init__(self) -> None:
        """Start in a stopped IDLE state."""
        self._state = MissionStateCode.IDLE
        self._waypoint_index = 0
        self._detail = 'Waiting for a start request.'

    @property
    def state(self) -> MissionStateCode:
        """Return the current state."""
        return self._state

    @property
    def waypoint_index(self) -> int:
        """Return the current zero-based waypoint index."""
        return self._waypoint_index

    @property
    def detail(self) -> str:
        """Return the explanation associated with the latest transition."""
        return self._detail

    @property
    def is_active(self) -> bool:
        """Return True while the mission requires live safety data."""
        return self._state in ACTIVE_STATES

    def request_start(self) -> TransitionResult:
        """Start a mission only from IDLE."""
        if self._state != MissionStateCode.IDLE:
            return self._reject(
                'Start is accepted only while the mission is IDLE.'
            )
        self._waypoint_index = 0
        return self._transition(
            MissionStateCode.MOVING_TO_WAYPOINT,
            'Mission started; moving toward waypoint 0.',
        )

    def request_reset(self) -> TransitionResult:
        """Return any state to IDLE and clear waypoint progress."""
        self._waypoint_index = 0
        return self._transition(
            MissionStateCode.IDLE,
            'Mission reset; waiting for a start request.',
        )

    def request_abort(
        self,
        reason: str = 'Operator requested abort.',
    ) -> TransitionResult:
        """Stop an unfinished mission in ABORTED."""
        if self._state == MissionStateCode.ABORTED:
            return self._reject('Mission is already ABORTED.')
        if self._state == MissionStateCode.MISSION_COMPLETE:
            return self._reject(
                'Completed mission must be reset before another command.'
            )
        return self._transition(MissionStateCode.ABORTED, reason)

    def report_hazard(self, hazard_value: float) -> TransitionResult:
        """Enter HAZARD_DETECTED from a motion or inspection state."""
        if self._state not in {
            MissionStateCode.MOVING_TO_WAYPOINT,
            MissionStateCode.INSPECTING,
        }:
            return self._reject(
                'Hazard transition requires motion or inspection state.'
            )
        return self._transition(
            MissionStateCode.HAZARD_DETECTED,
            f'Hazard threshold exceeded: measured {hazard_value:.3f}.',
        )

    def report_waypoint_reached(self) -> TransitionResult:
        """Enter INSPECTING when the active waypoint is reached."""
        if self._state != MissionStateCode.MOVING_TO_WAYPOINT:
            return self._reject(
                'Waypoint arrival requires MOVING_TO_WAYPOINT.'
            )
        return self._transition(
            MissionStateCode.INSPECTING,
            f'Waypoint {self._waypoint_index} reached; inspecting.',
        )

    def report_transit_waypoint_reached(
        self,
        more_waypoints: bool,
    ) -> TransitionResult:
        """Advance through a route point that requires no inspection."""
        if self._state != MissionStateCode.MOVING_TO_WAYPOINT:
            return self._reject(
                'Transit arrival requires MOVING_TO_WAYPOINT.'
            )
        if not more_waypoints:
            return self._transition(
                MissionStateCode.MISSION_COMPLETE,
                'Final transit waypoint reached.',
            )

        reached_index = self._waypoint_index
        self._waypoint_index += 1
        return self._transition(
            MissionStateCode.MOVING_TO_WAYPOINT,
            f'Transit waypoint {reached_index} reached; moving toward '
            f'waypoint {self._waypoint_index}.',
        )

    def report_inspection_complete(
        self,
        more_waypoints: bool,
    ) -> TransitionResult:
        """Continue to the next waypoint or complete the mission."""
        if self._state != MissionStateCode.INSPECTING:
            return self._reject(
                'Inspection completion requires INSPECTING.'
            )
        if more_waypoints:
            self._waypoint_index += 1
            return self._transition(
                MissionStateCode.MOVING_TO_WAYPOINT,
                f'Moving toward waypoint {self._waypoint_index}.',
            )
        return self._transition(
            MissionStateCode.MISSION_COMPLETE,
            'Final inspection completed.',
        )

    def begin_retreat(self) -> TransitionResult:
        """Enter RETREATING after the stop action has been established."""
        if self._state != MissionStateCode.HAZARD_DETECTED:
            return self._reject(
                'Retreat requires HAZARD_DETECTED.'
            )
        return self._transition(
            MissionStateCode.RETREATING,
            'Retreat initiated toward the previous safe waypoint.',
        )

    def report_retreat_complete(self) -> TransitionResult:
        """End a safety retreat in ABORTED pending operator reset."""
        if self._state != MissionStateCode.RETREATING:
            return self._reject(
                'Retreat completion requires RETREATING.'
            )
        return self._transition(
            MissionStateCode.ABORTED,
            'Safe waypoint reached; mission remains aborted.',
        )

    def _transition(
        self,
        new_state: MissionStateCode,
        reason: str,
    ) -> TransitionResult:
        """Apply an accepted transition and return its record."""
        previous_state = self._state
        self._state = new_state
        self._detail = reason
        return TransitionResult(
            accepted=True,
            previous_state=previous_state,
            current_state=self._state,
            reason=reason,
        )

    def _reject(self, reason: str) -> TransitionResult:
        """Return a rejected event without changing state."""
        return TransitionResult(
            accepted=False,
            previous_state=self._state,
            current_state=self._state,
            reason=reason,
        )
