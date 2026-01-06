#!/usr/bin/env python3
"""Minimal Linmot state machine based on status word bits.

Usage example:
    sm = LinmotStateMachine()
    status_val = 0x3f41  # sample status word from drive
    state = sm.update(status_val)
    cmd = sm.next_command()
"""

import time
from enum import Enum
from .linmot_controller import decode_status_word, LinmotCommand


class LinmotState(Enum):
    IDLE = 0
    READY = 1
    HOMING = 2
    MOVING = 3
    IN_POSITION = 4
    ERROR = 5
    FATAL = 6


class LinmotStateMachine:
    def __init__(self):
        self.state = LinmotState.IDLE
        self.last_flags = None

    def update(self, status_word_value: int) -> LinmotState:
        """Update state from raw status word value and return new state."""
        flags = decode_status_word(status_word_value)
        self.last_flags = flags

        if flags["fatalError"]:
            self.state = LinmotState.FATAL
        elif flags["error"]:
            self.state = LinmotState.ERROR
        elif flags["motionActive"]:
            self.state = LinmotState.MOVING
        elif flags["inTargetPosition"]:
            self.state = LinmotState.IN_POSITION
        elif flags["operationEnabled"]:
            self.state = LinmotState.READY
        else:
            self.state = LinmotState.IDLE

        # Simple homing detection: special motion flag maps to HOMING state
        if flags["specialMotionActive"]:
            self.state = LinmotState.HOMING

        return self.state

    def next_command(self) -> LinmotCommand | None:
        """Suggest next command based on current state.

        Returns a LinmotCommand or None if no action is suggested.
        """
        if self.state in (LinmotState.FATAL, LinmotState.ERROR):
            return LinmotCommand.SWITCH_OFF
        if self.state == LinmotState.IDLE:
            return LinmotCommand.OPERATIONAL_MODE
        if self.state == LinmotState.READY:
            return LinmotCommand.HOMING
        if self.state == LinmotState.HOMING:
            return LinmotCommand.GET_POSITION
        if self.state == LinmotState.MOVING:
            return None  # wait
        if self.state == LinmotState.IN_POSITION:
            return LinmotCommand.GET_POSITION
        return None

    def status_summary(self) -> str:
        """Return a short human-readable summary of flags."""
        if not self.last_flags:
            return "No status yet"
        f = self.last_flags
        return (
            f"state={self.state.name}, "
            f"enabled={f['operationEnabled']}, in_pos={f['inTargetPosition']}, "
            f"moving={f['motionActive']}, homed={f['homed']}, "
            f"error={f['error']}, fatal={f['fatalError']}"
        )


def bringup_sequence(read_status_word, send_command,
                    poll_interval: float = 0.2,
                    ready_timeout: float = 5.0,
                    homing_timeout: float = 20.0):
    """Run a safe bring-up sequence: Operational → Switch On → Home.

    Args:
        read_status_word: callable that returns the latest status word (int) or None.
        send_command: callable that accepts a LinmotCommand and sends it.
        poll_interval: seconds between status polls.
        ready_timeout: max seconds to wait for READY after switch on.
        homing_timeout: max seconds to wait for HOMED after homing.

    Returns:
        (state, flags) on success, or (None, None) on timeout/failure.
    """

    sm = LinmotStateMachine()

    def wait_for(predicate, timeout):
        start = time.time()
        while time.time() - start < timeout:
            sw = read_status_word()
            if sw is None:
                time.sleep(poll_interval)
                continue
            state = sm.update(sw)
            if predicate(sm.last_flags, state):
                return state, sm.last_flags
            time.sleep(poll_interval)
        return None, None

    # 1) Operational mode
    send_command(LinmotCommand.OPERATIONAL_MODE)

    # 2) Switch on (power bridge)
    send_command(LinmotCommand.SWITCH_ON)
    state, flags = wait_for(lambda f, s: f and f["operationEnabled"], ready_timeout)
    if not state:
        return None, None

    # 3) Home
    send_command(LinmotCommand.HOMING)
    state, flags = wait_for(
        lambda f, s: f and f["homed"] and f["inTargetPosition"] and not f["motionActive"],
        homing_timeout,
    )
    if not state:
        return None, None

    return state, flags
