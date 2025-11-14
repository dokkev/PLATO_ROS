"""SpaceMouse Hardware Interface Module.

This module provides a clean hardware abstraction for the SpaceMouse device.
It handles all the low-level pyspacemouse interactions.
"""

import pyspacemouse
from typing import Optional, List, Tuple


class SpaceMouseState:
    """Data class representing the current state of the SpaceMouse."""

    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0,
                 roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0,
                 buttons: List[bool] = None):
        self.x = x
        self.y = y
        self.z = z
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.buttons = buttons if buttons is not None else [False, False]


class SpaceMouseHardware:
    """Hardware interface for SpaceMouse device.

    This class encapsulates all interactions with the pyspacemouse library,
    providing a clean, testable interface for ROS nodes.
    """

    def __init__(self):
        """Initialize the SpaceMouse hardware interface."""
        self._is_open = False
        self._prev_button_states = [False, False]

    def open(self) -> bool:
        """Open connection to the SpaceMouse device.

        Returns:
            bool: True if successful, False otherwise.
        """
        try:
            self._is_open = pyspacemouse.open()
            return self._is_open
        except Exception as e:
            print(f"Error opening SpaceMouse: {e}")
            return False

    def close(self):
        """Close connection to the SpaceMouse device."""
        if self._is_open:
            try:
                pyspacemouse.close()
                self._is_open = False
            except Exception as e:
                print(f"Error closing SpaceMouse: {e}")

    def read(self) -> Optional[SpaceMouseState]:
        """Read the current state from the SpaceMouse.

        Returns:
            SpaceMouseState or None: Current device state, or None if read failed.
        """
        if not self._is_open:
            return None

        try:
            raw_state = pyspacemouse.read()
            if raw_state is None:
                return None

            # Create our clean state object
            state = SpaceMouseState(
                x=float(raw_state.x),
                y=float(raw_state.y),
                z=float(raw_state.z),
                roll=float(raw_state.roll),
                pitch=float(raw_state.pitch),
                yaw=float(raw_state.yaw),
                buttons=raw_state.buttons.copy() if hasattr(raw_state, 'buttons') else [False, False]
            )

            return state
        except Exception as e:
            print(f"Error reading SpaceMouse: {e}")
            return None

    def get_button_transitions(self, current_buttons: List[bool]) -> Tuple[List[bool], List[bool]]:
        """Detect button press and release transitions.

        Args:
            current_buttons: Current button states.

        Returns:
            Tuple of (pressed, released) lists indicating which buttons changed state.
        """
        pressed = []
        released = []

        for i, (prev, curr) in enumerate(zip(self._prev_button_states, current_buttons)):
            if not prev and curr:
                pressed.append(i)
            elif prev and not curr:
                released.append(i)

        self._prev_button_states = current_buttons.copy()
        return pressed, released

    def is_open(self) -> bool:
        """Check if the device is currently open.

        Returns:
            bool: True if device is open, False otherwise.
        """
        return self._is_open
