"""Mission finite-state machine core (ROS-independent: stdlib only).

States: EXPLORE -> RETURN -> LAND -> DONE, with HOLD on sensor/localization/
comms loss and FAILSAFE when a loss persists beyond its timeout (Goal 7).

Contract:
- ``tick(now, inputs) -> action`` where action is the command string for the
  current state (equal to ``.state``). ``inputs`` must contain ``sensors_ok``,
  ``loc_ok``, ``comms_ok``, ``mission_complete``, ``at_entry``.
- ``on_sensor_loss()`` / ``on_loc_loss()`` / ``on_comms_loss()`` stamp the loss
  at the last tick time so the next tick escalates immediately. ``tick`` also
  auto-stamps a loss when an ``*_ok`` input is False, and clears the stamp
  when the channel recovers (True).
- Any active loss -> HOLD; a loss persisting >= its per-channel timeout, or a
  continuous HOLD >= ``hold_timeout_s`` (backstop), -> FAILSAFE (terminal).
  Recovery from HOLD resumes the pre-HOLD state (EXPLORE or RETURN).
- ``mission_complete`` -> RETURN; ``at_entry`` during RETURN -> LAND; the tick
  after LAND -> DONE (terminal). LAND completes even if inputs degrade.
"""
from __future__ import annotations

from typing import Any

EXPLORE = "EXPLORE"
RETURN = "RETURN"
LAND = "LAND"
HOLD = "HOLD"
FAILSAFE = "FAILSAFE"
DONE = "DONE"

# Defaults mirror airmouse_autonomy/config/mission.yaml (fsm section).
DEFAULT_PARAMS: dict[str, float] = {
    "sensor_loss_timeout_s": 2.0,
    "loc_loss_timeout_s": 2.0,
    "comms_loss_timeout_s": 1.0,
    "hold_timeout_s": 10.0,
}

_REQUIRED_INPUTS = ("sensors_ok", "loc_ok", "comms_ok", "mission_complete", "at_entry")

# channel -> (inputs ok key, params timeout key)
_LOSS_CHANNELS = {
    "sensor": ("sensors_ok", "sensor_loss_timeout_s"),
    "loc": ("loc_ok", "loc_loss_timeout_s"),
    "comms": ("comms_ok", "comms_loss_timeout_s"),
}

_TERMINAL_STATES = (FAILSAFE, DONE)


class MissionFSM:
    """Loss-aware mission state machine; ``tick`` returns the action string."""

    def __init__(self, params_dict: dict[str, Any] | None = None) -> None:
        params = {**DEFAULT_PARAMS, **(params_dict or {})}
        self._timeouts = {
            key: _non_negative_float(params, key)
            for key in DEFAULT_PARAMS
        }
        self._state = EXPLORE
        self._last_now = 0.0
        self._loss_at: dict[str, float | None] = {ch: None for ch in _LOSS_CHANNELS}
        self._hold_since: float | None = None
        self._resume_state = EXPLORE

    @property
    def state(self) -> str:
        return self._state

    # ------------------------------------------------------- loss notifications

    def on_sensor_loss(self) -> None:
        """Stamp sensor loss at the last tick time (node notification hook)."""
        self._loss_at["sensor"] = self._last_now

    def on_loc_loss(self) -> None:
        """Stamp localization loss at the last tick time."""
        self._loss_at["loc"] = self._last_now

    def on_comms_loss(self) -> None:
        """Stamp comms loss at the last tick time."""
        self._loss_at["comms"] = self._last_now

    # ------------------------------------------------------------------- tick

    def tick(self, now: float, inputs_dict: dict[str, Any]) -> str:
        """Advance the FSM; return the action string (== ``.state``)."""
        now = _as_float(now, "now")
        inputs = _validate_inputs(inputs_dict)
        self._last_now = now
        if self._state in _TERMINAL_STATES:
            return self._state
        if self._state == LAND:
            self._state = DONE  # landing commanded last tick; complete now
            return self._state
        self._sync_losses(now, inputs)
        if any(stamp is not None for stamp in self._loss_at.values()):
            self._handle_active_losses(now)
            return self._state
        self._handle_nominal(inputs)
        return self._state

    # -------------------------------------------------------------- internals

    def _sync_losses(self, now: float, inputs: dict[str, bool]) -> None:
        for channel, (ok_key, _timeout_key) in _LOSS_CHANNELS.items():
            if inputs[ok_key]:
                self._loss_at[channel] = None  # channel recovered
            elif self._loss_at[channel] is None:
                self._loss_at[channel] = now  # auto-stamp unnotified loss

    def _handle_active_losses(self, now: float) -> None:
        if self._any_loss_expired(now) or self._hold_expired(now):
            self._state = FAILSAFE
            self._hold_since = None
            return
        if self._state != HOLD:
            self._resume_state = self._state
            self._hold_since = now
            self._state = HOLD

    def _any_loss_expired(self, now: float) -> bool:
        for channel, (_ok_key, timeout_key) in _LOSS_CHANNELS.items():
            stamp = self._loss_at[channel]
            if stamp is not None and now - stamp >= self._timeouts[timeout_key]:
                return True
        return False

    def _hold_expired(self, now: float) -> bool:
        return (
            self._state == HOLD
            and self._hold_since is not None
            and now - self._hold_since >= self._timeouts["hold_timeout_s"]
        )

    def _handle_nominal(self, inputs: dict[str, bool]) -> None:
        if self._state == HOLD:
            self._state = self._resume_state
            self._hold_since = None
        if self._state == EXPLORE and inputs["mission_complete"]:
            self._state = RETURN
        elif self._state == RETURN and inputs["at_entry"]:
            self._state = LAND


# ------------------------------------------------------------------ helpers


def _validate_inputs(inputs_dict: dict[str, Any]) -> dict[str, bool]:
    if not isinstance(inputs_dict, dict):
        raise ValueError(f"inputs must be a dict, got {inputs_dict!r}")
    missing = [key for key in _REQUIRED_INPUTS if key not in inputs_dict]
    if missing:
        raise ValueError(f"inputs missing required keys: {missing}")
    return {key: bool(inputs_dict[key]) for key in _REQUIRED_INPUTS}


def _as_float(value: Any, name: str) -> float:
    if isinstance(value, bool):
        raise ValueError(f"{name} must be a number, got {value!r}")
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a number, got {value!r}") from exc


def _non_negative_float(params: dict[str, Any], key: str) -> float:
    value = _as_float(params[key], key)
    if value < 0.0:
        raise ValueError(f"{key} must be >= 0, got {value}")
    return value