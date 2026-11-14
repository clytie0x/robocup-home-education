"""Human-robot interaction. For now this is only a log of what the robot says."""

READY_TEXT = "I am ready to follow you."
# The rulebook: the referees tell the operator to follow the robot's instructions.
WAIT_TEXT = "Please wait for me."
GO_ON_TEXT = "Thank you, please go on."
COME_BACK_TEXT = "I lost you. Please come back and stand in front of me."
# The way back (route.py).
RETURN_TEXT = "I will go back to the start now."
PASS_TEXT = "Please let me pass."
ARRIVED_TEXT = "I am back at the start."
_SLOW_START = "Please walk slowly, at about "
_SLOW_END = " metres per second, so I can follow you."


def walk_slowly_text(speed: float) -> str:
    """The sentence that asks the operator to walk at `speed` m/s."""
    return f"{_SLOW_START}{round(speed, 2):g}{_SLOW_END}"


def asked_pace(text: str) -> float | None:
    """The speed a walk-slowly sentence names, as the operator hears it, or None."""
    if text.startswith(_SLOW_START) and text.endswith(_SLOW_END):
        return float(text[len(_SLOW_START) : -len(_SLOW_END)])
    return None


class Speech:
    def __init__(self) -> None:
        self.messages: list[tuple[float, str]] = []  # (time, text)

    def say(self, time: float, text: str) -> None:
        self.messages.append((time, text))

    def has_said(self, text: str) -> bool:
        return any(message == text for _, message in self.messages)
