"""Human-robot interaction. For now this is only a log of what the robot says."""

READY_TEXT = "I am ready to follow you."


class Speech:
    def __init__(self) -> None:
        self.messages: list[tuple[float, str]] = []  # (time, text)

    def say(self, time: float, text: str) -> None:
        self.messages.append((time, text))

    def has_said(self, text: str) -> bool:
        return any(message == text for _, message in self.messages)
