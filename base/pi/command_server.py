"""The Jetson's commands for run_follow.py --remote: a TCP server on the Pi, JSON lines.

One client at a time; a new connection replaces the old one. Every line is one UTF-8 JSON object
ending in "\\n". Client -> Pi: {"cmd": NAME}, NAME one of COMMANDS. Pi -> client: the status
objects run_follow sends, and {"error": "..."} for a line that is not a command (it is ignored).

Nothing here waits: poll() is called once per control cycle and takes only what has already
arrived (select with timeout 0, non-blocking sockets). No thread, so nothing is left at exit.
"""
import json
import select
import socket

PORT = 5604
COMMANDS = ("follow", "wait", "resume", "return", "stop", "ping")
MAX_LINE = 4096  # bytes; a longer line is not a command
MAX_READ = 16 * MAX_LINE  # bytes taken in one poll; the rest waits for the next cycle
MAX_UNSENT = 65536  # bytes of status waiting for a client that does not read: drop that client


class CommandServer:
    def __init__(self, host="0.0.0.0", port=PORT):
        self.host, self.port = host, port
        self.listener = self.client = None
        self.inbox = self.outbox = b""
        self.heard = None  # cycle time of the last command line (ping too), or of the first poll
        self.events = []  # what happened to the connection, for the log; the caller empties it

    def start(self):
        self.listener = socket.create_server((self.host, self.port))
        self.listener.setblocking(False)
        self.port = self.listener.getsockname()[1]  # the real port when 0 was asked

    def poll(self, now):
        """The command names that arrived since the last poll, in order. Never waits."""
        if self.heard is None:
            self.heard = now
        self._accept()
        commands = []
        for line in self._lines():
            cmd = self._parse(line)
            if cmd is not None:
                self.heard = now
                commands.append(cmd)
        self._flush()
        return commands

    def silent(self, now):
        """Seconds since the last command line."""
        return 0.0 if self.heard is None else now - self.heard

    def send(self, message):
        if self.client is None:
            return
        self.outbox += json.dumps(message).encode("utf-8") + b"\n"
        self._flush()

    def stop(self):
        if self.client is not None:
            self._drop("run ended")
        if self.listener is not None:
            self.listener.close()

    def _accept(self):
        while select.select([self.listener], [], [], 0)[0]:
            try:
                client, address = self.listener.accept()
            except OSError:  # gone again before accept (BlockingIOError, ECONNABORTED)
                break
            if self.client is not None:
                self._drop("replaced by a new client %s:%d" % address[:2])
            client.setblocking(False)
            self.client, self.inbox, self.outbox = client, b"", b""
            self.events.append("client %s:%d connected" % address[:2])

    def _lines(self):
        """The complete lines received so far (a client that just closed keeps its last lines)."""
        while self.client is not None and len(self.inbox) < MAX_READ:
            try:
                chunk = self.client.recv(4096)
            except (BlockingIOError, InterruptedError):
                break
            except OSError as error:
                self._drop("receive failed: %s" % error)
                break
            if not chunk:
                self._drop("closed by the client")
                break
            self.inbox += chunk
        lines = self.inbox.split(b"\n")
        self.inbox = lines.pop()
        if len(self.inbox) > MAX_LINE:
            self.inbox = b""
            self._error("a line longer than %d bytes" % MAX_LINE)
        return [line.strip() for line in lines]

    def _parse(self, line):
        """The command name in line, or None after an error reply."""
        try:
            message = json.loads(line.decode("utf-8"))
        except ValueError:  # UnicodeDecodeError is a ValueError too
            return self._error("not JSON: %r" % line[:80])
        if not isinstance(message, dict) or message.get("cmd") not in COMMANDS:
            return self._error("not a command (%s): %r" % (", ".join(COMMANDS), line[:80]))
        return message["cmd"]

    def _error(self, text):
        self.send({"error": text})
        return None

    def _flush(self):
        while self.client is not None and self.outbox:
            try:
                sent = self.client.send(self.outbox)
            except (BlockingIOError, InterruptedError):
                break
            except OSError as error:
                self._drop("send failed: %s" % error)
                return
            self.outbox = self.outbox[sent:]
        if len(self.outbox) > MAX_UNSENT:
            self._drop("not reading its status")

    def _drop(self, why):
        self.client.close()
        self.client, self.outbox = None, b""  # the inbox stays: its complete lines still count
        self.events.append(why)
