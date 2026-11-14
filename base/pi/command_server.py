"""The Jetson's commands for run_follow.py --remote: a TCP server on the Pi, JSON lines.

One client at a time; a new connection replaces the old one, after the lines the old one already
sent are read. Only a sender on the allow list (the Jetson's addresses) may connect: anyone else on
the LAN could drive the robot, or keep the silence hold away with pings. Others are closed at once,
never replace the client, and are logged once a minute per address. Every line is one UTF-8 JSON object ending in "\\n". Client -> Pi: {"cmd": NAME},
NAME one of COMMANDS. Pi -> client: the status objects run_follow sends, and {"error": "..."} for a
line that is not a command (it is ignored) or one longer than MAX_LINE (dropped whole).

Nothing here waits: poll() is called once per control cycle and takes only what has already
arrived (select with timeout 0, non-blocking sockets). No thread, so nothing is left at exit.
Nothing from the network escapes poll() or send(): a client whose socket fails is dropped, and
the reason goes to `events` for the log.
"""
import contextlib
import json
import select
import socket

PORT = 5604
COMMANDS = ("follow", "wait", "resume", "return", "stop", "ping", "bag", "bag_again")  # bag: run_follow's BAG phase
MAX_LINE = 4096  # bytes; a longer line is not a command
MAX_READ = 16 * MAX_LINE  # bytes taken in one poll; the rest waits for the next cycle
MAX_UNSENT = 65536  # bytes of status waiting for a client that does not read: drop that client
LOOPBACK = ("127.0.0.1",)  # the allow list when none is given: this computer only
REFUSED_LOG_EVERY = 60.0  # s between log lines for one refused address


class CommandServer:
    def __init__(self, host="0.0.0.0", port=PORT, allow=LOOPBACK):
        self.host, self.port = host, port
        self.allow = tuple(allow)  # the addresses that may connect
        self.refused = {}  # address -> cycle time of its last log line
        self.listener = self.client = None
        self.inbox = self.outbox = b""
        self.skipping = False  # the rest of a line cut at MAX_LINE: discarded up to its newline
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
        lines = []
        try:
            lines += self._lines()  # the current client first: a replaced client's last lines still count
            lines += self._accept(now)
            lines += self._lines()
        except Exception as error:  # a bug here must not stop the cycle (the handlers below catch the network)
            self._broken("poll failed: %s: %s" % (type(error).__name__, error))
        commands = []
        for line in lines:
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

    def _accept(self, now):
        """Take the waiting connections from the allow list. Returns the lines of each client replaced on the way."""
        lines = []
        while True:
            try:
                if not select.select([self.listener], [], [], 0)[0]:
                    break
                client, address = self.listener.accept()
                client.setblocking(False)
            except Exception as error:  # gone again before accept (BlockingIOError, ECONNABORTED)
                self.events.append("accept failed: %s: %s" % (type(error).__name__, error))
                break
            if address[0] not in self.allow:
                with contextlib.suppress(Exception):
                    client.close()
                if now - self.refused.get(address[0], -REFUSED_LOG_EVERY) >= REFUSED_LOG_EVERY:
                    self.refused[address[0]] = now
                    self.events.append("refused %s: not on the allow list" % address[0])
                continue
            if self.client is not None:
                lines += self._lines()
                if self.client is not None:
                    self._drop("replaced by a new client %s:%d" % address[:2])
            self.client, self.inbox, self.outbox, self.skipping = client, b"", b"", False
            self.events.append("client %s:%d connected" % address[:2])
        return lines

    def _lines(self):
        """The complete lines received so far (a client that just closed keeps its last lines)."""
        while self.client is not None and len(self.inbox) < MAX_READ:
            try:
                chunk = self.client.recv(4096)
            except (BlockingIOError, InterruptedError):
                break
            except Exception as error:
                self._drop("receive failed: %s" % error)
                break
            if not chunk:
                self._drop("closed by the client")
                break
            self.inbox += chunk
        lines = []
        while True:
            if self.skipping:
                cut = self.inbox.find(b"\n")
                if cut < 0:
                    self.inbox = b""
                    break
                self.inbox, self.skipping = self.inbox[cut + 1:], False
            cut = self.inbox.find(b"\n")
            if cut < 0:
                if len(self.inbox) > MAX_LINE:
                    self.inbox, self.skipping = b"", True
                    self._error("a line longer than %d bytes" % MAX_LINE)
                break
            line, self.inbox = self.inbox[:cut], self.inbox[cut + 1:]
            if len(line) > MAX_LINE:
                self._error("a line longer than %d bytes" % MAX_LINE)
            else:
                lines.append(line.strip())
        return lines

    def _parse(self, line):
        """The command name in line, or None after an error reply."""
        try:
            message = json.loads(line.decode("utf-8"))
        except Exception:  # not UTF-8, not JSON, or nested too deep (RecursionError)
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
            except Exception as error:
                self._drop("send failed: %s" % error)
                return
            self.outbox = self.outbox[sent:]
        if len(self.outbox) > MAX_UNSENT:
            self._drop("not reading its status")

    def _broken(self, why):
        if self.client is not None:
            self._drop(why)
        else:
            self.events.append(why)

    def _drop(self, why):
        with contextlib.suppress(Exception):
            self.client.close()
        self.client, self.outbox = None, b""  # the inbox stays: its complete lines still count
        self.events.append(why)
