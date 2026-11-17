"""RuleParser: keyword/regex brain, the /speech/text -> /robot/intent part of rule_node (no ROS here).

Kept apart from rule_node.py so it can be tested on a PC without rclpy. rule_node.py holds only the node
and the topics. Only understands the sentence patterns below.

  stop / halt / freeze / wait                          -> stop
  beatbox / drop a beat / play music                   -> beatbox (plays beatbox_file)
  drop / release / put ... down / let go                -> drop_bag
  carry / pick up / take / hold ... bag                 -> carry_bag
  follow me / come with me                              -> follow_me
  turn / rotate left|right [N degrees] / around         -> turn  (left +, right -, default 90, around 180)
  go|move back / backward / reverse [N m]               -> move_backward (default 0.5 m)
  go|move forward / ahead / straight [N m]              -> move_forward  (default 0.5 m)
  go to <place> / look at <thing>                       -> go_to / look_at
  hello, thanks, what can you do                        -> chat replies
Numbers: digits ("1.5") or words ("one", "twenty five", "half", "a little"); units m / cm / degrees.

Help Me Carry (task_words, switched on by launch/hmc_voice.launch.py) adds the words that the task flow
base/jetson/hmc_flow.py acts on (its INTENT_KINDS: follow_me, stop/wait, go_back, resume, and carry_bag
for its "done"): "this is the car", "go back to the start", "take the bag back", "continue", and "ready" /
"done" / "okay" at the car.
"""
import re
from difflib import SequenceMatcher

UNITS = {"zero": 0, "one": 1, "a": 1, "an": 1, "two": 2, "three": 3, "four": 4, "five": 5, "six": 6,
         "seven": 7, "eight": 8, "nine": 9, "ten": 10, "eleven": 11, "twelve": 12, "fifteen": 15,
         "twenty": 20, "thirty": 30, "forty": 40, "forty-five": 45, "fifty": 50, "sixty": 60,
         "seventy": 70, "eighty": 80, "ninety": 90, "hundred": 100}

NEG = re.compile(r"\b(don'?t|do not|never|not now)\b", re.I)
R = lambda p: re.compile(p, re.I)  # noqa: E731
# Whole-sentence sound-alikes of a lone "stop" that whisper produced on the real robot
# ("Tap top.", "Ta-ta.", "Stab.") + natural ways to say stop. Only matched when that is ALL that was said.
STOP_ALIKE = R(r"^(?:(?:please|robot|hey|okay|ok)\s+)*(?:stop+s?|stops? it|stopped|stopping|stab|stob|stup|top|tops|"
               r"tap ?tops?|stap|stuff|ta[- ]?ta|tata|that'?s (?:all|it|enough)|that is (?:all|it|enough)|"
               r"enough|wait|hold on|stay|stay there|stay here|"
               r"finish(?:ed)?|finish (?:it|that|the task)|pause|pause (?:it|that)|end|end (?:it|that|the task)|the end)"
               r"(?:\s+(?:please|now|robot|here))*$")
# finish / pause / end count as stop only when they are the whole sentence ("go to the end of the hall" is not stop)
# Fuzzy fallback for the few commands the robot really needs. Whisper often writes them slightly wrong
# ("Dopting the bag", "Hollow me", "Keep up the bag"); if no rule matched, a SHORT sentence that sounds
# close enough to exactly one of these phrases is taken as that command.
COMMAND_PHRASES = {
    "carry_bag": ["help me carry this bag", "help me carry the bag", "carry this bag", "carry the bag",
                  "carry my bag", "pick up the bag", "pick up this bag", "take the bag", "take this bag",
                  "hold the bag", "grab the bag", "keep up the bag", "beat up the bag"],
    "drop_bag": ["drop the bag", "drop this bag", "drop it", "put the bag down", "put down the bag",
                 "release the bag", "let go of the bag"],
    "follow_me": ["follow me", "follow me please", "come with me", "please follow me"],
    "stop": ["stop", "robot stop", "please stop", "stop here", "stay here", "stay there", "wait here", "stay",
             "finish", "pause", "end", "robot finish", "robot pause"],
}


def _norm(s):
    return " ".join(re.sub(r"[^a-z ]", " ", s.lower().replace("'", "")).split())


def fuzzy_command(text, allowed=None, min_ratio=0.75, margin=0.1, max_words=7):
    """(action, score) for the best phrase if score >= min_ratio and clearly ahead of the 2nd-best
    command (margin), else (None, score). Whole text and each sentence of it are tried."""
    best = (None, 0.0, 0.0)
    for c in [text] + re.split(r"[.!?,]+", text):
        t = _norm(c)
        if not t or len(t.split()) > max_words:
            continue
        sc = sorted(((a, max(SequenceMatcher(None, t, _norm(p)).ratio() for p in ph))
                     for a, ph in COMMAND_PHRASES.items() if allowed is None or a in allowed),
                    key=lambda kv: -kv[1])
        if sc and sc[0][1] > best[1]:
            best = (sc[0][0], sc[0][1], sc[1][1] if len(sc) > 1 else 0.0)
    a, s1, s2 = best
    return (a, s1) if a and s1 >= min_ratio and s1 - s2 >= margin else (None, s1)


FUZZY_REPLY = {"stop": "Stopping.", "carry_bag": "Sure, I will carry the bag for you.",
               "drop_bag": "Okay, dropping the bag.", "follow_me": "Okay, I will follow you."}

STOP_RX = R(r"\b(stop|halt|freeze|stay still|stay here|stay there|wait here|wait for me|don'?t move)\b")

RULES = [
    ("stop", STOP_RX),
    ("beatbox", R(r"\b(beat|bit|beet|beats)[\s-]?box(ing)?\b|\bbeatbox|\b(drop|play)\s+(a|the|some)\s+beats?\b|\bplay\s+(some\s+)?music\b")),   # before drop_bag: "drop a beat" is not "drop the bag"
    ("drop_bag", R(r"\b(drop|release|unload|let\s+go)\b|\bput\b.*\bdown\b")),
    ("carry_bag", R(r"\bcarry\b|\b(pick\s+up|hold|take|grab)\b.*\bbags?\b")),
    ("follow_me", R(r"\bfollow\b|\bcome with me\b")),
    ("turn", R(r"\b(turn|rotate|spin)\b")),
    ("move_backward", R(r"\b(go|move|drive|walk|step)\s+back(ward|wards)?\b|\bback\s+up\b|\breverse\b|\bbackward")),
    ("move_forward", R(r"\b(go|move|drive|walk|step)\s+(forward|forwards|ahead|straight)\b|\bforward\b|\bahead\b")),
    ("go_to", R(r"\bgo\s+to\s+(?:the\s+)?(?P<target>[a-z ]+?)\s*(?:please)?[.!?]*$")),
    ("look_at", R(r"\blook\s+at\s+(?:the\s+)?(?P<target>[a-z ]+?)\s*(?:please)?[.!?]*$")),
    ("name", R(r"what'?s your name|what is your name|who are you|introduce yourself")),
    ("caps", R(r"what (can|do|else can) you do|how can you help|your (skills|abilities)")),
    ("thanks", R(r"\b(thank you|thanks)\b")),
    ("hello", R(r"\b(hi|hello|hey|good (morning|afternoon|evening))\b")),
]

# Help Me Carry task words. hmc_flow.py classifies the same words in the raw sentence; these are its
# intents (INTENT_KINDS there), so a sentence the flow passed on still means the same thing here.
# Only with task_words on: in the generic voice launch "take the bag back" must still pick the bag up.
TASK_STOP = R(r"\b(?:this|that)(?:'s| is) (?:the|my|your) car\b|\b(?:arrived at the car|"
              r"we(?:'re| are) (?:at|by) the car|we(?:'re| are) here|here we are)\b")
TASK_WORDS = [
    ("go_back", R(r"\b(?:go|take it|take the bag|bring it|bring the bag|come) back\b|\bgo to the start\b|"
                  r"\bgo home\b|\breturn\b|\bhead back\b")),
    ("resume", R(r"\b(?:continue|resume|keep going|go on|carry on)\b")),
]
# at_car the flow asks "point at the bag, then say ready": its word for "go on with the bag" is carry_bag
# (INTENT_KINDS maps carry_bag to its "done")
TASK_ACK = {"ready", "done", "okay", "ok", "got it", "go ahead"}
TASK_REPLY = {"go_back": "Okay, I will go back to the start.", "resume": "Okay, going on.",
              "carry_bag": "Okay, I will take the bag."}


def _words_to_number(s):
    """'twenty five' -> 25, 'one hundred eighty' -> 180, 'half' -> 0.5. None if no number."""
    total, found = 0.0, False
    for w in re.findall(r"[a-z\-]+", s.lower()):
        if w == "hundred":
            total = (total or 1) * 100
            found = True
        elif w in UNITS and w not in ("a", "an"):
            total += UNITS[w]
            found = True
        elif w in ("a", "an", "and"):
            continue
        elif found:
            break
    return total if found else None


def parse_number(text, unit_rx):
    """Find '<number> <unit>' in text. Returns float or None."""
    m = re.search(rf"(\d+(?:\.\d+)?)\s*{unit_rx}", text, re.I)
    if m:
        return float(m.group(1))
    m = re.search(rf"((?:[a-z\-]+\s+){{0,4}}?)(?:and\s+a\s+half\s+)?{unit_rx}", text, re.I)
    if m:
        n = _words_to_number(m.group(1))
        if n is not None:
            if re.search(rf"and\s+a\s+half\s+{unit_rx}", text, re.I):
                n += 0.5
            return n
    return None


def clause_stops(clause):
    """One clause of a sentence asks the robot to stop. A negation in the same clause cancels it
    ("don't stop"), except "don't move", which always meant "stand still" here."""
    c = " ".join(re.sub(r"[^a-z' -]", " ", clause).split())
    if STOP_ALIKE.match(c):
        return True
    if not STOP_RX.search(c):
        return False
    return not NEG.search(c) or re.search(r"don'?t move|do not move", c) is not None


class RuleParser:
    def __init__(self, capabilities, allowed, robot_name="Predictably Slightly Unpredictable"):
        self.capabilities = capabilities
        self.name = robot_name
        self.allowed = set(allowed)
        self.fuzzy = True            # sound-alike fallback for the core commands (see COMMAND_PHRASES)
        self.fuzzy_min = 0.75
        self.task_words = False      # Help Me Carry words (see TASK_WORDS); launch/hmc_voice turns it on
        self.last_fuzzy = None

    def parse(self, text):
        """Returns (action, params, reply). action 'none' = nothing to send to the robot."""
        t = text.strip()
        low = t.lower()
        bare = " ".join(re.sub(r"[^a-z' -]", " ", low).split())
        parts = [p.strip() for p in re.split(r"[.!?,]+", low) if p.strip()]
        if self.task_words and TASK_STOP.search(bare):
            return "stop", {}, "Stopping."
        # a stop in any clause wins, even with a negation in another one ("I don't know, stop")
        if "stop" in self.allowed and any(clause_stops(p) for p in parts):
            return "stop", {}, "Stopping."
        if NEG.search(low) and not re.search(r"don'?t move", low):
            return "none", {}, "Okay, I won't."
        if self.task_words:
            if bare in TASK_ACK:
                return "carry_bag", {}, TASK_REPLY["carry_bag"]
            for action, rx in TASK_WORDS:
                if rx.search(bare):
                    return action, {}, TASK_REPLY[action]
        for action, rx in RULES:
            m = rx.search(low)
            if not m:
                continue
            if action == "stop":
                return "stop", {}, "Stopping."
            if action == "beatbox":
                return "beatbox", {}, "Okay, check this out!"
            if action == "drop_bag":
                return "drop_bag", {}, "Okay, dropping the bag."
            if action == "carry_bag":
                return "carry_bag", {}, "Sure, I will carry the bag for you."
            if action == "follow_me":
                return "follow_me", {}, "Okay, I will follow you."
            if action == "turn":
                if re.search(r"\baround\b", low):
                    ang = 180.0
                else:
                    ang = parse_number(low, r"(?:degrees?|deg)\b")
                    if ang is None:
                        ang = 20.0 if re.search(r"\b(a little|a bit|slightly)\b", low) else 90.0
                right = re.search(r"\bright\b", low) is not None
                ang = -abs(ang) if right else abs(ang)
                side = "right" if right else "left"
                say = "Turning around." if re.search(r"\baround\b", low) else \
                    f"Turning {side} {abs(ang):.0f} degrees."
                return "turn", {"angle": ang}, say
            if action in ("move_forward", "move_backward"):
                d = parse_number(low, r"(?:meters?|metres?|m)\b")
                if d is None:
                    cm = parse_number(low, r"(?:centimeters?|centimetres?|cm)\b")
                    d = cm / 100 if cm is not None else None
                if d is None and re.search(r"\bhalf\b", low):
                    d = 0.5
                if d is None:
                    d = 0.2 if re.search(r"\b(a little|a bit|slightly)\b", low) else 0.5
                params = {"distance": d}
                if re.search(r"\bslow(ly)?\b", low):
                    params["speed"] = 0.1
                elif re.search(r"\b(fast|quickly)\b", low):
                    params["speed"] = 0.3
                way = "forward" if action == "move_forward" else "back"
                unit = "meter" if d == 1 else "meters"
                return action, params, f"Moving {way} {d:g} {unit}."
            if action in ("go_to", "look_at"):
                target = m.group("target").strip()
                verb = "Going to" if action == "go_to" else "Looking at"
                return action, {"target": target}, f"{verb} the {target}."
            if action == "name":
                return "none", {}, f"I'm {self.name}. {self.capabilities}"
            if action == "caps":
                return "none", {}, self.capabilities
            if action == "thanks":
                return "none", {}, "You're welcome."
            if action == "hello":
                return "none", {}, "Hello! What can I do for you?"
        if self.fuzzy:
            a, score = fuzzy_command(t, self.allowed, self.fuzzy_min)
            if a:
                self.last_fuzzy = (a, score)
                return a, {}, FUZZY_REPLY[a]
        return "none", {}, "Sorry, I didn't catch that. Please say it again."
