"""Stored scans along the route out (keyframes), and the pose correction on the way back.

A keyframe is a pose in the corrected frame and the scan taken there. Near one, the current scan is
matched against it and the believed pose moves to where the match says, if the match is trustworthy
(spec "Position correction"). Poses are (x, y, yaw) in the odometry frame.
"""
import math

import motion
import scanmatch

NEAR = 1.0  # m, a keyframe this close is matched
FACING = math.radians(30)  # and the heading within this of its heading or its reverse
SHARE_MIN = 0.6  # of the points agree within scanmatch.AGREE
MAX_SHIFT = 0.5  # m, a larger correction is refused
MAX_TURN = math.radians(20)
SLIDE_MAX = 0.8  # above this the match cannot tell position along the surfaces (Review Focus 1)


class Keyframes:
    def __init__(self, p):
        self.p, self.frames = p, []

    def add(self, pose, scan):
        self.frames.append((tuple(pose), scan))

    def correct(self, pose, scan):
        """(pose, report): the corrected pose, or `pose` itself when no match is used, and
        {"used", "index", "share", "shift", "turn", "why"}; why is "far", "no match", "share",
        "shift", "turn" or "used"."""
        report = {"used": False, "index": None, "share": 0.0, "shift": 0.0, "turn": 0.0, "why": "far"}
        near = []
        for i, (key, _) in enumerate(self.frames):
            gap = math.hypot(pose[0] - key[0], pose[1] - key[1])
            facing = abs(motion._wrap(pose[2] - key[2]))
            if gap <= NEAR and (facing <= FACING or facing >= math.pi - FACING):
                near.append((gap, i))
        if not near:
            return pose, report
        report["index"] = i = min(near)[1]
        key, reference = self.frames[i]
        guess = motion._relative(key, pose)  # the believed pose in the keyframe's frame; yaw near pi when reversed
        try:
            # only near returns, as in M1: far walls have no surface lines and read the shift short
            dx, dy, dyaw, share, slide = scanmatch.scan_match(motion._near(reference), motion._near(scan), guess, self.p)
        except ValueError:
            report["why"] = "no match"
            return pose, report
        if slide > SLIDE_MAX:
            # shortcut: "along" is the keyframe's heading, fine while it was stored driving the corridor;
            # return the slide direction from scan_match if keyframes face across corridors
            dx = guess[0]
        turn = motion._wrap(dyaw - guess[2])
        report.update(share=share, shift=math.hypot(dx - guess[0], dy - guess[1]), turn=turn)
        if share < SHARE_MIN:
            report["why"] = "share"
        elif report["shift"] >= MAX_SHIFT:
            report["why"] = "shift"
        elif abs(turn) >= MAX_TURN:
            report["why"] = "turn"
        else:
            x, y, yaw = motion._compose(key, (dx, dy, dyaw))
            report.update(used=True, why="used")
            return (x, y, pose[2] + motion._wrap(yaw - pose[2])), report  # keep the caller's turn count
        return pose, report
