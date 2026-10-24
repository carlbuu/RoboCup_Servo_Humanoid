"""Synthetic-image integration checks; Maix devices are simulated, OpenCV is real."""
import contextlib
import importlib.util
import io
import math
from pathlib import Path
from vision_test_source import reference_vision_source
import sys
import types

if len(sys.argv) > 1:
    sys.path.insert(0, sys.argv[1])
import cv2
import numpy as np


class Clock:
    now = 0

    def ticks_ms(self):
        return self.now

    def fps(self):
        return 20.0


clock = Clock()
maix = types.ModuleType("maix")
for name in ("app", "camera", "display", "image", "uart"):
    setattr(maix, name, types.SimpleNamespace())
maix.time = clock
sys.modules["maix"] = maix
path = reference_vision_source()
spec = importlib.util.spec_from_file_location("vision_card_guard", path)
vision = importlib.util.module_from_spec(spec)
spec.loader.exec_module(vision)
vision.SHOW_CONSOLE = False


def scene(card=True, centered=False):
    bgr = np.full((480, 640, 3), 240, dtype=np.uint8)
    left, right = (70, 570) if centered else (70, 450)
    cv2.line(bgr, (left, 0), (left, 479), (0, 0, 0), 8)
    cv2.line(bgr, (right, 0), (right, 479), (0, 0, 0), 8)
    if card:
        cv2.rectangle(bgr, (240, 220), (400, 380), (0, 0, 0), 6)
        cv2.rectangle(bgr, (270, 250), (370, 350), (0, 0, 0), 6)
    return bgr


bgr = scene()
gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
quads = vision.find_card_quads_fast(bgr)
assert quads, "Small-image detector must find the square card before line control"
tracker = vision.CardTracker()
assert tracker.observe(quads, 0)
raw = vision.track_center(gray, 297, 640, 190)
guarded = vision.track_center(gray, 297, 640, 190, tracker.exclusion_regions(0))
assert abs(raw[0] - 260) > 20, raw
assert guarded[2] == "LR" and abs(guarded[0] - 260) <= 1, guarded
assert abs(guarded[1] - 190) <= 1
line = vision.LineCorrector(640, 480)
line.card_regions = tracker.exclusion_regions(0)
assert line.measure(gray)[0] < 0, "Card must not reverse the measured correction direction"
# The geometric fixture has 60px deviation; exercise its turn with a 50px trigger
# independently of the user's current configurable threshold.
line.e_tol = 50
assert line.step(gray)[0] == vision.CMD_TURN_L, "Card must not reverse the correction direction"
assert not vision.find_card_quads_fast(scene(card=False))
refined, _ = vision.refine_card_quad(bgr, tracker.quad)
assert refined is not None

connected = scene()
cv2.line(connected, (400, 297), (450, 297), (0, 0, 0), 8)
joined_quads = vision.find_card_quads_fast(connected)
joined = vision.CardTracker()
assert joined.observe(joined_quads, 0)
joined_line = vision.track_center(cv2.cvtColor(connected, cv2.COLOR_BGR2GRAY), 297,
                                  640, 190, joined.exclusion_regions(0))
assert joined_line[0] is not None and abs(joined_line[0] - 260) <= 1, joined_line

# A card by itself is never a replacement for actual track boundaries.
only_card = scene()
cv2.line(only_card, (70, 0), (70, 479), (240, 240, 240), 12)
cv2.line(only_card, (450, 0), (450, 479), (240, 240, 240), 12)
assert vision.track_center(cv2.cvtColor(only_card, cv2.COLOR_BGR2GRAY), 297,
                           640, 190, tracker.exclusion_regions(0))[0] is None

# One reported instance stays locked while moving; another instance can be recognized.
tracker.mark_reported("square", 0.9)
assert not tracker.can_classify(0)
moved = [q + np.array([15, 10], np.float32) for q in quads]
assert not tracker.observe(moved, 200) and tracker.reported
assert not tracker.observe([], 500) and tracker.reported
assert not tracker.exclusion_regions(1000), "Stale masks cannot cover newly visible track"
assert tracker.observe([], 200 + vision.CARD_GONE_MS) and not tracker.reported
assert tracker.observe(quads, 2000) and tracker.can_classify(2000)
tracker.mark_reported("square", 0.9)
assert tracker.observe([q + np.array([220, -100], np.float32) for q in quads], 2100)
assert tracker.can_classify(2100), "A different card instance is not locked by shape label"
print("PASS square exclusion, correct turn direction, real-track requirement, mask expiry and per-card rearm")


def run_camera(frames):
    writes = []
    index = [0]
    classify_calls = [0]
    original_classify = vision.classify

    def classify(*args):
        classify_calls[0] += 1
        return original_classify(*args)

    def read_frame():
        frame = frames[index[0]]
        index[0] += 1
        clock.now += 50
        return frame

    clock.now = 0
    maix.app.need_exit = lambda: index[0] >= len(frames)
    maix.camera.Camera = lambda *args: types.SimpleNamespace(read=read_frame)
    maix.display.Display = lambda: types.SimpleNamespace(show=lambda frame: None)
    maix.image.image2cv = lambda frame, **kwargs: frame
    maix.image.cv2image = lambda frame, **kwargs: frame
    maix.uart.UART = lambda *args: types.SimpleNamespace(write=lambda payload: writes.append(payload))
    vision.classify = classify
    vision.SHOW_CONSOLE = True
    capture = io.StringIO()
    try:
        with contextlib.redirect_stdout(capture):
            vision.main()
    finally:
        vision.classify = original_classify
    return writes, classify_calls[0], capture.getvalue()


writes, calls, console = run_camera([scene(centered=True)] * 120)
shapes = [p for p in writes if p.startswith(b"SHAPE,")]
commands = [p for p in writes if p.startswith(b"CMD,")]
assert len(shapes) == 1 and shapes[0] == b"SHAPE,SQUARE,3\n", shapes
assert calls == 4, calls
assert len(commands) >= 10, len(commands)
assert all(p == b"CMD,FWD,2\n" for p in commands)
assert console.count("[指令]") <= 7 and "[特征]" not in console
print("PASS 120 frames: %d classifications, %d SHAPE, %d CMD; command logs throttled" %
      (calls, len(shapes), len(commands)))

writes, calls, _ = run_camera([scene(centered=True)] * 80 +
                              [scene(card=False, centered=True)] * 40 +
                              [scene(centered=True)] * 80)
assert len([p for p in writes if p.startswith(b"SHAPE,")]) == 2
assert calls == 8, calls
print("PASS same label on a new visible card after departure produces a second event")

templates = vision.build_templates(120)
for label, contour in templates.items():
    card_scene = scene(centered=True)
    cv2.rectangle(card_scene, (264, 244), (376, 356), (240, 240, 240), -1)
    cv2.drawContours(card_scene, [contour + np.array([[[260, 240]]], np.int32)],
                     -1, (0, 0, 0), -1)
    writes, _, _ = run_camera([card_scene] * 80)
    shapes = [p for p in writes if p.startswith(b"SHAPE,")]
    expected = ("SHAPE,%s,%d\n" % (label.upper(), vision.SHAPE_ID[label])).encode()
    assert shapes == [expected], (label, shapes)
print("PASS six original shape mappings after ROI refinement")

# Failure to write the shape must not mark it as already reported.
vision.SHOW_CONSOLE = False
maix.uart.UART = lambda *args: types.SimpleNamespace(write=lambda payload: (_ for _ in ()).throw(OSError("test")))
with contextlib.redirect_stdout(io.StringIO()):
    out = vision.SerialOut("test", 115200)
    assert out.send("square", 3) is False
print("ALL CARD GUARD TESTS PASSED")
