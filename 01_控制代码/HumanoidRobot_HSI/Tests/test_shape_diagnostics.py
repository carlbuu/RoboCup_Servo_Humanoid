"""Exercise failure diagnostics through the actual main loop with mocked Maix devices."""
from pathlib import Path
import runpy

state = runpy.run_path(str(Path(__file__).with_name("test_vision_speed.py")))
vision, clock = state["vision"], state["clock"]
scene, run_camera = state["scene"], state["run_camera"]
frames = [scene(centered=True)] * 120

def shape_logs(output):
    lines = [line for line in output.splitlines() if line.startswith("[SHAPE]")]
    assert len(lines) == 3, lines  # 6 seconds, one diagnostic every 2 seconds.
    return lines

# A recognized card keeps its original single event while diagnostics are enabled.
writes, calls, output = run_camera(frames)
enabled = shape_logs(output)
assert calls == 4 and len([p for p in writes if p.startswith(b"SHAPE,")]) == 1
assert all("state=REPORTED" in line and "votes=4/4" in line for line in enabled)
vision.SHAPE_DIAG_ON = False
try:
    disabled_writes, _, disabled = run_camera(frames)
    assert disabled_writes == writes and "[SHAPE]" not in disabled
finally:
    vision.SHAPE_DIAG_ON = True
print("PASS diagnostic frequency is bounded and UART output is unchanged")

original_extract = vision.extract_shape
vision.extract_shape = lambda card: (None, None, vision.INSET_RETRY[-1])
try:
    failed_writes, _, output = run_camera(frames)
    assert not any(p.startswith(b"SHAPE,") for p in failed_writes)
    assert all("result=NO_SHAPE" in line and "no_shape:" in line and "votes=0/4" in line
               for line in shape_logs(output))
finally:
    vision.extract_shape = original_extract
print("PASS missing contour is distinguishable from a stopped recognition pipeline")

original_classify = vision.classify
vision.classify = lambda *args: ("square", vision.CONF_MIN / 2, {})
try:
    failed_writes, _, output = run_camera(frames)
    assert not any(p.startswith(b"SHAPE,") for p in failed_writes)
    assert all("result=LOW_CONF" in line and "cand=SQUARE" in line and "votes=0/4" in line
               for line in shape_logs(output))
finally:
    vision.classify = original_classify
print("PASS rejected confidence still reports the raw candidate")

index = [0]
def alternating(*args):
    index[0] += 1
    return ("circle" if index[0] % 2 else "triangle"), 0.9, {}
vision.classify = alternating
try:
    failed_writes, _, output = run_camera(frames)
    assert not any(p.startswith(b"SHAPE,") for p in failed_writes)
    assert all("result=OK" in line and "votes=1/4" in line and "CIRCLE" in line and
               "TRIANGLE" in line for line in shape_logs(output))
finally:
    vision.classify = original_classify
print("PASS unstable categories expose their actual voting history")

# Report gate and tracker reset counters together with the existing timed-turn state.
vision.SHOW_CONSOLE = True
clock.now = 0
diag = vision.ShapeDiagnostics()
corr = vision.LineCorrector(640, 480)
tracker = vision.CardTracker()
voter = vision.Voter(4)
diag.reset("line")
diag.reset("card")
corr.turn_waiting = True
clock.now = vision.SHAPE_DIAG_INTERVAL_MS
import contextlib
import io
capture = io.StringIO()
with contextlib.redirect_stdout(capture):
    diag.report(clock.now, corr, tracker, voter)
assert "state=TURN_WAIT" in capture.getvalue()
assert "resets=card:1/line:1" in capture.getvalue()
assert not diag.resets
print("PASS gate and tracker resets are separated and counters reset each window")
print("ALL SHAPE DIAGNOSTIC TESTS PASSED")
