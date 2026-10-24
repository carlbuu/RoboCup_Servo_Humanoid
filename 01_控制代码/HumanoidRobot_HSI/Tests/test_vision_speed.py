"""Check output parity and bounded image work independently of display refresh."""
import contextlib
import io
from pathlib import Path
import runpy

state = runpy.run_path(str(Path(__file__).with_name("test_card_guard.py")))
vision = state["vision"]
cv2, np = state["cv2"], state["np"]
scene, run_camera = state["scene"], state["run_camera"]
clock, maix = state["clock"], state["maix"]

# Strip conversion must give exactly the same track estimate as full-frame gray.
for bgr in (scene(), scene(centered=True), scene(card=False),
            np.full((480, 640, 3), 240, np.uint8)):
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    tracker = vision.CardTracker()
    tracker.observe(vision.find_card_quads_fast(bgr), 0)
    for yc in (297, 432):
        for regions in ((), tracker.exclusion_regions(0)):
            assert vision.track_center(bgr, yc, 640, 190, regions) == \
                   vision.track_center(gray, yc, 640, 190, regions)
print("PASS color-strip and full-gray track centers are identical")

# Default Otsu success generates one mask; fixed thresholds stay available on demand.
original_threshold = cv2.threshold
threshold_calls = []
def threshold(*args, **kwargs):
    threshold_calls.append(args[1])
    return original_threshold(*args, **kwargs)
cv2.threshold = threshold
try:
    assert vision.find_card_quads_fast(scene())
    assert len(threshold_calls) == 1, threshold_calls
    threshold_calls.clear()
    eager = vision.ink_masks(scene())
    lazy = list(vision.iter_ink_masks(scene()))
    assert all(np.array_equal(a, b) for a, b in zip(eager, lazy))
    assert len(lazy) == 1 + len(vision.THRESH_LIST)
finally:
    cv2.threshold = original_threshold
print("PASS Otsu success needs one threshold and lazy fallback matches all original masks")

# Refresh cadence must never drive the camera loop or throttle serial decisions.
display_count = [0]
# run_camera creates the Display factory itself, so count conversion calls instead.
original_convert = maix.image.cv2image
original_color = cv2.cvtColor
full_grays = []
def convert(frame, **kwargs):
    display_count[0] += 1
    return frame
def color(frame, mode, *args, **kwargs):
    if mode == cv2.COLOR_BGR2GRAY and frame.shape[:2] == (480, 640):
        full_grays.append(frame.shape)
    return original_color(frame, mode, *args, **kwargs)

# Install observers inside the helper's assignments via a custom module namespace.
class ImageAPI:
    def __setattr__(self, name, value):
        object.__setattr__(self, name, convert if name == "cv2image" else value)
original_image = maix.image
maix.image = ImageAPI()
vision.image = maix.image
cv2.cvtColor = color
frames = [scene(centered=True)] * 120
try:
    displayed, calls, output = run_camera(frames)
    assert display_count[0] == 60, display_count
    assert not full_grays, full_grays
    assert calls == 4
    assert "[PERF]" in output and "capture=" in output and "display=" in output
    vision.DISPLAY_ON = False
    display_count[0] = 0
    headless, calls_headless, _ = run_camera(frames)
    assert display_count[0] == 0
    assert displayed == headless, "Display must not change UART output or shape votes"
    assert calls_headless == calls
finally:
    vision.DISPLAY_ON = True
    maix.image = original_image
    vision.image = original_image
    maix.image.cv2image = original_convert
    cv2.cvtColor = original_color
print("PASS 120 frames: 60 displays, zero full-frame grayscale conversions, identical headless UART")

# Large nearby cards have bounded refinement cost, preserving original-frame coordinates.
large = np.full((480, 640, 3), 240, np.uint8)
cv2.rectangle(large, (160, 60), (520, 420), (0, 0, 0), 10)
cv2.rectangle(large, (240, 140), (440, 340), (0, 0, 0), 8)
coarse = vision.find_card_quads_fast(large)[0]
original_masks = vision.iter_ink_masks
refine_sizes = []
def masks(roi):
    refine_sizes.append(roi.shape[:2])
    yield from original_masks(roi)
vision.iter_ink_masks = masks
try:
    quad, _ = vision.refine_card_quad(large, coarse)
    assert max(refine_sizes[0]) <= vision.CARD_REFINE_MAX_SIDE
    assert np.allclose(vision.quad_bounds(quad), (155, 55, 525, 425), atol=8), quad
    card = vision.rectify(large, quad)
    contour, _, _ = vision.extract_shape(card)
    label, _, _ = vision.classify(contour, vision.features(contour), vision.build_templates())
    assert label == "square", label
finally:
    vision.iter_ink_masks = original_masks
print("PASS large-card refinement is bounded and original coordinates/classification preserved")

# Runtime telemetry reports slow stages and differentiates the timed turn wait.
clock.now = 0
stats = vision.LoopStats()
corr = vision.LineCorrector(640, 480)
corr.turn_waiting = True
stats.add("capture", 0, 15)
stats.add("shape", 15, 180)
clock.now = vision.PERF_INTERVAL_MS
text = io.StringIO()
with contextlib.redirect_stdout(text):
    stats.finish(0, clock.now, corr)
assert "shape=165.0" in text.getvalue() and "state=TURN_WAIT" in text.getvalue()
print("PASS telemetry identifies stage latency and timed turn wait")
print("ALL VISION SPEED TESTS PASSED")
