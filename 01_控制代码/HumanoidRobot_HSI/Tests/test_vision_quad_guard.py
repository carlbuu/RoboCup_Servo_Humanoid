"""Actual OpenCV geometry/line regressions; camera and UART hardware are mocked."""
import argparse
import ast
import contextlib
import importlib.util
import io
from pathlib import Path
from vision_test_source import reference_vision_source
import sys
import types
import unittest

import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("--deps", help="Optional directory containing an existing cv2 package")
parser.add_argument("--baseline", type=Path, help="Optional original script for before/after checks")
args, unittest_args = parser.parse_known_args()
if args.deps:
    # Keep this interpreter's compatible NumPy; an older test directory may carry another ABI.
    sys.path.insert(0, args.deps)
import cv2

# Existing host dependencies are OpenCV 5; Maix's OpenCV 4 returns Nx1x4 here.
# Restore only that documented array shape, retaining every defect value for
# both baseline and modified classifiers. Production recognition is unchanged.
if cv2.__version__.split(".")[0] == "5":
    native_defects = cv2.convexityDefects

    def defects_opencv4_shape(*values):
        result = native_defects(*values)
        return result.reshape(-1, 1, 4) if result is not None and result.ndim == 2 else result

    cv2.convexityDefects = defects_opencv4_shape

source = reference_vision_source()
clock = types.SimpleNamespace(ticks_ms=lambda: 0, fps=lambda: 2.0)
maix = types.ModuleType("maix")
for name in ("app", "camera", "display", "image", "uart"):
    setattr(maix, name, types.SimpleNamespace())
maix.time = clock
sys.modules["maix"] = maix


def load(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    module.SHOW_CONSOLE = False
    return module


vision = load(source, "guarded_vision")
baseline = load(args.baseline, "original_vision") if args.baseline else None
full = np.array([[0, 0], [639, 0], [639, 479], [0, 479]], np.float32)
normal = np.array([[245, 205], [395, 205], [395, 355], [245, 355]], np.float32)


def scene(shape=None):
    bgr = np.full((480, 640, 3), 160, np.uint8)
    cv2.line(bgr, (150, 0), (150, 479), (20, 20, 20), 12)
    cv2.line(bgr, (490, 0), (490, 479), (20, 20, 20), 12)
    if shape:
        cv2.rectangle(bgr, (245, 205), (395, 355), (230, 230, 230), -1)
        cv2.rectangle(bgr, (245, 205), (395, 355), (0, 0, 0), 5)
        cnt = vision.build_templates()[shape]
        cnt = np.rint(cnt.astype(np.float32) * (110.0 / vision.CARD_PX) + [265, 225]).astype(np.int32)
        cv2.drawContours(bgr, [cnt], -1, (0, 0, 0), -1)
    return bgr


def expanded_mask():
    # Raw contour fills about 70%; its convex hull / final quad encloses almost 100%.
    mask = np.zeros((480, 640), np.uint8)
    points = np.array([[0, 0], [639, 0], [639, 479], [0, 479],
                       [0, 330], [500, 330], [500, 150], [0, 150]], np.int32)
    cv2.fillPoly(mask, [points], 255)
    return mask


def shape_result(module, image, quad):
    card = module.rectify(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), quad)
    cross = module.detect_cross_center(card)
    if cross > 0:
        return "cross", cross
    cnt, _, _ = module.extract_shape(card)
    if cnt is None:
        return None, None
    feat = module.features(cnt)
    if feat is None:
        return None, None
    name, conf, _ = module.classify(cnt, feat, module.build_templates())
    return name, conf


class QuadGuardTests(unittest.TestCase):
    def test_final_near_full_frame_quad_is_rejected(self):
        self.assertEqual(vision.card_quad_reject_reason(full, 640, 480), "area")
        quad, diag = vision.quad_from_mask(expanded_mask())
        self.assertIsNone(quad)
        self.assertIn("quad_reject=1:area", diag)
        if baseline:
            before, _ = baseline.quad_from_mask(expanded_mask())
            self.assertIsNotNone(before, "Fixture must reproduce the original false card")
            self.assertGreater(cv2.contourArea(before), vision.AREA_MAX_REL * 640 * 480)

    def test_fit_fallback_is_checked_after_box_points(self):
        # Force the existing fallback; OpenCV computes the actual enclosing rectangle.
        original = cv2.approxPolyDP
        cv2.approxPolyDP = lambda curve, *unused: curve[:3]
        try:
            quad, diag = vision.quad_from_mask(expanded_mask())
        finally:
            cv2.approxPolyDP = original
        self.assertIsNone(quad)
        self.assertIn("quad_reject=1:area", diag)

    def test_invalid_external_quad_cannot_erase_visible_track(self):
        gray = cv2.cvtColor(scene(), cv2.COLOR_BGR2GRAY)
        result = vision.track_center(gray, 297, 640, 170, full)
        self.assertEqual(result[2], "LR")
        self.assertAlmostEqual(result[0], 320.0)
        self.assertEqual(result, vision.track_center(gray, 297, 640, 170))
        if baseline:
            self.assertIsNone(baseline.track_center(gray, 297, 640, 170, full)[0])

    def test_bounds_finite_convexity_and_min_area(self):
        self.assertEqual(vision.card_quad_reject_reason(normal, 640, 480), "")
        self.assertEqual(vision.card_quad_reject_reason(normal + [500, 0], 640, 480), "bounds")
        bad = normal.copy(); bad[0, 0] = np.nan
        self.assertEqual(vision.card_quad_reject_reason(bad, 640, 480), "nonfinite")
        self.assertEqual(vision.card_quad_reject_reason(normal[[0, 2, 1, 3]], 640, 480), "nonconvex")
        self.assertEqual(vision.card_quad_reject_reason(np.array([[1, 1], [2, 1], [2, 2], [1, 2]]), 640, 480), "area")
        self.assertEqual(vision.card_quad_reject_reason(normal[:3], 640, 480), "shape")
        edge_card = np.array([[0, 100], [150, 100], [150, 250], [0, 250]], np.float32)
        self.assertEqual(vision.card_quad_reject_reason(edge_card, 640, 480), "")

    def test_real_card_remains_excluded_from_line(self):
        gray = cv2.cvtColor(scene("square"), cv2.COLOR_BGR2GRAY)
        result = vision.track_center(gray, 297, 640, 170, normal)
        self.assertEqual(result[2], "LR")
        self.assertAlmostEqual(result[0], 320.0)
        self.assertTrue(result[3], "True card mask must remain present")
        if baseline:
            self.assertEqual(result, baseline.track_center(gray, 297, 640, 170, normal))

    def test_real_card_without_track_still_reports_loss(self):
        image = scene("square")
        cv2.line(image, (150, 0), (150, 479), (160, 160, 160), 18)
        cv2.line(image, (490, 0), (490, 479), (160, 160, 160), 18)
        self.assertIsNone(vision.track_center(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY), 297, 640, 170, normal)[0])

    def test_unrecognized_real_card_is_still_excluded(self):
        image = scene("square")
        cv2.rectangle(image, (254, 214), (386, 346), (230, 230, 230), -1)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        quad, _ = vision.find_card_quad(vision.ink_masks(image))
        self.assertIsNotNone(quad)
        result = vision.track_center(gray, 297, 640, 170, quad)
        self.assertEqual(result[2], "LR")
        self.assertTrue(result[3], "Mask must not depend on successful shape recognition")
        self.assertAlmostEqual(result[0], 320.0)

    def test_valid_occluding_region_keeps_stop_rule(self):
        # Do not remove the existing <25% remaining-band protection for a valid region.
        covering = np.array([[0, 200], [639, 200], [639, 400], [0, 400]], np.float32)
        self.assertEqual(vision.card_quad_reject_reason(covering, 640, 480), "")
        gray = cv2.cvtColor(scene(), cv2.COLOR_BGR2GRAY)
        self.assertIsNone(vision.track_center(gray, 297, 640, 170, covering)[0])

    def test_six_real_card_quads_and_classification_are_preserved(self):
        for shape in vision.SHAPES:
            with self.subTest(shape=shape):
                image = scene(shape)
                quad, _ = vision.find_card_quad(vision.ink_masks(image))
                self.assertIsNotNone(quad)
                self.assertEqual(vision.card_quad_reject_reason(quad, 640, 480), "")
                if baseline:
                    before, _ = baseline.find_card_quad(baseline.ink_masks(image))
                    self.assertIsNotNone(before)
                    np.testing.assert_array_equal(quad, before)
                    self.assertEqual(shape_result(vision, image, quad), shape_result(baseline, image, before))

    def test_loss_and_recovery_commands_stay_unchanged(self):
        corrector = vision.LineCorrector(640, 480)
        blank = np.full((480, 640), 160, np.uint8)
        self.assertEqual([corrector.step(blank)[0] for _ in range(6)], [0, 0, 0, 0, 4, 4])
        self.assertEqual(corrector.step(cv2.cvtColor(scene(), cv2.COLOR_BGR2GRAY), full)[0], vision.CMD_FWD)

    @unittest.skipUnless(baseline, "Pass --baseline for original-source preservation checks")
    def test_classifier_voter_corrector_and_constants_unchanged(self):
        def definitions(path):
            return {n.name: ast.dump(n, include_attributes=False) for n in ast.parse(path.read_text(encoding="utf-8")).body
                    if isinstance(n, (ast.FunctionDef, ast.ClassDef))}
        before, after = definitions(args.baseline), definitions(source)
        changed = {name for name in before if before[name] != after[name]}
        self.assertEqual(changed, {"quad_from_mask", "track_center", "main"})
        def constants(path):
            return [ast.dump(n, include_attributes=False) for n in ast.parse(path.read_text(encoding="utf-8")).body
                    if isinstance(n, ast.Assign)]
        self.assertEqual(constants(args.baseline), constants(source))


if __name__ == "__main__":
    print("OpenCV=%s; synthetic images, hardware not tested" % cv2.__version__)
    unittest.main(argv=[sys.argv[0]] + unittest_args, verbosity=2)
