"""Test the deployed LineCorrector decisions without Maix camera hardware."""
import ast
from pathlib import Path
from vision_test_source import current_vision_source
import unittest

source = current_vision_source()
tree = ast.parse(source.read_text(encoding="utf-8"))
names = {"CORR_TRACK_W", "CORR_E_TOL", "CORR_E_OK", "CORR_LOST_N", "CORR_STEP_N",
         "CORR_TURN_DEG", "CORR_SHIFT_ON", "CORR_HEADING_TOL"}
definitions = [node for node in tree.body if isinstance(node, ast.Assign)
               and any(isinstance(t, ast.Name) and t.id in names for t in node.targets)]
definitions.append(next(node for node in tree.body if isinstance(node, ast.ClassDef)
                        and node.name == "LineCorrector"))
namespace = dict(CMD_HOLD=0, CMD_FWD=1, CMD_TURN_L=2, CMD_TURN_R=3,
                 CMD_BLOCKED=4, CMD_SHIFT_L=6, CMD_SHIFT_R=7)
exec(compile(ast.Module(body=definitions, type_ignores=[]), str(source), "exec"), namespace)


class ContinuousAdviceTests(unittest.TestCase):
    def setUp(self):
        self.corrector = namespace["LineCorrector"](640, 480)
        self.error = 0
        self.corrector.measure = lambda *args: (self.error, "LR")

    def step(self):
        return self.corrector.step(None)[0]

    def test_repeated_turn_advice_never_counts_as_executed_turns(self):
        self.error = 150
        self.assertEqual([self.step() for _ in range(100)], [3] * 100)
        self.assertEqual(self.corrector.wait, 0)
        self.assertEqual(self.corrector.net_turn, 0)
        self.assertEqual(self.corrector.pending, [])

    def test_every_image_produces_forward_advice(self):
        self.assertEqual([self.step() for _ in range(20)], [1] * 20)

    def test_lost_line_stops_without_inventing_reverse_rotation(self):
        self.error = None
        self.assertEqual([self.step() for _ in range(6)], [0, 0, 0, 0, 4, 4])
        self.assertEqual(self.corrector.pending, [])
        self.error = 0
        self.assertEqual(self.step(), 1)

    def test_parallel_offset_shifts_and_heading_error_turns(self):
        self.error = 150
        self.corrector.near_e, self.corrector.heading_e = 140, 10
        self.assertEqual(self.step(), 7)
        self.corrector.heading_e = 80
        self.assertEqual(self.step(), 3)
        self.corrector.heading_e = None
        self.assertEqual(self.step(), 3)
        self.error = -150
        self.corrector.near_e, self.corrector.heading_e = -140, -10
        self.assertEqual(self.step(), 6)

    def test_existing_hysteresis_thresholds_are_kept(self):
        self.error = 150
        self.assertEqual(self.step(), 3)
        self.error = 90
        self.assertEqual(self.step(), 3)
        self.error = 60
        self.assertEqual(self.step(), 1)
        self.error = 90
        self.assertEqual(self.step(), 1)

    def test_confirmed_loss_discards_old_lane_center_prior(self):
        self.corrector.c_far, self.corrector.c_near = 150, 160
        self.error = None
        for _ in range(4):
            self.step()
        self.assertEqual(self.corrector.c_far, 150)
        self.assertEqual(self.corrector.c_near, 160)
        self.step()
        self.assertIsNone(self.corrector.c_far)
        self.assertIsNone(self.corrector.c_near)


if __name__ == "__main__":
    unittest.main(verbosity=2)
