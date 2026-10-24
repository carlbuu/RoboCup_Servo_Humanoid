"""Exercise the real visual correction class without importing Maix hardware APIs."""
import ast
from collections import deque
from pathlib import Path
from vision_test_source import reference_vision_source
from types import SimpleNamespace

script = reference_vision_source()
source = script.read_text(encoding="utf-8-sig")
compile(source, str(script), "exec")
tree = ast.parse(source)


class Clock:
    now = 0

    def ticks_ms(self):
        return self.now


clock = Clock()
env = {"time": clock, "deque": deque, "SHOW_CONSOLE": False,
       "CMD_HOLD": 0, "CMD_FWD": 1, "CMD_TURN_L": 2,
       "CMD_TURN_R": 3, "CMD_BLOCKED": 4}
for node in tree.body:
    if isinstance(node, ast.Assign) and len(node.targets) == 1:
        name = getattr(node.targets[0], "id", "")
        if name.startswith(("CORR_", "CARD_", "SHAPE_SCAN_")) or name == "SHAPE_EVERY":
            env[name] = ast.literal_eval(node.value)
    if isinstance(node, ast.ClassDef) and node.name in ("LineCorrector", "Voter"):
        exec(compile(ast.Module(body=[node], type_ignores=[]), str(script), "exec"), env)

tol, ok = env["CORR_E_TOL"], env["CORR_E_OK"]
assert 0 < ok <= tol
corr = env["LineCorrector"](640, 480)
deviation = [tol]
corr.measure = lambda gray: (deviation[0], "LR")
assert corr.step(None)[0] == env["CMD_FWD"] and corr.aligned
corr.wait = 0
deviation[0] = tol + 1
assert corr.step(None)[0] == env["CMD_TURN_R"] and not corr.aligned
assert corr.turns == 1
clock.now = 9999
assert corr.step(None)[0] is None and corr.turns == 1
clock.now = 10000
for _ in range(env["CORR_SETTLE"]):
    assert corr.step(None)[0] is None
deviation[0] = ok + 1
assert corr.step(None)[0] == env["CMD_TURN_R"] and not corr.aligned
clock.now += 10000
corr.wait = 0
deviation[0] = ok
assert corr.step(None)[0] == env["CMD_FWD"] and corr.aligned

half = env["LineCorrector"](320, 240)
assert half.e_tol == tol / 2 and half.e_ok == ok / 2
left = env["LineCorrector"](640, 480)
left.measure = lambda gray: (-tol - 1, "LR")
assert left.step(None)[0] == env["CMD_TURN_L"]
lost = env["LineCorrector"](640, 480)
lost.measure = lambda gray: (None, "")
assert lost.step(None)[0] == env["CMD_TURN_R"] and not lost.aligned
lost_left = env["LineCorrector"](640, 480)
lost_left.last_e = -60
lost_left.measure = lambda gray: (None, "")
assert lost_left.step(None)[0] == env["CMD_TURN_L"]
assert lost_left.step(None)[0] is None  # One action at a time, not continuous blind turns.
many = env["LineCorrector"](640, 480)
many.measure = lambda gray: (tol + 1, "LR")
for _ in range(env["CORR_MAX_TURN"] + 2):
    clock.now += 10000
    many.wait = 0
    assert many.step(None)[0] == env["CMD_TURN_R"]

# Execute the actual main-loop gate with correction states produced above.
main = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "main")
loop = next(node for node in main.body if isinstance(node, ast.While))
index = next(i for i, node in enumerate(loop.body) if isinstance(node, ast.Assign)
             and getattr(node.targets[0], "id", "") == "line_ready")
gate = compile(ast.Module(body=loop.body[index:index + 3], type_ignores=[]), str(script), "exec")
voter = env["Voter"](4)
voter.buf.extend(["circle", "circle"])
voter.locked = "circle"
class ReadyCard:
    def can_classify(self, now):
        return True


env.update({"corr": left, "voter": voter, "frame": 3, "now": clock.now,
            "diag": SimpleNamespace(reset=lambda reason: None),
            "last_shape_scan": -1000, "tracker": ReadyCard()})
exec(gate, env)
assert not env["ran_shape"] and not voter.buf and voter.locked == "circle"
env["corr"] = corr
exec(gate, env)
assert env["ran_shape"]
print("PASS visual thresholds, timed single-turn search, last-direction lost-line recovery, no 14-turn lock and shape gate")
