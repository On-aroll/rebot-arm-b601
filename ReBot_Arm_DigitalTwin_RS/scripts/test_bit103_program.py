import math
import unittest
from generate_bit103_program import TEXT, make_program
from writing_plane import tool_plane, project, dot


class WritingProgramTests(unittest.TestCase):
    def test_all_ink_stays_on_tcp_normal_plane_with_90mm_word_width(self):
        pose = [.4, .1, .078, .1, .2, .3]
        program, path = make_program(pose)
        self.assertEqual("".join(dict.fromkeys(s["char"] for s in path["strokes"])), TEXT)
        self.assertLessEqual(len(program["steps"]), 250)
        plane = path["plane"]
        points = [project(plane, p) for s in path["strokes"] for p in s["points"]]
        self.assertAlmostEqual(max(p[0] for p in points) - min(p[0] for p in points), .09)
        self.assertAlmostEqual(max(p[1] for p in points) - min(p[1] for p in points), .02)
        for step in program["steps"]:
            self.assertEqual(step["pose"][3:], pose[3:])
            offset = 0 if step["label"].endswith(("书写", "落笔")) else .005
            self.assertAlmostEqual(project(plane, step["pose"][:3])[2], offset)
            self.assertTrue(step["orientation"])

    def test_each_stroke_has_lift_transfer_and_pen_down(self):
        program, geometry = make_program([.4, .1, .078, 0, 0, 0])
        groups = {}
        for step in program["steps"]:
            char, index, action = step["label"].split("·")
            groups.setdefault((char, index), []).append(action)
        self.assertEqual(len(groups), len(geometry["strokes"]))
        for actions in groups.values():
            self.assertEqual(actions[:2], ["移笔", "落笔"])
            self.assertEqual(actions[-1], "抬笔")
            self.assertIn("书写", actions)

    def test_oversize_or_nonfinite_input_is_rejected(self):
        with self.assertRaises(ValueError):
            make_program([.4, .1, .078, 0, 0, 0], width=.1001)
        with self.assertRaises(ValueError):
            make_program([.4, .1, math.nan, 0, 0, 0])

    def test_plane_tracks_rotated_gripper_and_remains_frozen(self):
        for rpy in ([0, 0, 0], [.4, -.8, 1.2], [0, math.pi / 2, 0]):
            pose = [.4, .1, .078, *rpy]
            program, geometry = make_program(pose)
            plane = geometry["plane"]
            for name in ("u", "v", "normal"):
                self.assertAlmostEqual(dot(plane[name], plane[name]), 1)
            for a, b in (("u", "v"), ("u", "normal"), ("v", "normal")):
                self.assertAlmostEqual(dot(plane[a], plane[b]), 0)
            self.assertTrue(plane["frozen"])
            self.assertEqual(program["writing_plane"]["origin"], pose[:3])
            for stroke in geometry["strokes"]:
                for p in stroke["points"]:
                    self.assertAlmostEqual(project(plane, p)[2], 0)
        self.assertEqual(tool_plane([0, 0, 0, 0, 0, 0])["normal"], [-1, 0, 0])


if __name__ == "__main__":
    unittest.main()
