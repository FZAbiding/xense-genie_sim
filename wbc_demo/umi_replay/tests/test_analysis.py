import unittest

from umi_replay.analysis import source_at


class SourcePhaseTests(unittest.TestCase):
    def test_source_phase_interpolates_original_frames(self):
        frames = []
        for index in range(3):
            pose = {"position": [index * 0.1, 0, 0], "quaternion_xyzw": [0, 0, 0, 1]}
            frames.append({"timestamp": index * 0.1, "eef": [pose, pose]})
        self.assertAlmostEqual(source_at(frames, 0.15, 1)["position"][0], 0.15)
        self.assertAlmostEqual(source_at(frames, 0.0, 0)["position"][0], 0.0)
        self.assertAlmostEqual(source_at(frames, 0.2, 0)["position"][0], 0.2)


if __name__ == "__main__":
    unittest.main()
