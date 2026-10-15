import json
import shutil
import tempfile
import unittest
from pathlib import Path

import numpy as np
import pyarrow
import pyarrow.parquet as parquet

from umi_replay.dataset import read_episode
from umi_replay.download import sha256

ROOT = Path(__file__).resolve().parents[2] / "data/xtac-umi-g1-block-to-box"


@unittest.skipUnless((ROOT / "download-manifest.json").exists(), "Download the pinned public dataset first")
class DatasetTests(unittest.TestCase):
    def fixture(self, directory):
        destination = Path(directory) / "dataset"
        shutil.copytree(ROOT, destination, ignore=shutil.ignore_patterns("videos", "assets", ".cache"))
        return destination

    def refresh_checksum(self, destination, relative):
        manifest_path = destination / "download-manifest.json"
        manifest = json.loads(manifest_path.read_text())
        for item in manifest["files"]:
            if item["path"] == relative:
                item["sha256"] = sha256(destination / relative)
        manifest_path.write_text(json.dumps(manifest))

    def test_all_episodes_are_valid(self):
        counts = []
        for episode in range(11):
            frames, provenance = read_episode(ROOT, episode)
            counts.append(len(frames))
            self.assertTrue(provenance["extrinsics_already_applied"])
            self.assertNotEqual(provenance["revision"], "unverified_local_copy")
        self.assertEqual(sum(counts), 13500)

    def test_action_is_next_state_not_delta(self):
        table = parquet.read_table(ROOT / "data/chunk-000/file-000.parquet")
        states = np.array(table["observation.state"].to_pylist())
        actions = np.array(table["action"].to_pylist())
        episodes = table["episode_index"].to_numpy()
        mask = episodes[:-1] == episodes[1:]
        np.testing.assert_allclose(actions[:-1][mask], states[1:][mask], atol=0, rtol=0)

    def test_missing_episode_rejected(self):
        with self.assertRaises(ValueError):
            read_episode(ROOT, 999)

    def test_g1_is_handheld_not_unitree(self):
        info = json.loads((ROOT / "meta/info.json").read_text())
        self.assertEqual(info["robot_type"], "xtac_umi_g1")
        self.assertEqual(len(info["features"]["observation.state"]["names"]), 29)

    def test_checksum_tampering_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            destination = self.fixture(directory)
            path = destination / "meta/info.json"
            path.write_text(path.read_text() + " ")
            with self.assertRaisesRegex(ValueError, "checksum mismatch"):
                read_episode(destination, 0)

    def test_ambiguous_pose_stage_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            destination = self.fixture(directory)
            relative = "meta/taccap_extrinsics.json"
            path = destination / relative
            metadata = json.loads(path.read_text())
            metadata["episodes"][0]["poses_are_ee"] = False
            path.write_text(json.dumps(metadata))
            self.refresh_checksum(destination, relative)
            with self.assertRaisesRegex(ValueError, "EEF/world convention"):
                read_episode(destination, 0)

    def test_low_confidence_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            destination = self.fixture(directory)
            relative = "data/chunk-000/file-000.parquet"
            path = destination / relative
            table = parquet.read_table(path)
            key = "observation.tracker_confidence"
            values = table[key].to_pylist()
            values[0] = [0.5, 1.0]
            column = pyarrow.array(values, type=table.schema.field(key).type)
            table = table.set_column(table.schema.get_field_index(key), key, column)
            parquet.write_table(table, path)
            self.refresh_checksum(destination, relative)
            with self.assertRaisesRegex(ValueError, "tracking confidence"):
                read_episode(destination, 0)


if __name__ == "__main__":
    unittest.main()
