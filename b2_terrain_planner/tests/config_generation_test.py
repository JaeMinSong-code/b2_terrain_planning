#!/usr/bin/env python3
"""단일 설정의 모드 분리, 중복 키 거부와 ROS 호환 출력을 검증한다."""

import copy
from pathlib import Path
import sys
import tempfile
import unittest

import yaml

PACKAGE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PACKAGE / 'scripts'))
from generate_config import generate_configs, load_configs, merge_parameters


class ConfigGenerationTest(unittest.TestCase):
    def setUp(self):
        self.source = PACKAGE / 'config/b2_terrain.yaml'

    def test_runtime_modes(self):
        configs = load_configs(self.source)
        self.assertEqual(len(configs), 7)
        sim = configs['elevation_input_sim.yaml'][
            'elevation_input_adapter']['ros__parameters']
        real = configs['elevation_input_real.yaml'][
            'elevation_input_adapter']['ros__parameters']
        self.assertEqual(sim['cloud_topic'], '/lidar/points')
        self.assertEqual(real['cloud_topic'], '/rslidar_points')
        self.assertNotIn('extrinsic_xyz', sim)
        self.assertEqual(real['extrinsic_xyz'], [0.34218, 0.0, 0.17851])
        wbc = configs['terrain_planner_wbc.yaml'][
            'terrain_planner_node']['ros__parameters']
        flat = configs['terrain_planner_flat.yaml'][
            'terrain_planner_node']['ros__parameters']
        self.assertTrue(wbc['support_patch.enabled'])
        self.assertFalse(flat['support_patch.enabled'])
        self.assertNotIn('vz_weight', wbc)
        self.assertNotIn('map_datum_reanchor.enabled', flat)
        self.assertEqual(
            flat['map_topic'], '/terrain_planner/elevation_map_flat')
        self.assertEqual(wbc['command_topic'], flat['command_topic'])

    def test_nested_override_replaces_arrays_and_preserves_common(self):
        common = {'ramp': {'linear': 0.5, 'angular': 1.0}, 'command': [0, 0, 0]}
        original = copy.deepcopy(common)
        result = merge_parameters(
            common, {'ramp': {'linear': 0.25}, 'command': [1, 0, 0]})
        self.assertEqual(result['ramp'], {'linear': 0.25, 'angular': 1.0})
        self.assertEqual(result['command'], [1, 0, 0])
        self.assertEqual(common, original)

    def test_emitted_files_round_trip_and_do_not_rewrite(self):
        with tempfile.TemporaryDirectory() as directory:
            configs = generate_configs(self.source, directory)
            timestamps = {}
            for name, document in configs.items():
                path = Path(directory) / name
                self.assertEqual(yaml.safe_load(path.read_text()), document)
                timestamps[name] = path.stat().st_mtime_ns
            generate_configs(self.source, directory)
            for name, timestamp in timestamps.items():
                self.assertEqual(
                    (Path(directory) / name).stat().st_mtime_ns, timestamp)

    def test_duplicate_keys_and_missing_modes_fail_before_writing(self):
        cases = [
            self.source.read_text() + '\nexample: {}\n',
            self.source.read_text().replace('  flat:\n', '  typo:\n'),
            self.source.read_text().replace(
                '  real:\n', '  real: []\n  unexpected:\n'),
        ]
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / 'invalid.yaml'
            output = Path(directory) / 'output'
            for text in cases:
                with self.subTest(text=text[-60:]):
                    source.write_text(text)
                    with self.assertRaises(ValueError):
                        generate_configs(source, output)
                    self.assertFalse(output.exists())


if __name__ == '__main__':
    unittest.main()
