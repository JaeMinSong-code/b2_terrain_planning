#!/usr/bin/env python3
"""단일 설정의 모드 분리, 중복 키 거부와 ROS 호환 출력을 검증한다."""

import copy
import xml.etree.ElementTree as ET
from pathlib import Path
import sys
import tempfile
import unittest

import yaml

PACKAGE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PACKAGE / 'scripts'))
from generate_config import generate_configs, load_configs, merge_parameters


class ConfigGenerationTest(unittest.TestCase):
    def setUp(self):
        self.source = PACKAGE / 'config/b2_terrain.yaml'

    def test_step_swing_parameters(self):
        configs = load_configs(self.source)
        for mode in ('wbc', 'flat'):
            params = configs[f'terrain_planner_{mode}.yaml'][
                'terrain_planner_node']['ros__parameters']
            self.assertTrue(params['step_swing.enabled'])
            self.assertEqual(params['step_swing.proximity'], 0.05)
            self.assertEqual(params['step_swing.height_threshold'], 0.05)
            self.assertEqual(params['step_swing.retract_distance'], 0.05)

    def test_foot_separation_parameters(self):
        configs = load_configs(self.source)
        for mode in ('wbc', 'flat'):
            params = configs[f'terrain_planner_{mode}.yaml'][
                'terrain_planner_node']['ros__parameters']
            self.assertEqual(params['terrain.foot_separation_weight'], 0.1)
            self.assertEqual(params['terrain.foot_separation_distance_y'], 0.08)
            self.assertEqual(params['terrain.foot_separation_scale'], 0.01)

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
        self.assertTrue(real['use_receive_time'])
        self.assertTrue(real['self_filter.enabled'])
        self.assertEqual(real['self_filter.foot_radius'], .032)
        self.assertEqual(real['self_filter.box_padding_xy'], [0., .08])
        self.assertEqual(real['self_filter.box_top_padding'], .05)
        self.assertNotIn('self_filter.enabled', sim)
        self.assertEqual(real['body_frame_alias'], 'robot_center')
        self.assertNotIn('use_receive_time', sim)
        self.assertEqual(real['extrinsic_xyz'], [0.34218, 0.0, 0.17851])
        wbc = configs['terrain_planner_wbc.yaml'][
            'terrain_planner_node']['ros__parameters']
        flat = configs['terrain_planner_flat.yaml'][
            'terrain_planner_node']['ros__parameters']
        self.assertTrue(wbc['support_patch.enabled'])
        self.assertTrue(wbc['terrain.pipeline_enabled'])
        self.assertTrue(wbc['terrain.graduated_initialization'])
        self.assertEqual(wbc['terrain.graduated_budget_fraction'], 0.25)
        self.assertGreater(wbc['terrain.virtual_floor_sigma'], wbc['terrain.smoothing_sigma'])

        self.assertFalse(flat['support_patch.enabled'])
        self.assertNotIn('vz_weight', wbc)
        self.assertNotIn('map_datum_reanchor.enabled', flat)
        self.assertEqual(
            flat['map_topic'], '/terrain_planner/elevation_map_flat')
        self.assertEqual(wbc['command_topic'], flat['command_topic'])
        self.assertEqual(wbc['max_reach_margin'], 0.02)
        self.assertEqual(flat['max_reach_margin'], 0.02)
        self.assertEqual(wbc['foot_radius'], 0.032)
        self.assertEqual(flat['foot_radius'], wbc['foot_radius'])
        self.assertEqual(configs['terrain_planner_flat.yaml'][
            'flat_elevation_map']['ros__parameters']['foot_radius'], flat['foot_radius'])
        self.assertNotIn('map_datum_reanchor.contact_offset', wbc)
        self.assertNotIn('swing_foot_half_width', wbc)

    def test_flat_node_uses_the_configured_radius(self):
        with tempfile.TemporaryDirectory() as directory:
            source = Path(directory) / 'radius.yaml'
            document = yaml.safe_load(self.source.read_text())
            document['planner']['common']['foot_radius'] = 0.05
            source.write_text(yaml.safe_dump(document))
            config = load_configs(source)['terrain_planner_flat.yaml']
            self.assertEqual(config['terrain_planner_node']['ros__parameters'][
                'foot_radius'], 0.05)
            self.assertEqual(config['flat_elevation_map']['ros__parameters'][
                'foot_radius'], 0.05)

    def test_foot_collision_matches_planner_radius(self):
        config = load_configs(self.source)['terrain_planner_wbc.yaml']
        radius = config['terrain_planner_node']['ros__parameters']['foot_radius']
        robot = ET.parse(PACKAGE / 'model/unitree_b2/b2.urdf').getroot()
        for leg in ['FL', 'FR', 'RL', 'RR']:
            collision = robot.find(f"link[@name='{leg}_foot']/collision")
            geometry = list(collision.find('geometry'))
            self.assertEqual(len(geometry), 1)
            self.assertEqual(geometry[0].tag, 'sphere')
            self.assertEqual(float(geometry[0].get('radius')), radius)
            self.assertEqual(collision.find('origin').get('xyz'), '0 0 0')

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
