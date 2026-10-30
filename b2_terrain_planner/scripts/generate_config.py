#!/usr/bin/env python3
"""단일 B2 설정에서 기존 실행기용 ROS 파라미터 YAML을 생성한다."""

import argparse
import copy
from pathlib import Path

import yaml


class UniqueKeyLoader(yaml.SafeLoader):
    """중복 키로 설정값이 조용히 사라지는 것을 거부한다."""


def _unique_mapping(loader, node):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node)
        if not isinstance(key, str):
            raise ValueError('설정 키는 문자열이어야 합니다')
        if key in result:
            raise ValueError(f'중복 설정 키: {key}')
        result[key] = loader.construct_object(value_node)
    return result


UniqueKeyLoader.add_constructor(
    yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _unique_mapping)


def _mapping(value, label, keys=None):
    if not isinstance(value, dict):
        raise ValueError(f'{label}: mapping이 필요합니다')
    if keys is not None and set(value) != set(keys):
        raise ValueError(f'{label}: 필요한 필드는 {", ".join(keys)}입니다')
    return value


def merge_parameters(common, overrides):
    """중첩 mapping은 병합하고 스칼라·배열은 모드별 값으로 대체한다."""
    result = copy.deepcopy(common)
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = merge_parameters(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def load_configs(path):
    """출력 전에 전체 구조를 검증하고 파일명별 ROS 문서를 반환한다."""
    with Path(path).open(encoding='utf-8') as stream:
        document = yaml.load(stream, Loader=UniqueKeyLoader)
    _mapping(document, '설정', ('input', 'mapping', 'planner', 'example'))
    inputs = _mapping(document['input'], 'input', ('sim', 'real', 'wbc'))
    planner = _mapping(
        document['planner'], 'planner', ('common', 'wbc', 'flat'))
    common = _mapping(planner['common'], 'planner.common')
    outputs = {}
    for mode in ('sim', 'real', 'wbc'):
        parameters = _mapping(inputs[mode], f'input.{mode}')
        outputs[f'elevation_input_{mode}.yaml'] = {
            'elevation_input_adapter': {'ros__parameters': parameters}}
    outputs['elevation_map_sim.yaml'] = {
        '/elevation_mapping_node': {
            'ros__parameters': _mapping(document['mapping'], 'mapping')}}
    for mode in ('wbc', 'flat'):
        overrides = _mapping(planner[mode], f'planner.{mode}')
        outputs[f'terrain_planner_{mode}.yaml'] = {
            'terrain_planner_node': {
                'ros__parameters': merge_parameters(common, overrides)}}
    # 같은 YAML을 두 노드에 전달해 반경 설정의 소유권을 planner.common에 둔다.
    flat_parameters = outputs['terrain_planner_flat.yaml'][
        'terrain_planner_node']['ros__parameters']
    outputs['terrain_planner_flat.yaml']['flat_elevation_map'] = {
        'ros__parameters': {'foot_radius': flat_parameters['foot_radius']}}
    outputs['planner_b2_example.yaml'] = _mapping(
        document['example'], 'example')
    return outputs


def generate_configs(source, output_dir):
    """검증한 호환 파일만 지정한 디렉터리에 기록한다."""
    outputs = load_configs(source)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    for name, document in outputs.items():
        text = '# 자동 생성: config/b2_terrain.yaml을 수정하고 다시 빌드하세요.\n'
        text += yaml.safe_dump(document, allow_unicode=True, sort_keys=False)
        target = output_dir / name
        if not target.exists() or target.read_text(encoding='utf-8') != text:
            target.write_text(text, encoding='utf-8')
    return outputs


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--input', required=True, type=Path)
    parser.add_argument('--output-dir', required=True, type=Path)
    args = parser.parse_args()
    try:
        generate_configs(args.input, args.output_dir)
    except (OSError, ValueError, yaml.YAMLError) as error:
        parser.exit(1, f'설정 생성 실패: {error}\n')


if __name__ == '__main__':
    main()
