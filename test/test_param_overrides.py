'''
Unit tests of the param_overrides merge and of its place in SimpleLauncher.node()
Run with: python3 -m pytest test/test_param_overrides.py
'''

import pytest
import yaml
from launch.actions import DeclareLaunchArgument
from launch_ros.parameter_descriptions import ParameterFile

from simple_launch import SimpleLauncher
from simple_launch.param_overrides import MergedOverrides, merge_override_files, write_merged_file


def write(tmp_path, name, content):
    path = tmp_path / name
    path.write_text(yaml.safe_dump(content))
    return str(path)


def section(**params):
    return {'ros__parameters': params}


def test_later_file_wins_and_nested_keys_merge(tmp_path):
    a = write(tmp_path, 'a.yaml', {'/**/node': section(rate=1.0, kept='a')})
    b = write(tmp_path, 'b.yaml', {'/**/node': section(rate=2.0), '/**/other': section(x=1)})

    assert merge_override_files([a, b]) == {
        '/**/node': section(rate=2.0, kept='a'),
        '/**/other': section(x=1)}
    assert merge_override_files([b, a])['/**/node'] == section(rate=1.0, kept='a')


def test_list_is_replaced_whole(tmp_path):
    a = write(tmp_path, 'a.yaml', {'/**/node': section(names=['x', 'y'])})
    b = write(tmp_path, 'b.yaml', {'/**/node': section(names=['z'])})

    assert merge_override_files([a, b])['/**/node'] == section(names=['z'])


def test_key_without_wildcard_is_refused(tmp_path):
    bad = write(tmp_path, 'bad.yaml', {'node': section(rate=1.0)})

    with pytest.raises(RuntimeError, match='/\\*\\*/<node>'):
        merge_override_files([bad])


def test_missing_file_is_refused(tmp_path):
    with pytest.raises(RuntimeError, match='not found'):
        merge_override_files([str(tmp_path / 'missing.yaml')])


def test_empty_list_gives_a_file_that_sets_nothing(tmp_path):
    path = write_merged_file([], out_dir=str(tmp_path))

    with open(path) as f:
        assert yaml.safe_load(f) == {'/**': section()}


@pytest.mark.parametrize('parameters', [None, {'a': 1}, [{'a': 1}, 'file.yaml']])
@pytest.mark.parametrize('sim_time', [None, False])
def test_node_gets_the_overrides_last(parameters, sim_time):
    sl = SimpleLauncher(use_sim_time=sim_time)
    kwargs = {} if parameters is None else {'parameters': parameters}

    node = sl.node('pkg', 'exe', **kwargs)

    # launch_ros wraps a substitution given as parameters into a ParameterFile
    params = node._Node__parameters
    assert isinstance(params[-1], ParameterFile)
    assert isinstance(params[-1].param_file[0], MergedOverrides)
    if sim_time is not None:
        # dictionary keys are normalized into tuples of TextSubstitution
        names = [key[0].text for p in params if isinstance(p, dict) for key in p]
        assert 'use_sim_time' in names


def test_launcher_does_not_declare_the_argument():
    # an included file must not hide the default given by the file that includes it
    actions = SimpleLauncher(use_sim_time=False).launch_description().entities

    assert not [action for action in actions
                if isinstance(action, DeclareLaunchArgument) and action.name == 'param_overrides']


def test_named_node_gets_its_sections_under_its_full_name(tmp_path):
    a = write(tmp_path, 'a.yaml', {'/**': section(shared=1, rate=1.0),
                                   '/**/mode_fsm': section(rate=2.0),
                                   '/**/other': section(x=1)})

    path = write_merged_file([a], out_dir=str(tmp_path), full_name='/brain/mode_fsm')

    with open(path) as f:
        assert yaml.safe_load(f) == {'/brain/mode_fsm': section(shared=1, rate=2.0)}
