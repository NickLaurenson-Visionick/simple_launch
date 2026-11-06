'''
Unit tests of the launch_config settings and of their effect on the nodes of SimpleLauncher.node()
Run with: python3 -m pytest test/test_launch_config.py
'''

import pytest
import yaml
from launch import LaunchContext
from launch.utilities import perform_substitutions

from simple_launch import SimpleLauncher
from simple_launch import launch_config
from simple_launch.launch_config import ConfiguredNode, load_rules, logger_name, resolve


@pytest.fixture(autouse=True)
def fresh_cache():
    launch_config._rules_of_file.clear()


def write_config(tmp_path, nodes, name='launch.yaml'):
    path = tmp_path / name
    path.write_text(yaml.safe_dump({'nodes': nodes}, sort_keys=False))
    return str(path)


def context_for(config, namespace=None):
    context = LaunchContext()
    context.launch_configurations['launch_config'] = config
    if namespace is not None:
        context.launch_configurations['ros_namespace'] = namespace
    return context


def command(node, context):
    # Node.execute() fills these in before the command is expanded
    context.extend_locals({'ros_specific_arguments': {'name': '__node:=n', 'ns': '__ns:=/ns'}})
    return [perform_substitutions(context, part) for part in node.cmd[1:]]


def test_last_matching_rule_wins_per_setting(tmp_path):
    rules = load_rules(write_config(tmp_path, {
        '*': {'log_level': 'info'},
        '/navigation/*': {'log_level': 'debug'},
        '/navigation/planner_server': {},
    }))

    assert resolve(rules, '/navigation/controller_server').log_level == 'debug'
    assert resolve(rules, '/brain/mode_fsm').log_level == 'info'
    # a rule that names no setting keeps the earlier values
    planner = resolve(rules, '/navigation/planner_server')
    assert planner.log_level == 'debug'
    assert planner.rules == ('*', '/navigation/*', '/navigation/planner_server')


def test_no_file_gives_no_rules():
    assert load_rules('') == []
    assert resolve([], '/a').log_level is None


def test_wrong_layout_is_refused(tmp_path):
    path = tmp_path / 'bad.yaml'
    path.write_text(yaml.safe_dump({'nodes': {'*': 'debug'}}))

    with pytest.raises(RuntimeError, match='pattern: settings'):
        load_rules(str(path))
    with pytest.raises(RuntimeError, match='not found'):
        load_rules(str(tmp_path / 'missing.yaml'))


def test_logger_name():
    assert logger_name('/navigation/controller_server') == 'navigation.controller_server'
    assert logger_name('/rviz2') == 'rviz2'


def test_sl_node_creates_a_configured_node():
    assert isinstance(SimpleLauncher().node('pkg', 'exe'), ConfiguredNode)


@pytest.mark.parametrize('name, namespace, full_name', [
    ('ctrl', '/navigation', '/navigation/ctrl'),
    (None, '/navigation', '/navigation/exe'),
    ('ctrl', None, '/ctrl'),
])
def test_full_name_includes_the_pushed_namespace(name, namespace, full_name):
    node = ConfiguredNode(package='pkg', executable='exe', name=name)

    assert node.full_name(context_for('', namespace)) == full_name


def test_log_level_is_appended_after_the_node_arguments(tmp_path):
    config = write_config(tmp_path, {'/navigation/*': {'log_level': 'debug'}})
    node = ConfiguredNode(package='pkg', executable='exe', name='ctrl',
                          ros_arguments=['--log-level', 'warn'])
    context = context_for(config, '/navigation')

    node._configure(context)

    assert command(node, context)[-2:] == ['--log-level', 'navigation.ctrl:=debug']
    assert '--log-level' in command(node, context)[:-2]


def test_node_without_matching_rule_is_unchanged(tmp_path):
    config = write_config(tmp_path, {'/navigation/*': {'log_level': 'debug'}})
    node = ConfiguredNode(package='pkg', executable='exe', name='mode_fsm')
    context = context_for(config, '/brain')

    node._configure(context)

    assert '--log-level' not in command(node, context)
