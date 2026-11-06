'''
Launch settings of the nodes, read from the YAML file given by the launch_config launch argument.

The file maps patterns on the full node name (fnmatch, where * also matches /) to settings:

    nodes:
      '*':
        log_level: info
      /navigation/*:
        log_level: debug

Rules apply from top to bottom. For each setting, the last rule that matches the node and
names that setting wins: put the catch-all first, then groups, then single nodes.

An unknown setting or a value of the wrong type stops the launch. The rules that matched no
node are logged when the launch shuts down, since that is how a mistyped pattern shows.
'''

import fnmatch
import os
import re
from dataclasses import dataclass, field, fields
from typing import Dict, List, Optional, Set, Text, Tuple

import yaml
from launch.event_handlers import OnShutdown
from launch import Substitution
from launch.launch_context import LaunchContext
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node

ARG_NAME = 'launch_config'

# rules of each file already read by this launch process
_rules_of_file: Dict[Text, List[Tuple[Text, dict]]] = {}
# patterns of each file that matched at least one node
_matched_patterns: Dict[Text, Set[Text]] = {}
# whether the unmatched rules were logged
_reported = False

LOG_LEVELS = ('debug', 'info', 'warn', 'error', 'fatal')


@dataclass
class NodeSettings:
    '''
    Launch settings of one node. A setting no rule names keeps its default.
    '''

    log_level: Optional[Text] = field(default=None, metadata={'type': str, 'choices': LOG_LEVELS})
    # command the node is started with, e.g. 'gdb -batch -ex run -ex bt --args'
    prefix: Optional[Text] = field(default=None, metadata={'type': str})
    # patterns of the rules that matched the node, in file order
    rules: Tuple[Text, ...] = ()


def setting_names() -> List[Text]:
    '''
    Returns the names of the settings a rule may set
    '''
    return [f.name for f in fields(NodeSettings) if f.name != 'rules']


def load_rules(path: Text) -> List[Tuple[Text, dict]]:
    '''
    Returns the (pattern, settings) rules of a launch config file, read once per process.
    An empty path gives no rules. Raises RuntimeError on a missing file or a wrong layout.
    '''
    if not path:
        return []
    path = os.path.abspath(os.path.expanduser(path))
    if path in _rules_of_file:
        return _rules_of_file[path]

    if not os.path.isfile(path):
        raise RuntimeError(f'{ARG_NAME}: file not found: {path}')
    with open(path) as f:
        content = yaml.safe_load(f) or {}
    nodes = (content.get('nodes') or {}) if isinstance(content, dict) else None
    if not isinstance(nodes, dict) or not all(isinstance(s, dict) for s in nodes.values()):
        raise RuntimeError(f'{ARG_NAME}: {path} must map "nodes" to pattern: settings entries')

    rules = [(str(pattern), settings) for pattern, settings in nodes.items()]
    for pattern, settings in rules:
        check_settings(settings, f'{ARG_NAME}: {path}, rule {pattern!r}')
    _rules_of_file[path] = rules
    return rules


def check_settings(settings: dict, where: Text) -> None:
    '''
    Raises RuntimeError if a rule names an unknown setting or gives one a value of the wrong type
    '''
    known = {f.name: f.metadata for f in fields(NodeSettings) if f.name != 'rules'}
    unknown = sorted(set(settings) - set(known))
    if unknown:
        raise RuntimeError(f'{where}: unknown settings {unknown}, known ones are {sorted(known)}')
    for key, value in settings.items():
        expected = known[key].get('type')
        if expected is not None and not isinstance(value, expected):
            raise RuntimeError(f'{where}: {key} must be a {expected.__name__}, not {value!r}')
        choices = known[key].get('choices')
        if choices is not None and str(value).lower() not in choices:
            raise RuntimeError(f'{where}: {key} must be one of {list(choices)}, not {value!r}')


def unmatched_rules() -> List[Tuple[Text, Text]]:
    '''
    Returns the (file, pattern) rules of the files read so far that matched no node
    '''
    return [(path, pattern) for path, rules in _rules_of_file.items() for pattern, _ in rules
            if pattern not in _matched_patterns.get(path, set())]


def report_unmatched_rules() -> None:
    '''
    Logs the rules that matched no node, once per launch process: a launch can shut down
    several times over, e.g. when required nodes exit during the shutdown
    '''
    global _reported
    if _reported:
        return
    _reported = True
    logger = get_logger('simple_launch')
    for path, pattern in unmatched_rules():
        logger.info(f'{ARG_NAME}: rule {pattern!r} of {path} matched no node')


def resolve(rules: List[Tuple[Text, dict]], full_name: Text) -> NodeSettings:
    '''
    Returns the settings of the node of this full name, the last matching rule winning per setting
    '''
    settings = NodeSettings()
    names = setting_names()
    matched = []
    for pattern, values in rules:
        if not fnmatch.fnmatchcase(full_name, pattern):
            continue
        matched.append(pattern)
        for key, value in values.items():
            if key in names:
                setattr(settings, key, value)
    settings.rules = tuple(matched)
    return settings


def logger_name(full_name: Text) -> Text:
    '''
    Returns the logger name of a node, e.g. navigation.controller_server for /navigation/controller_server
    '''
    return full_name.strip('/').replace('/', '.')


class NodePrefix(Substitution):
    '''
    Prefix of a ConfiguredNode: the one of its settings, else the one its launch file gives,
    else the launch-prefix launch configuration filtered by launch-prefix-filter, as launch_ros does
    '''

    def __init__(self, given, executable):
        super().__init__()
        self.__given = None if given is None else normalize_to_list_of_substitutions(given)
        self.__executable = normalize_to_list_of_substitutions(executable)
        # set by the node, once created
        self.node = None

    def describe(self) -> Text:
        return 'NodePrefix()'

    def perform(self, context: LaunchContext) -> Text:
        settings = self.node.settings
        if settings is not None and settings.prefix:
            return settings.prefix
        if self.__given is not None:
            return perform_substitutions(context, self.__given)
        prefix_filter = context.launch_configurations.get('launch-prefix-filter', '')
        executable = os.path.basename(perform_substitutions(context, self.__executable))
        if prefix_filter and re.match(prefix_filter, executable) is None:
            return ''
        return context.launch_configurations.get('launch-prefix', '')


class ConfiguredNode(Node):
    '''
    Node whose launch settings come from the launch_config file. They are resolved when the
    node starts, because only then is its full name known (namespaces pushed by including files).
    '''

    def __init__(self, *, executable, **kwargs):
        prefix = NodePrefix(kwargs.pop('prefix', None), executable)
        super().__init__(executable=executable, prefix=prefix, **kwargs)
        prefix.node = self
        self.__executable = executable
        # the settings of this node, set when it starts
        self.settings = None

    def full_name(self, context: LaunchContext) -> Text:
        '''
        Returns the full name of the node, using its executable when it is given no name
        '''
        self._perform_substitutions(context)
        executable = os.path.basename(perform_substitutions(
            context, normalize_to_list_of_substitutions(self.__executable)))
        name = self.node_name.replace(self.UNSPECIFIED_NODE_NAME, executable)
        name = name.replace(self.UNSPECIFIED_NODE_NAMESPACE, '')
        return '/' + name.lstrip('/')

    def _configure(self, context: LaunchContext) -> None:
        '''
        Resolves the settings of this node and applies them to its command
        '''
        path = perform_substitutions(context, [LaunchConfiguration(ARG_NAME, default='')])
        full_name = self.full_name(context)
        rules = load_rules(path)
        self.settings = resolve(rules, full_name)

        if rules:
            path = os.path.abspath(os.path.expanduser(path))
            _matched_patterns.setdefault(path, set()).update(self.settings.rules)
            if not getattr(context, '_simple_launch_report', False):
                # once per launch
                context._simple_launch_report = True
                context.register_event_handler(OnShutdown(
                    on_shutdown=lambda event, context: report_unmatched_rules()))

        if self.settings.log_level:
            # given after the node's own arguments, so that it wins over them
            self.cmd.extend([[TextSubstitution(text='--log-level')],
                             [TextSubstitution(text=f'{logger_name(full_name)}:={self.settings.log_level}')]])

    def execute(self, context: LaunchContext):
        self._configure(context)
        return super().execute(context)
