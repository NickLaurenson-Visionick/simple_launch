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
'''

import fnmatch
import os
from dataclasses import dataclass, fields
from typing import Dict, List, Optional, Text, Tuple

import yaml
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import Node

ARG_NAME = 'launch_config'

# rules of each file already read by this launch process
_rules_of_file: Dict[Text, List[Tuple[Text, dict]]] = {}


@dataclass
class NodeSettings:
    '''
    Launch settings of one node. A setting no rule names keeps its default.
    '''

    log_level: Optional[Text] = None
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
    _rules_of_file[path] = rules
    return rules


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


class ConfiguredNode(Node):
    '''
    Node whose launch settings come from the launch_config file. They are resolved when the
    node starts, because only then is its full name known (namespaces pushed by including files).
    '''

    def __init__(self, *, executable, **kwargs):
        super().__init__(executable=executable, **kwargs)
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
        self.settings = resolve(load_rules(path), full_name)

        if self.settings.log_level:
            # given after the node's own arguments, so that it wins over them
            self.cmd.extend([[TextSubstitution(text='--log-level')],
                             [TextSubstitution(text=f'{logger_name(full_name)}:={self.settings.log_level}')]])

    def execute(self, context: LaunchContext):
        self._configure(context)
        return super().execute(context)
