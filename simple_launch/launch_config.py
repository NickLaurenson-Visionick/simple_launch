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
from ament_index_python.packages import get_package_prefix
from launch.actions import Shutdown
from launch.event_handlers import OnProcessExit, OnShutdown
from launch import Substitution
from launch.condition import Condition
from launch.launch_context import LaunchContext
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

ARG_NAME = 'launch_config'
# launch argument: true logs the settings of every node and starts none
DRY_RUN_ARG = 'dry_run'

LOG_LEVELS = ('debug', 'info', 'warn', 'error', 'fatal')


@dataclass
class NodeSettings:
    '''
    Launch settings of one node. A setting no rule names keeps its default.
    '''

    # false leaves the node out of the launch
    enabled: bool = field(default=True, metadata={'type': bool})
    log_level: Optional[Text] = field(default=None, metadata={'type': str, 'choices': LOG_LEVELS})
    # true shuts the whole launch down when the node exits
    required: bool = field(default=False, metadata={'type': bool})
    # true starts the node again when it dies, after respawn_delay seconds
    respawn: Optional[bool] = field(default=None, metadata={'type': bool})
    respawn_delay: Optional[float] = field(default=None, metadata={'type': (int, float)})
    # command the node is started with, e.g. 'gdb -batch -ex run -ex bt --args'
    prefix: Optional[Text] = field(default=None, metadata={'type': str})
    # true or {port, host}: the node waits for a debugger before main(), see debug_prefix()
    debug: object = field(default=None, metadata={'type': (bool, dict), 'keys': ('port', 'host')})
    # patterns of the rules that matched the node, in file order
    rules: Tuple[Text, ...] = ()


def setting_names() -> List[Text]:
    '''
    Returns the names of the settings a rule may set
    '''
    return [f.name for f in fields(NodeSettings) if f.name != 'rules']


def load_rules(path: Text) -> List[Tuple[Text, dict]]:
    '''
    Returns the (pattern, settings) rules of a launch config file.
    An empty path gives no rules. Raises RuntimeError on a missing file or a wrong layout.
    '''
    if not path:
        return []
    path = os.path.abspath(os.path.expanduser(path))
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
        types = expected if isinstance(expected, tuple) else (expected,)
        # YAML true is also an int, but no number setting takes it
        wrong_bool = isinstance(value, bool) and bool not in types
        if expected is not None and (wrong_bool or not isinstance(value, types)):
            names = '/'.join(t.__name__ for t in types)
            raise RuntimeError(f'{where}: {key} must be a {names}, not {value!r}')
        keys = known[key].get('keys')
        if keys is not None and isinstance(value, dict) and set(value) - set(keys):
            raise RuntimeError(f'{where}: {key} takes only {list(keys)}, not {sorted(value)}')
        choices = known[key].get('choices')
        if choices is not None and str(value).lower() not in choices:
            raise RuntimeError(f'{where}: {key} must be one of {list(choices)}, not {value!r}')


def unmatched_rules(state: Dict[Text, Tuple[list, Set[Text]]]) -> List[Tuple[Text, Text]]:
    '''
    Returns the (file, pattern) rules of a launch that matched no node
    '''
    return [(path, pattern) for path, (rules, matched) in state.items() for pattern, _ in rules
            if pattern not in matched]


def launch_state(context: LaunchContext) -> Dict[Text, Tuple[list, Set[Text]]]:
    '''
    Returns the rules this launch read, per file, with the patterns that matched a node.
    Kept on the launch context, so that each launch starts afresh. The rules that matched
    no node are logged once, when the launch shuts down.
    '''
    state = getattr(context, '_simple_launch_config', None)
    if state is None:
        state = context._simple_launch_config = {}
        logger = get_logger('simple_launch')
        context.register_event_handler(OnShutdown(
            on_shutdown=lambda event, context: [
                logger.info(f'{ARG_NAME}: rule {pattern!r} of {path} matched no node')
                for path, pattern in unmatched_rules(state)],
            handle_once=True))
    return state


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


def describe(full_name: Text, settings: NodeSettings) -> Text:
    '''
    Returns a one-line summary of the settings of a node: those that differ from the default,
    then the rules that matched
    '''
    default = NodeSettings()
    changed = [f'{f.name}={getattr(settings, f.name)!r}' for f in fields(NodeSettings)
               if f.name != 'rules' and getattr(settings, f.name) != getattr(default, f.name)]
    rules = ', '.join(settings.rules) or 'none'
    return f'{full_name}: {" ".join(changed) or "defaults"} (rules: {rules})'


def debug_prefix(package: Text, executable: Text, debug) -> Text:
    '''
    Returns the prefix that makes a node wait for a debugger before main(): gdbserver for a
    compiled executable (default port 3000), debugpy for a Python one (default port 5678).
    The default host 127.0.0.1 keeps the port off the network.
    '''
    debug = debug if isinstance(debug, dict) else {}
    host = debug.get('host', '127.0.0.1')
    path = os.path.join(get_package_prefix(package), 'lib', package, executable)
    with open(path, 'rb') as f:
        compiled = f.read(4) == b'\x7fELF'
    if compiled:
        return f'gdbserver --once {host}:{debug.get("port", 3000)}'
    return f'python3 -m debugpy --listen {host}:{debug.get("port", 5678)} --wait-for-client'


def is_dry_run(context: LaunchContext) -> bool:
    '''
    Returns whether the dry_run launch argument is true
    '''
    return perform_substitutions(
        context, [LaunchConfiguration(DRY_RUN_ARG, default='false')]).lower() == 'true'


def settings_of(context: LaunchContext, full_name: Text) -> NodeSettings:
    '''
    Returns the settings of a node from the launch_config file of this launch, and records
    which rules matched it
    '''
    path = perform_substitutions(context, [LaunchConfiguration(ARG_NAME, default='')])
    if not path:
        return resolve([], full_name)
    path = os.path.abspath(os.path.expanduser(path))
    state = launch_state(context)
    if path not in state:
        state[path] = (load_rules(path), set())
    rules, matched = state[path]
    settings = resolve(rules, full_name)
    matched.update(settings.rules)
    return settings


def log_args(settings: NodeSettings, logger: Optional[Text] = None) -> List[List[TextSubstitution]]:
    '''
    Returns the --log-level arguments of a node's settings, none if they set no level.
    Without a logger, the level is the default of the whole process, as for a node alone in it.
    '''
    if not settings.log_level:
        return []
    level = settings.log_level if logger is None else f'{logger}:={settings.log_level}'
    return [[TextSubstitution(text='--log-level')], [TextSubstitution(text=level)]]


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


class NodeRespawn(Substitution):
    '''
    Respawn flag of a ConfiguredNode: the one of its settings, else the one its launch file gives
    '''

    def __init__(self, given):
        super().__init__()
        self.__given = given
        # set by the node, once created
        self.node = None

    def describe(self) -> Text:
        return 'NodeRespawn()'

    def perform(self, context: LaunchContext) -> Text:
        settings = self.node.settings
        if settings is not None and settings.respawn is not None:
            return str(settings.respawn).lower()
        if isinstance(self.__given, bool):
            return str(self.__given).lower()
        return perform_substitutions(context, normalize_to_list_of_substitutions(self.__given))


class ConfiguredNode(Node):
    '''
    Node whose launch settings come from the launch_config file. They are resolved when the
    node starts, because only then is its full name known (namespaces pushed by including files).
    '''

    def __init__(self, *, executable, package=None, **kwargs):
        prefix = NodePrefix(kwargs.pop('prefix', None), executable)
        respawn = NodeRespawn(kwargs.pop('respawn', False))
        super().__init__(executable=executable, package=package, prefix=prefix, respawn=respawn,
                         **kwargs)
        self.__package = package
        prefix.node = self
        respawn.node = self
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
        full_name = self.full_name(context)
        self.settings = settings_of(context, full_name)

        # an empty {port, host} mapping asks for the defaults
        if self.settings.debug not in (None, False):
            executable = perform_substitutions(
                context, normalize_to_list_of_substitutions(self.__executable))
            self.settings.prefix = debug_prefix(self.__package, executable, self.settings.debug)
            # a node stopped in a debugger must not come back by itself
            self.settings.respawn = False
            get_logger('simple_launch').info(
                f'{ARG_NAME}: {full_name} waits for a debugger: {self.settings.prefix}')

        if self.settings.respawn_delay is not None:
            # launch takes the delay as a plain number at construction only
            self._ExecuteLocal__respawn_delay = float(self.settings.respawn_delay)

        # given after the node's own arguments, so that it wins over them
        self.cmd.extend(log_args(self.settings))

    def execute(self, context: LaunchContext):
        self._configure(context)
        dry_run = is_dry_run(context)
        summary = f'{ARG_NAME}: {describe(self.full_name(context), self.settings)}'
        if dry_run:
            get_logger('simple_launch').info(summary)
            return None
        get_logger('simple_launch').debug(summary)
        if not self.settings.enabled:
            get_logger('simple_launch').info(
                f'{ARG_NAME}: {self.full_name(context)} not started, enabled is false')
            return None
        self._supervise(context)
        return super().execute(context)

    def _supervise(self, context: LaunchContext) -> None:
        '''
        Shuts the launch down when this node exits, if its settings say it is required
        '''
        if self.settings.required:
            reason = f'{self.full_name(context)} exited and is required'
            context.register_event_handler(OnProcessExit(
                target_action=self, on_exit=[Shutdown(reason=reason)]))


def component_full_name(context: LaunchContext, description: ComposableNode) -> Text:
    '''
    Returns the full name of a composable node, as its container will load it. A component
    given no name is matched by the class name of its plugin.
    '''
    namespace = ''
    if description.node_namespace is not None:
        namespace = perform_substitutions(context, description.node_namespace)
    if not namespace.startswith('/'):
        namespace = (context.launch_configurations.get('ros_namespace') or '') + '/' + namespace
    if description.node_name is not None:
        name = perform_substitutions(context, description.node_name)
    else:
        name = perform_substitutions(context, description.node_plugin).split('::')[-1]
    return '/' + '/'.join(part for part in (namespace + '/' + name).split('/') if part)


class ComponentEnabled(Condition):
    '''
    Condition of a composable node: the condition it was given, and its enabled setting
    '''

    def __init__(self, given: Optional[Condition] = None):
        super().__init__(predicate=self.__evaluate)
        self.__given = given
        # set by SimpleLauncher.node(), once the description exists
        self.description = None

    def __evaluate(self, context: LaunchContext) -> bool:
        if self.__given is not None and not self.__given.evaluate(context):
            return False
        full_name = component_full_name(context, self.description)
        if not settings_of(context, full_name).enabled:
            get_logger('simple_launch').info(f'{ARG_NAME}: {full_name} not loaded, enabled is false')
            return False
        return True


class ConfiguredContainer(ConfiguredNode, ComposableNodeContainer):
    '''
    Container whose own settings come from the launch_config file, like a ConfiguredNode.
    The log levels of its components are given to its process, which holds their loggers.
    Settings that act on a process (prefix, required, respawn, debug) apply only to the container.
    '''

    def __init__(self, *, composable_node_descriptions=None, **kwargs):
        super().__init__(composable_node_descriptions=composable_node_descriptions, **kwargs)
        self.__components = composable_node_descriptions or []

    def _configure(self, context: LaunchContext) -> None:
        super()._configure(context)
        logger = get_logger('simple_launch')
        for description in self.__components:
            full_name = component_full_name(context, description)
            settings = settings_of(context, full_name)
            # the components share the container's process: each gets its own logger's level
            self.cmd.extend(log_args(settings, logger_name(full_name)))
            per_process = [key for key in ('prefix', 'required', 'respawn', 'debug')
                           if getattr(settings, key) not in (None, False)]
            if per_process:
                logger.warning(f'{ARG_NAME}: {full_name} is a component, {per_process} '
                               'apply to its container only')
            summary = f'{ARG_NAME}: {describe(full_name, settings)}, in {self.full_name(context)}'
            if is_dry_run(context):
                logger.info(summary)
            else:
                logger.debug(summary)
