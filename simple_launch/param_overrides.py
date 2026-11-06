'''
Parameter override files given to every node through the param_overrides launch argument.

The argument is a space-separated list of YAML parameter files. They are merged in order,
a later file winning over an earlier one, and the result is passed last to each node,
after its own parameters.
'''

import os
import tempfile
from typing import Dict, List, Text

import yaml
from launch import Substitution
from launch.launch_context import LaunchContext
from launch.logging import get_logger, launch_config
from launch.substitutions import LaunchConfiguration
from launch.utilities import perform_substitutions

ARG_NAME = 'param_overrides'

# merged file of each list of files already seen by this launch process
_merged_files: Dict[tuple, str] = {}


def _is_wildcard_key(key) -> bool:
    return key == '/**' or (isinstance(key, str) and key.startswith('/**/'))


def _deep_merge(base: dict, update: dict) -> dict:
    '''
    Merges update into base: nested dictionaries are merged, any other value is replaced
    '''
    for key, value in update.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _deep_merge(base[key], value)
        else:
            base[key] = value
    return base


def merge_override_files(paths: List[Text]) -> dict:
    '''
    Loads the given parameter files and merges them in order, the last one winning.
    Every top-level key must be '/**' or '/**/<node>': two spellings of the same node
    would stay separate sections whose relative order is not defined.
    Raises RuntimeError on a missing file, a file that is not a mapping or a wrong key.
    '''
    merged = {}
    for path in paths:
        if not os.path.isfile(path):
            raise RuntimeError(f'{ARG_NAME}: file not found: {path}')
        with open(path) as f:
            content = yaml.safe_load(f) or {}
        if not isinstance(content, dict):
            raise RuntimeError(f'{ARG_NAME}: {path} is not a YAML mapping')
        wrong = [key for key in content if not _is_wildcard_key(key)]
        if wrong:
            raise RuntimeError(
                f'{ARG_NAME}: {path} has keys {wrong}, key node sections as "/**/<node>"')
        _deep_merge(merged, content)
    return merged


def write_merged_file(paths: List[Text], out_dir: Text = None) -> Text:
    '''
    Merges the given files and writes the result, once per list of files and process.
    Returns the path of the written file. An empty list gives a file that sets nothing.
    '''
    key = tuple(paths)
    if key in _merged_files:
        return _merged_files[key]

    merged = merge_override_files(paths)
    if not merged:
        # a parameter file must hold at least one section
        merged = {'/**': {'ros__parameters': {}}}

    if out_dir is None:
        out_dir = launch_config.log_dir
    os.makedirs(out_dir, exist_ok=True)
    fd, out_path = tempfile.mkstemp(prefix=f'{ARG_NAME}_', suffix='.yaml', dir=out_dir)
    with os.fdopen(fd, 'w') as f:
        f.write(''.join(f'# {path}\n' for path in paths))
        yaml.safe_dump(merged, f, default_flow_style=False)

    if paths:
        get_logger('simple_launch').info(f'{ARG_NAME}: {" ".join(paths)} merged into {out_path}')
    _merged_files[key] = out_path
    return out_path


class MergedOverrides(Substitution):
    '''
    Substitution that resolves to the merged file of the param_overrides launch argument
    '''

    def __init__(self):
        super().__init__()
        self.__files = LaunchConfiguration(ARG_NAME, default='')

    def describe(self) -> Text:
        return f'MergedOverrides({ARG_NAME})'

    def perform(self, context: LaunchContext) -> Text:
        files = perform_substitutions(context, [self.__files]).split()
        return write_merged_file([os.path.abspath(os.path.expanduser(f)) for f in files])
