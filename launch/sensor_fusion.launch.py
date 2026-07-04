#!/usr/bin/env python3

"""
OLIVE sensor-fusion launch.

Reads config/fusion.yaml and starts the fusion backend plus every front-end
enabled under `modalities`. Front-end nodes are registered here as they are
implemented; a modality with no registered node is reported and skipped.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction


def launch_fusion_stack(context, *args, **kwargs):
    pkg_olive = get_package_share_directory('olive')

    config_file = context.launch_configurations.get('config_file') or os.path.join(
        pkg_olive, 'config', 'fusion.yaml')
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)

    modalities = config.get('modalities', {})

    # modality -> Node factory; populated as the fusion stack lands.
    node_registry = {}

    actions = []
    enabled = [name for name, on in modalities.items() if on]
    print(f"[olive] fusion config: {config_file}")
    print(f"[olive] enabled modalities: {', '.join(enabled) if enabled else 'none'}")

    for name in enabled:
        factory = node_registry.get(name)
        if factory is None:
            print(f"[olive]   '{name}' enabled but its node is not implemented yet - skipped")
            continue
        actions.append(factory(config_file))

    return actions


def generate_launch_description():
    declare_config_file = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='Path to a fusion.yaml override (defaults to the installed config)'
    )

    ld = LaunchDescription()
    ld.add_action(declare_config_file)
    ld.add_action(OpaqueFunction(function=launch_fusion_stack))
    return ld
