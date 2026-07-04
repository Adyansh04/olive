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
from launch_ros.actions import Node


def launch_fusion_stack(context, *args, **kwargs):
    pkg_olive = get_package_share_directory('olive')

    config_file = context.launch_configurations.get('config_file') or os.path.join(
        pkg_olive, 'config', 'fusion.yaml')
    with open(config_file, 'r') as f:
        config = yaml.safe_load(f)

    modalities = config.get('modalities', {})

    # The config mixes launch-level keys (modalities) with node sections, so
    # each node receives its section as a dict rather than the raw file (the
    # strict rcl params parser rejects non-node top-level keys).
    def node_params(node_name):
        return config.get(node_name, {}).get('ros__parameters', {})

    def fusion_node(_config_file):
        params = dict(node_params('fusion_node'))
        # Modality toggles flow into the core as parameters.
        params['use_wheel_odom'] = bool(modalities.get('wheel', False))
        params['use_markers'] = bool(modalities.get('markers', False))
        return Node(
            package='olive',
            executable='fusion_node',
            name='fusion_node',
            output='screen',
            parameters=[params],
        )

    def whycode_detector(_config_file):
        return Node(
            package='whycode_vision',
            executable='whycon',
            name='whycon',
            output='screen',
            parameters=[{'config_file': os.path.join(
                pkg_olive, 'config', 'whycode_detector_sim.yaml')}],
        )

    # modality -> Node factory. 'lio' starts the fusion core itself (the
    # LiDAR-inertial backbone); 'wheel' is an input of the core, not a node.
    node_registry = {'lio': fusion_node, 'markers': whycode_detector, 'wheel': None}

    actions = []
    enabled = [name for name, on in modalities.items() if on]
    print(f"[olive] fusion config: {config_file}")
    print(f"[olive] enabled modalities: {', '.join(enabled) if enabled else 'none'}")

    for name in enabled:
        if name not in node_registry:
            print(f"[olive]   '{name}' enabled but its node is not implemented yet - skipped")
            continue
        factory = node_registry[name]
        if factory is not None:
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
