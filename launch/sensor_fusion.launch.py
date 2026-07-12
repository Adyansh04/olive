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

    # Global debug override (debug:=on|off). Unset leaves the per-flag values
    # in fusion.yaml untouched; on/off force every olive debug flag together so
    # a peak-performance run is one argument, not a dozen `ros2 param set`s.
    debug_arg = context.launch_configurations.get('debug', '').strip().lower()
    if debug_arg in ('on', 'true', '1', 'enable', 'enabled'):
        debug_override = True
    elif debug_arg in ('off', 'false', '0', 'disable', 'disabled'):
        debug_override = False
    else:
        debug_override = None
    fusion_debug_flags = ('publish_debug', 'debug_path', 'debug_keyframes',
                          'debug_local_map', 'debug_scan_features',
                          'debug_fiducials', 'debug_imu_state')
    vo_debug_flags = ('debug', 'publish_debug_image')

    def apply_debug_override(params, flags):
        if debug_override is not None:
            for flag in flags:
                params[flag] = debug_override
        return params

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
        params['use_vo'] = bool(modalities.get('vo', False))
        apply_debug_override(params, fusion_debug_flags)
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

    def vo_node(_config_file):
        params = apply_debug_override(dict(node_params('vo_node')), vo_debug_flags)
        return Node(
            package='olive',
            executable='vo_node',
            name='vo_node',
            output='screen',
            parameters=[params],
        )

    # modality -> Node factory. 'lio' starts the fusion core itself (the
    # LiDAR-inertial backbone); 'wheel' is an input of the core, not a node.
    node_registry = {
        'lio': fusion_node,
        'markers': whycode_detector,
        'vo': vo_node,
        'wheel': None,
    }

    actions = []
    if context.launch_configurations.get('rviz', 'false') == 'true':
        actions.append(Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_olive, 'rviz', 'fusion_debug.rviz')],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ))
    enabled = [name for name, on in modalities.items() if on]
    print(f"[olive] fusion config: {config_file}")
    print(f"[olive] enabled modalities: {', '.join(enabled) if enabled else 'none'}")
    if debug_override is not None:
        print(f"[olive] debug override: all debug flags forced "
              f"{'ON' if debug_override else 'OFF'}")

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
    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Start RViz with the fusion debug configuration'
    )
    declare_debug = DeclareLaunchArgument(
        'debug',
        default_value='',
        description='Force every olive debug flag on/off (fusion + vo). '
                    'on|off; unset keeps the per-flag values in fusion.yaml. '
                    'Use debug:=off for peak-performance runs.'
    )

    ld = LaunchDescription()
    ld.add_action(declare_config_file)
    ld.add_action(declare_rviz)
    ld.add_action(declare_debug)
    ld.add_action(OpaqueFunction(function=launch_fusion_stack))
    return ld
