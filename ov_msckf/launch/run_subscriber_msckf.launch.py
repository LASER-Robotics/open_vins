import os

import launch

from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression

from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare

import yaml


def launch_setup(context: launch.LaunchContext, ld):
    executable_name = 'run_subscriber_msckf'
    namespace = 'run_subscriber_msckf'

    # #{ ov_msckf_config

    ov_msckf_config = LaunchConfiguration('ov_msckf_config')
    _ov_msckf_config = ov_msckf_config.perform(context)

    # #}

    # #{ uav_name

    uav_name = LaunchConfiguration('uav_name')

    ld.add_action(DeclareLaunchArgument(
        'uav_name',
        default_value=os.getenv('UAV_NAME', 'uav1'),
        description='Top-level namespace.'
    ))

    # #}

    # #{ standalone

    standalone = LaunchConfiguration('standalone')

    ld.add_action(DeclareLaunchArgument(
        'standalone',
        default_value='true',
        description='Whether to start as a container or load into an existing container.'
    ))

    # #}

    # #{ container_name

    container_name = LaunchConfiguration('container_name')

    ld.add_action(DeclareLaunchArgument(
        'container_name',
        default_value='',
        description='Name of an existing container to load into (if standalone is false)'
    ))

    # #}

    # #{ use_sim_time

    use_sim_time = LaunchConfiguration('use_sim_time')

    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value=PythonExpression(['"', os.getenv('REAL_UAV', 'true'), '" == "false"']),
        description='Whether use the simulation time.'
    ))

    # #}

    # #{ log_level

    log_level = LaunchConfiguration('log_level')

    ld.add_action(DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level.'
    ))

    # #}

    # #{ read yaml

    ld.add_action(
        LogInfo(msg=f'OpenVINS MSCKF configuration file: {_ov_msckf_config}')
    )

    remappings = []
    with open(_ov_msckf_config, 'r') as f:
        yaml_data = yaml.load(f, Loader=yaml.FullLoader)

        estimator_config_folder = yaml_data[executable_name]['estimator_config_folder']

        if executable_name in yaml_data and 'remappings' in yaml_data[executable_name]:
            remappings_yaml = yaml_data[executable_name]['remappings']

            for original_name in remappings_yaml:
                new_name = remappings_yaml[original_name]
                remappings.append(('~/' + original_name, new_name))

    ld.add_action(
        LogInfo(msg='OpenVINS MSCKF remappings:')
    )

    for remapping in remappings:
        ld.add_action(
            LogInfo(msg=f'\t{remapping[0]} -> {remapping[1]}')
        )

    # #}

    # #{ run subscriber msckf node

    run_subscriber_msckf_node = ComposableNode(

        package='ov_msckf',
        plugin='ov_msckf::RunSubscriberMsckf',
        namespace=uav_name,
        name='ov_msckf',

        parameters=[
            {'uav_name': uav_name},
            {'use_sim_time': use_sim_time},
            {'config_path': PathJoinSubstitution([
                FindPackageShare('ov_msckf'),
                'config',
                estimator_config_folder,
                'estimator_config.yaml'
            ])}
        ],

        remappings=remappings
    )

    load_into_existing = LoadComposableNodes(
        target_container=container_name,
        composable_node_descriptions=[run_subscriber_msckf_node],
        condition=UnlessCondition(standalone)
    )

    ld.add_action(load_into_existing)

    # #}

    # #{ standalone container

    standalone_container = ComposableNodeContainer(
        namespace=uav_name,
        name=namespace+'_container',
        package='rclcpp_components',
        executable='component_container_mt',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        composable_node_descriptions=[run_subscriber_msckf_node],
        condition=IfCondition(standalone)
    )

    ld.add_action(standalone_container)

    # #}


def generate_launch_description():
    ld = launch.LaunchDescription()

    # #{ ov_msckf_config

    ld.add_action(DeclareLaunchArgument(
        'ov_msckf_config',
        default_value='',
        description='Path to the OpenVINS MSCKF configuration file.'
    ))

    # #}

    # #{ opaque function

    ld.add_action(
        OpaqueFunction(function=launch_setup, args=[ld])
    )

    # #}

    return ld
