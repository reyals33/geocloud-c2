#!/usr/bin/env python3
"""
fleet_sim.launch.py  v2.4.1  (ROS2 Jazzy / Gazebo Harmonic)
═══════════════════════════════════════════════════════════════
GeoCloud Fleet Simulation — TurtleBot3 ground robots

  ├── Gazebo Harmonic (ros_gz_sim)
  ├── per-robot: ros_gz_bridge, robot_state_publisher
  ├── per-robot: slam_toolbox (online async SLAM)        [enable_slam:=true]
  ├── per-robot: Nav2 (navigation stack)                 [enable_nav2:=true]
  └── multirobot_map_merge → /global_map_2d              [optional; not in Jazzy apt]

Launch args:
  headless:=true        Skip Gazebo GUI — ~10x RTF improvement on WSL2
  enable_slam:=false    Disable SLAM toolbox per robot
  enable_nav2:=false    Disable Nav2 per robot
  use_sim_time:=false   Use wall clock instead of sim time

Changelog:
  v2.1.0  Bug fixes from 3-expert review
  v2.2.0  Ported from gazebo_ros (Classic) to ros_gz_sim (Gazebo Harmonic/Jazzy)
  v2.3.0  multirobot_map_merge and ros_gz_image made optional
  v2.4.0  Fix Gazebo Harmonic pause-on-connect race; fix DiffDrive /cmd_vel topic
          scoping; wire headless arg; gate SLAM/Nav2 on launch args; cleanup
  v2.4.1  Reviewer fixes: temp-file atexit cleanup, PackageNotFoundError,
          use_sim_time wired through, yaw passed to spawn, gzclient
          on_exit_shutdown:=false, FLEET coords as floats, nit cleanups

Author:  George O'Connor / GeoCloud Solutions LLC
"""

import atexit
import os
import tempfile

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ── Package directories ──────────────────────────────────────────────
TB3_DIR    = get_package_share_directory('turtlebot3_gazebo')
ROS_GZ_DIR = get_package_share_directory('ros_gz_sim')
NAV2_DIR   = get_package_share_directory('turtlebot3_navigation2')
SLAM_DIR   = get_package_share_directory('slam_toolbox')

WORLD_SDF  = os.path.join(TB3_DIR, 'worlds', 'turtlebot3_world.world')

# ── Optional package availability checks ────────────────────────────
def _pkg_available(pkg_name: str) -> bool:
    try:
        get_package_prefix(pkg_name)
        return True
    except PackageNotFoundError:
        return False

_HAS_MAP_MERGE = _pkg_available('multirobot_map_merge')
_HAS_GZ_IMAGE  = _pkg_available('ros_gz_image')

# ── Fleet definition ─────────────────────────────────────────────────
# Uncomment tb3_1 to add the second robot. Each entry spawns a full
# robot + bridge + (optionally) SLAM + Nav2 stack.
# Coordinates are floats; yaw is passed to both spawn (-Y) and map-merge init pose.
FLEET = [
    # {'name': 'tb3_1', 'type': 'burger', 'x': -2.0, 'y': 0.0, 'z': 0.01, 'yaw': 0.0},
    {'name': 'tb3_2', 'type': 'waffle', 'x': 2.0, 'y': 0.5, 'z': 0.01, 'yaw': 1.57},
]


def _urdf(robot_type: str) -> str:
    """Return URDF string for the given TurtleBot3 model type."""
    with open(os.path.join(TB3_DIR, 'urdf', f'turtlebot3_{robot_type}.urdf')) as f:
        return f.read()


def _patched_sdf(robot_type: str, ns: str) -> str:
    """Write a temp SDF with cmd_vel topic explicitly scoped to /model/<ns>/cmd_vel.

    Gazebo Harmonic does not auto-scope relative plugin topic names for
    dynamically spawned models, so all robots would share the bare /cmd_vel
    topic. Replacing with an absolute name gives each robot its own topic.
    The temp file is registered for deletion on process exit.
    """
    src = os.path.join(TB3_DIR, 'models', f'turtlebot3_{robot_type}', 'model.sdf')
    with open(src) as f:
        content = f.read()
    content = content.replace(
        '<topic>cmd_vel</topic>',
        f'<topic>/model/{ns}/cmd_vel</topic>',
    )
    content = content.replace(
        '<odom_topic>odom</odom_topic>',
        f'<odom_topic>/model/{ns}/odom</odom_topic>',
    )
    tmp = tempfile.NamedTemporaryFile(
        mode='w', suffix='.sdf', delete=False, prefix=f'tb3_{ns}_',
    )
    tmp.write(content)
    tmp.close()
    atexit.register(os.unlink, tmp.name)
    return tmp.name


def generate_launch_description():

    # ── Launch arguments ─────────────────────────────────────────────
    la_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true',
                      description='Use simulation (Gazebo) clock')
    la_headless = DeclareLaunchArgument('headless',     default_value='false',
                      description='Skip Gazebo GUI (recommended on WSL2)')
    la_nav2     = DeclareLaunchArgument('enable_nav2',  default_value='true')
    la_slam     = DeclareLaunchArgument('enable_slam',  default_value='true')

    sim_time = LaunchConfiguration('use_sim_time')

    # ── Gazebo resource path (models for TB3) ────────────────────────
    gz_resources = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.join(TB3_DIR, 'models'),
    )

    # ── Gazebo server (physics) ──────────────────────────────────────
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ROS_GZ_DIR, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args':          f'-r -s -v2 {WORLD_SDF}',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ── Gazebo GUI (skipped when headless:=true) ─────────────────────
    # on_exit_shutdown is false so closing the GUI window does not
    # tear down the entire simulation.
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ROS_GZ_DIR, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args':          '-g -v2',
            'on_exit_shutdown': 'false',
        }.items(),
    )

    # ── Per-robot actions ─────────────────────────────────────────────
    spawn_actions = []
    slam_actions  = []
    nav_actions   = []

    for robot in FLEET:
        ns    = robot['name']
        rtype = robot['type']

        # ── Spawn in Gazebo ─────────────────────────────────────────
        spawn_actions.append(Node(
            package='ros_gz_sim',
            executable='create',
            name=f'spawn_{ns}',
            arguments=[
                '-name', ns,
                '-file', _patched_sdf(rtype, ns),
                '-x', str(robot['x']),
                '-y', str(robot['y']),
                '-z', str(robot['z']),
                '-Y', str(robot['yaw']),
            ],
            output='screen',
        ))

        # ── robot_state_publisher ───────────────────────────────────
        # frame_prefix namespaces all TF frames: base_footprint → <ns>/base_footprint
        spawn_actions.append(Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            namespace=ns,
            parameters=[{
                'use_sim_time':      sim_time,
                'robot_description': _urdf(rtype),
                'frame_prefix':      f'{ns}/',
            }],
            output='screen',
        ))

        # ── ROS-GZ bridge ───────────────────────────────────────────
        # GZ → ROS2: odom, scan, imu, joint_states, clock
        # ROS2 → GZ: cmd_vel (Twist)
        bridge_args = [
            f'/model/{ns}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            f'/model/{ns}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            f'/model/{ns}/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            f'/model/{ns}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            f'/model/{ns}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ]
        bridge_remaps = [
            (f'/model/{ns}/odom',         f'/{ns}/odom'),
            (f'/model/{ns}/scan',         f'/{ns}/scan'),
            (f'/model/{ns}/imu',          f'/{ns}/imu'),
            (f'/model/{ns}/joint_states', f'/{ns}/joint_states'),
            (f'/model/{ns}/cmd_vel',      f'/{ns}/cmd_vel'),
        ]

        # Waffle camera: GZ topic is /camera/image_raw (no model prefix in SDF).
        # Use ros_gz_image if available (lower CPU), else fall back to parameter_bridge.
        if rtype == 'waffle' and not _HAS_GZ_IMAGE:
            bridge_args.append('/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image')
            bridge_remaps.append(('/camera/image_raw', f'/{ns}/camera/image_raw'))

        spawn_actions.append(Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name=f'bridge_{ns}',
            arguments=bridge_args,
            remappings=bridge_remaps,
            output='screen',
        ))

        if rtype == 'waffle' and _HAS_GZ_IMAGE:
            spawn_actions.append(Node(
                package='ros_gz_image',
                executable='image_bridge',
                name=f'cam_bridge_{ns}',
                arguments=['/camera/image_raw'],
                remappings=[('/camera/image_raw', f'/{ns}/camera/image_raw')],
                output='screen',
            ))

        # ── SLAM Toolbox (online async) ─────────────────────────────
        slam_actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(SLAM_DIR, 'launch', 'online_async_launch.py')
            ),
            launch_arguments={
                'use_sim_time':     sim_time,
                'slam_params_file': os.path.join(
                    SLAM_DIR, 'config', 'mapper_params_online_async.yaml'),
                'namespace': ns,
            }.items(),
        ))

        # ── Nav2 ────────────────────────────────────────────────────
        nav_actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(NAV2_DIR, 'launch', 'navigation2.launch.py')
            ),
            launch_arguments={
                'namespace':    ns,
                'use_sim_time': sim_time,
                'params_file':  os.path.join(NAV2_DIR, 'param', f'{rtype}.yaml'),
            }.items(),
        ))

    # ── Map merge (optional — not in Jazzy apt; build from source to enable) ──
    phase3_actions = []
    if _HAS_MAP_MERGE:
        map_merge_params = {
            'known_init_poses': True,
            'merging_rate':     0.5,
            'discovery_rate':   0.1,
            'estimation_rate':  0.5,
        }
        for idx, r in enumerate(FLEET, 1):
            map_merge_params[f'robot{idx}_init_pose_x']   = r['x']
            map_merge_params[f'robot{idx}_init_pose_y']   = r['y']
            map_merge_params[f'robot{idx}_init_pose_yaw'] = r['yaw']

        phase3_actions.append(Node(
            package='multirobot_map_merge',
            executable='map_merge',
            name='map_merge',
            parameters=[map_merge_params],
            remappings=[('map', '/global_map_2d')]
                      + [(f'map{i}', f'/{r["name"]}/map') for i, r in enumerate(FLEET, 1)],
            output='screen',
        ))

    # ── Assemble with staged startup ─────────────────────────────────
    return LaunchDescription([
        la_sim_time, la_headless, la_nav2, la_slam,
        gz_resources,

        # Phase 0 (t=0s): Gazebo server
        LogInfo(msg=f'[fleet_sim] Starting — map_merge: {_HAS_MAP_MERGE}, '
                    f'ros_gz_image: {_HAS_GZ_IMAGE}, '
                    + ('' if _HAS_MAP_MERGE else
                       'map_merge not found (build from source to enable), ')),
        gzserver,

        # Phase 0b (t=3s): GUI — delayed to avoid pause-on-connect race condition.
        # Gazebo Harmonic's GUI WorldControl plugin initialises paused and sends
        # pause:true to the server on first connect, overriding the -r flag.
        # Skipped when headless:=true.
        TimerAction(period=3.0, actions=[
            GroupAction(
                condition=UnlessCondition(LaunchConfiguration('headless')),
                actions=[
                    LogInfo(msg='[fleet_sim] Launching Gazebo GUI...'),
                    gzclient,
                ],
            ),
        ]),

        # Phase 1 (t=5s): Spawn robots + bridges
        TimerAction(period=5.0, actions=[
            LogInfo(msg='[fleet_sim] Spawning robots and bridges...'),
            *spawn_actions,
        ]),

        # Phase 1b (t=7s): Force-unpause physics.
        # Guards against the -r race condition and any GUI pause-on-connect override.
        TimerAction(period=7.0, actions=[
            ExecuteProcess(
                cmd=[
                    'gz', 'service',
                    '-s', '/world/default/control',
                    '--reqtype', 'gz.msgs.WorldControl',
                    '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '5000',
                    '--req', 'pause: false',
                ],
                output='screen',
                name='unpause_physics',
            ),
        ]),

        # Phase 2 (t=15s): SLAM + Nav2 — each gated by its launch arg
        TimerAction(period=15.0, actions=[
            GroupAction(
                condition=IfCondition(LaunchConfiguration('enable_slam')),
                actions=[LogInfo(msg='[fleet_sim] Starting SLAM...'), *slam_actions],
            ),
            GroupAction(
                condition=IfCondition(LaunchConfiguration('enable_nav2')),
                actions=[LogInfo(msg='[fleet_sim] Starting Nav2...'), *nav_actions],
            ),
        ]),

        # Phase 3 (t=25s): Map fusion (optional — skipped if package not installed)
        *([TimerAction(period=25.0, actions=phase3_actions)] if phase3_actions else []),
    ])
