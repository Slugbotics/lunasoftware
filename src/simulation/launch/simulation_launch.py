import os
import shutil
import launch
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
import sys

def generate_launch_description():
    package_dir = get_package_share_directory('simulation')
    robot_description_path = os.path.join(package_dir, 'resource', 'slugbot.urdf')
    world_path = os.path.join(package_dir, 'worlds', 'my_world.wbt')
    # Start the simulation controller as a regular ROS2 node (entrypoint)
    # Detect WSL environment and native webots binary similar to URC2026
    is_wsl = 'microsoft-standard' in os.uname().release
    linux_webots = shutil.which('webots')

    if is_wsl and linux_webots:
        webots = ExecuteProcess(
            output='screen',
            cmd=[
                linux_webots,
                '--port=1234',
                world_path,
                '--batch',
                ['--mode=', 'realtime'],
            ],
            name='webots'
        )

        # controller_script = os.path.join(
        #     get_package_share_directory('webots_ros2_driver'), 'scripts', 'webots-controller'
        # )
        # webots_controller = ExecuteProcess(
        #     output='screen',
        #     cmd=[
        #         controller_script,
        #         '--robot-name=slugbot',
        #         '--protocol=ipc',
        #         '--port=1234',
        #         'ros2',
        #         '--ros-args',
        #         '-p', f'robot_description:={robot_description_path}',
        #     ],
        #     name='webots_controller',
        #     additional_env={'WEBOTS_HOME': get_package_prefix('webots_ros2_driver')}
        # )
    else:
        webots = WebotsLauncher(world=world_path)

        # webots_controller = WebotsController(
        #     robot_name='slugbot',
        #     parameters=[
        #         {'robot_description': robot_description_path},
        #     ]
        # )

    # Start the Python simulation script directly (installed console script).
    # Using ExecuteProcess avoids the ament/libexec lookup that fails for
    # pure python console-scripts installed to `install/bin`.
    simulation_script = os.path.join(get_package_prefix('simulation'), 'bin', 'simulation')
    simulation_node = ExecuteProcess(
        output='screen',
        cmd=[simulation_script, '--ros-args', '-p', f'robot_description:={robot_description_path}'],
        name='simulation_node',
        additional_env={'WEBOTS_HOME': get_package_prefix('webots_ros2_driver')}
    )

    lunacontroller_script = os.path.join(get_package_prefix('lunacontroller'), 'lib', 'lunacontroller', 'main')
    lunacontroller_node = ExecuteProcess(
        output='screen',
        cmd=[lunacontroller_script],
        name='lunacontroller_node',
        additional_env={'GPIOZERO_PIN_FACTORY': 'mock'}
    )

    # `webots_controller` is created above in the WSL and non-WSL branches.
    # Do not recreate or overwrite it here; use the earlier value so the
    # parameter handling matches the working URC launch.

    return LaunchDescription([
        webots,
        # webots_controller,
        simulation_node,
        lunacontroller_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        )
    ])
