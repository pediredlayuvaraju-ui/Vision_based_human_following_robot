import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share = get_package_share_directory('human_following_robot')
    tb3_gazebo_share = get_package_share_directory('turtlebot3_gazebo')

    world_path = os.path.join(
        pkg_share,
        'worlds',
        'hospital_human_world.world'
    )

    gazebo_model_path = (
        os.path.join(tb3_gazebo_share, 'models')
        + ':'
        + '/usr/share/gazebo-11/models'
        + ':'
        + os.environ.get('GAZEBO_MODEL_PATH', '')
    )

    gazebo = ExecuteProcess(
        cmd=[
            'gazebo',
            '--verbose',
            world_path,
            '-s',
            'libgazebo_ros_init.so'
        ],
        output='screen'
    )

    return LaunchDescription([
        SetEnvironmentVariable(
            name='TURTLEBOT3_MODEL',
            value='waffle_pi'
        ),

        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=gazebo_model_path
        ),

        SetEnvironmentVariable(
            name='GAZEBO_MODEL_DATABASE_URI',
            value=''
        ),

        gazebo
    ])
