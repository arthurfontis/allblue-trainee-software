from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    test_pwm = LaunchConfiguration('test_pwm')
    test_duration = LaunchConfiguration('test_duration')

    return LaunchDescription([
        DeclareLaunchArgument('test_pwm', default_value='1600',
                              description='PWM aplicado durante o teste (1000 a 2000)'),
        DeclareLaunchArgument('test_duration', default_value='2.0',
                              description='Duração de cada teste em segundos'),

        Node(
            package='motor_test',
            executable='motor_control_node',
            name='motor_control_node',
            output='screen',
            parameters=[{
                'test_pwm': test_pwm,
                'test_duration': test_duration,
            }],
        ),
        Node(
            package='motor_test',
            executable='gui_node',
            name='motor_test_gui',
            output='screen',
        ),
    ])
