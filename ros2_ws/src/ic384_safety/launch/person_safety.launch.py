from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    defaults = {
        'model_path': '', 'device': 'cpu', 'aligned_depth_confirmed': 'false',
        'stop_distance': '1.0', 'release_distance': '1.5',
        'rgb_topic': '/camera/rgb/image_raw',
        'depth_topic': '/camera/aligned_depth/image_raw', 'depth_scale': '0.001',
        'depth_min': '0.6', 'depth_max': '8.0',
        'require_hardware_feedback': 'false',
        'use_sim_time': 'false',
    }
    arguments = [DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()]
    types = {'aligned_depth_confirmed': bool, 'use_sim_time': bool,
             'require_hardware_feedback': bool,
             'stop_distance': float, 'release_distance': float, 'depth_scale': float,
             'depth_min': float, 'depth_max': float}
    parameters = {k: ParameterValue(LaunchConfiguration(k), value_type=types.get(k, str))
                  for k in defaults if k != 'require_hardware_feedback'}
    return LaunchDescription(arguments + [
        Node(package='ic384_safety', executable='safety_supervisor',
             parameters=[{'require_person_clear': True, 'auto_resume_person': True,
                          'require_hardware_feedback': ParameterValue(
                              LaunchConfiguration('require_hardware_feedback'), value_type=bool),
                          'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}]),
        Node(package='ic384_safety', executable='person_safety', parameters=[parameters]),
    ])
