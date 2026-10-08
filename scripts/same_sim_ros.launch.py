"""Publish the dashboard simulator to ROS / TF using the matching control URDF."""
from pathlib import Path
import xml.etree.ElementTree as ET
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    root = Path(__file__).resolve().parents[1]
    path = root/'external/reBotArm_control_py/urdf/RS/urdf/ReBot_Arm_RS.urdf'
    tree = ET.parse(path)
    for mesh in tree.getroot().findall('.//mesh'):
        asset = (path.parent/mesh.get('filename')).resolve(strict=True)
        mesh.set('filename', asset.as_uri())
    description = ET.tostring(tree.getroot(), encoding='unicode')
    return LaunchDescription([
        DeclareLaunchArgument('server_url', default_value='http://127.0.0.1:8765'),
        ExecuteProcess(cmd=['/usr/bin/python3',str(root/'scripts/rebot_ros_bridge.py'),
                            '--url',LaunchConfiguration('server_url')],output='screen'),
        Node(package='robot_state_publisher',executable='robot_state_publisher',
             name='rebot_sim_state_publisher',output='screen',
             parameters=[dict(robot_description=ParameterValue(description,value_type=str),
                              frame_prefix='rebot_sim/',publish_frequency=30.)],
             remappings=[('joint_states','/rebot/sim/joint_states'),
                         ('robot_description','/rebot/sim/robot_description')]),
    ])
