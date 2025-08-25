import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from pathlib import Path

def extract_dds_config(context) -> list:
    """
    从配置文件提取DDS配置，生成全局环境变量动作（所有节点自动继承）
    :param context: Launch上下文（用于获取配置文件路径）
    :return: DDS环境变量设置动作列表
    """
    #获取配置文件路径（支持通过launch参数传入，默认使用crazyswarm2_config.yaml）
    
    config_path = LaunchConfiguration('dds_config_file').perform(context)
    if not Path(config_path).exists():
        raise FileNotFoundError(f"DDS配置文件不存在: {config_path}")
    
    #加载YAML，提取/crazyflie_server下的dds字段（与原有配置结构对齐）
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    dds_config = config.get('/crazyflie_server', {}).get('dds', {})
    
    # 设置DDS默认值（防止配置缺失）
    default_dds = {
        'rmw_implementation': 'rmw_cyclonedds_cpp',  # 默认CycloneDDS
        'domain_id': 0                               # 默认域ID
    }
    # 合并配置（用户配置覆盖默认值）
    merged_dds = {**default_dds, **dds_config}
    
    return [
        SetEnvironmentVariable('RMW_IMPLEMENTATION', merged_dds['rmw_implementation']),
        SetEnvironmentVariable('ROS_DOMAIN_ID', str(merged_dds['domain_id']))
    ]

def parse_yaml(context):
    # Load the crazyflies YAML file
    crazyflies_yaml = LaunchConfiguration('crazyflies_yaml_file').perform(context)
    with open(crazyflies_yaml, 'r') as file:
        crazyflies = yaml.safe_load(file)
    # store the fileversion
    fileversion = 1
    if "fileversion" in crazyflies:
        fileversion = crazyflies["fileversion"]

    # server params
    server_yaml = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'server.yaml')

    with open(server_yaml, 'r') as ymlfile:
        server_yaml_content = yaml.safe_load(ymlfile)

    server_params = [crazyflies] + [server_yaml_content['/crazyflie_server']['ros__parameters']]
    # robot description
    urdf = os.path.join(
        get_package_share_directory('crazyflie'),
        'urdf',
        'crazyflie_description.urdf')
    
    with open(urdf, 'r') as f:
        robot_desc = f.read()

    server_params[1]['robot_description'] = robot_desc

    # construct motion_capture_configuration
    motion_capture_yaml = LaunchConfiguration('motion_capture_yaml_file').perform(context)
    with open(motion_capture_yaml, 'r') as ymlfile:
        motion_capture_content = yaml.safe_load(ymlfile)

    motion_capture_params = motion_capture_content['/motion_capture_tracking']['ros__parameters']
    motion_capture_params['rigid_bodies'] = dict()
    for key, value in crazyflies['robots'].items():
        type = crazyflies['robot_types'][value['type']]
        if value['enabled'] and \
            ((fileversion == 1 and type['motion_capture']['enabled']) or \
            ((fileversion >= 2 and type['motion_capture']['tracking'] == "librigidbodytracker"))):
            motion_capture_params['rigid_bodies'][key] =  {
                    'initial_position': value['initial_position'],
                    'marker': type['motion_capture']['marker'],
                    'dynamics': type['motion_capture']['dynamics'],
                }

    # copy relevent settings to server params
    server_params[1]['poses_qos_deadline'] = motion_capture_params['topics']['poses']['qos']['deadline']
    
    return [
        Node(
            package='motion_capture_tracking',
            executable='motion_capture_tracking_node',
            condition=IfCondition(PythonExpression(["'", LaunchConfiguration('backend'), "' != 'sim' and '", LaunchConfiguration('mocap'), "' == 'True'"])),
            name='motion_capture_tracking',
            output='screen',
            parameters= [motion_capture_params],
        ),
        Node(
            package='crazyflie',
            executable='crazyflie_server.py',
            condition=LaunchConfigurationEquals('backend','cflib'),
            name='crazyflie_server',
            output='screen',
            parameters= server_params,
        ),
        Node(
            package='crazyflie',
            executable='crazyflie_server',
            condition=LaunchConfigurationEquals('backend','cpp'),
            name='crazyflie_server',
            output='screen',
            parameters= server_params,
            prefix=PythonExpression(['"gdbserver localhost:13000" if ', LaunchConfiguration('debug'), ' else ""']),
        ),
        Node(
            package='crazyflie_sim',
            executable='crazyflie_server',
            condition=LaunchConfigurationEquals('backend','sim'),
            name='crazyflie_server',
            output='screen',
            emulate_tty=True,
            parameters= server_params,
        )]


def launch_main(context):
    dds_actions = extract_dds_config(context)
    parse_yaml_nodes = parse_yaml(context)
    return dds_actions + parse_yaml_nodes


def generate_launch_description():
    default_crazyflies_yaml_path = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'crazyflies.yaml')
    
    default_motion_capture_yaml_path = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'motion_capture.yaml')

    default_rviz_config_path = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'config.rviz')

    telop_yaml_path = os.path.join(
        get_package_share_directory('crazyflie'),
        'config',
        'teleop.yaml')
    
    
    default_dds_config_path = os.path.join(
        get_package_share_directory('crazyflie'),  # 若配置在其他包，修改为对应包名
        'config',
        'server.yaml'  # 你的DDS配置文件路径
    )
    
    return LaunchDescription([
        # 原有参数声明（完全保留）
        DeclareLaunchArgument('crazyflies_yaml_file', 
                              default_value=default_crazyflies_yaml_path),
        DeclareLaunchArgument('motion_capture_yaml_file', 
                              default_value=default_motion_capture_yaml_path),
        DeclareLaunchArgument('rviz_config_file', 
                              default_value=default_rviz_config_path),
        DeclareLaunchArgument('backend', default_value='cpp'),
        DeclareLaunchArgument('debug', default_value='False'),
        DeclareLaunchArgument('rviz', default_value='False'),
        DeclareLaunchArgument('gui', default_value='False'),
        DeclareLaunchArgument('qgc', default_value='False'),
        DeclareLaunchArgument('teleop', default_value='False'),
        DeclareLaunchArgument('mocap', default_value='True'),
        DeclareLaunchArgument('teleop_yaml_file', default_value=''),
        
        DeclareLaunchArgument('dds_config_file',
                              default_value=default_dds_config_path,
                              description='DDS配置文件路径（含rmw_implementation和domain_id）'),
        
        #用launch_main整合DDS与原有节点
        OpaqueFunction(function=launch_main),
        
        Node(
            condition=LaunchConfigurationEquals('teleop', 'True'),
            package='crazyflie',
            executable='teleop',
            name='teleop',
            remappings=[
                ('emergency', 'all/emergency'),
                ('arm', 'all/arm'),
                ('takeoff', 'all/takeoff'),
                ('land', 'all/land'),
            ],
            parameters= [PythonExpression(["'" + telop_yaml_path +"' if '", LaunchConfiguration('teleop_yaml_file'), "' == '' else '", LaunchConfiguration('teleop_yaml_file'), "'"])],
        ),
        Node(
            condition=LaunchConfigurationEquals('teleop', 'True'),
            package='joy',
            executable='joy_node',
            name='joy_node'
        ),
        Node(
            condition=LaunchConfigurationEquals('rviz', 'True'),
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config_file')],
            parameters=[{
                "use_sim_time": PythonExpression(["'", LaunchConfiguration('backend'), "' == 'sim'"]),
            }]
        ),
        Node(
            condition=LaunchConfigurationEquals('gui', 'True'),
            package='crazyflie',
            namespace='',
            executable='gui.py',
            name='gui',
            parameters=[{
                "use_sim_time": PythonExpression(["'", LaunchConfiguration('backend'), "' == 'sim'"]),
            }]
        ),
        Node(
            condition=LaunchConfigurationEquals('qgc', 'True'),
            package='nokov_swarm',
            namespace='',
            executable='nokov_swarm_node',
            name='qgc',
            emulate_tty=True,
            output='screen'
        ),
    ])
