"""MoveIt with mock, Isaac Sim topic control, or existing trajectory controllers."""
from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def setup(context):
    get=lambda n: LaunchConfiguration(n).perform(context)
    mode=get('hardware'); pipeline=get('pipeline')
    sim=get('use_sim_time').lower()=='true'
    share=Path(get_package_share_directory('xsting_moveit_config'))
    config=share/'config'
    load=lambda n: yaml.safe_load((config/n).read_text())
    robot=ET.parse(config/'xsting_manipulation.urdf').getroot()
    initial=load('xsting.xrdf')['default_joint_positions']
    if mode!='external':
        control=ET.SubElement(robot,'ros2_control',name='xsting_planning',type='system')
        hw=ET.SubElement(control,'hardware')
        ET.SubElement(hw,'plugin').text=('mock_components/GenericSystem' if mode=='mock' else 'topic_based_ros2_control/TopicBasedSystem')
        if mode=='isaac':
            ET.SubElement(hw,'param',name='joint_commands_topic').text='/isaac_joint_commands'
            ET.SubElement(hw,'param',name='joint_states_topic').text='/isaac_joint_states'
        for name,q in initial.items():
            j=ET.SubElement(control,'joint',name=name)
            if name!='finger_r': ET.SubElement(j,'command_interface',name='position')
            state=ET.SubElement(j,'state_interface',name='position')
            ET.SubElement(state,'param',name='initial_value').text=str(q)
            ET.SubElement(j,'state_interface',name='velocity')
    description={'robot_description':ET.tostring(robot,encoding='unicode')}
    params={**description,'robot_description_semantic':(config/'xsting.srdf').read_text(),
            'robot_description_kinematics':load('kinematics.yaml'),
            'robot_description_planning':load('joint_limits.yaml'),
            'planning_pipelines':['ompl'],'default_planning_pipeline':'ompl',
            'ompl':load('ompl_planning.yaml'), 'use_sim_time':sim,
            'publish_robot_description_semantic':True,
            'publish_planning_scene':True,'publish_geometry_updates':True,
            'publish_state_updates':True,'publish_transforms_updates':True,
            **load('moveit_controllers.yaml')}
    nodes=[]
    if pipeline=='cumotion':
        cm=Path(get_package_share_directory('isaac_ros_cumotion_moveit'))
        params['isaac_ros_cumotion']=yaml.safe_load((cm/'config/isaac_ros_cumotion_planning.yaml').read_text())
        params['planning_pipelines'].append('isaac_ros_cumotion')
        params['default_planning_pipeline']='isaac_ros_cumotion'
        nodes.append(ComposableNodeContainer(name='cumotion_container',namespace='',
            package='rclcpp_components',executable='component_container_mt',output='screen',
            composable_node_descriptions=[
              ComposableNode(name='static_planning_scene_server',package='isaac_ros_cumotion',
                plugin='nvidia::isaac_ros::cumotion::StaticPlanningSceneServer'),
              ComposableNode(name='cumotion_planner',package='isaac_ros_cumotion',
                plugin='nvidia::isaac_ros::cumotion::CumotionPlanner',parameters=[{
                  'urdf_file_path':str(config/'xsting_manipulation.urdf'),
                  'xrdf_file_path':str(config/'xsting.xrdf'),'joint_states_topic':'/joint_states',
                  'use_sim_time':sim,'time_dilation_factor':0.2}])]))
    nodes.extend([
        Node(package='robot_state_publisher',executable='robot_state_publisher',parameters=[description,{'use_sim_time':sim}],output='screen'),
        Node(package='moveit_ros_move_group',executable='move_group',parameters=[params],output='screen')])
    if mode!='external':
        nodes.append(Node(package='controller_manager',executable='ros2_control_node',
            parameters=[str(config/'ros2_controllers.yaml'),{'use_sim_time':sim}],
            remappings=[('~/robot_description','/robot_description')],output='screen'))
        nodes.append(Node(package='controller_manager',executable='spawner',arguments=[
            'joint_state_broadcaster','arm_trajectory_controller','gripper_trajectory_controller',
            '--param-file',str(config/'ros2_controllers.yaml'),'--controller-manager-timeout','60'],output='screen'))
    if get('rviz').lower()=='true':
        nodes.append(Node(package='rviz2',executable='rviz2',arguments=['-d',str(config/'moveit.rviz')],parameters=[params]))
    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('hardware',default_value='mock',choices=['mock','isaac','external']),
        DeclareLaunchArgument('pipeline',default_value='ompl',choices=['ompl','cumotion']),
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        DeclareLaunchArgument('rviz',default_value='true'),
        OpaqueFunction(function=setup)])
