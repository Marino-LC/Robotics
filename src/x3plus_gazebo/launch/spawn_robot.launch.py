import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_description = get_package_share_directory('x3plus_description')
    
    xacro_file = os.path.join(pkg_description, 'urdf', 'x3plus.urdf.xacro')

    robot_description_content = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', xacro_file]), 
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True
        }]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU'
        ],
        output='screen'
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', 
                   '-name', 'rosmaster_x3plus',
                   '-z', '0.15'],
        output='screen'
    )

    # Definición de los Controladores
    load_jsb = Node(
        package="controller_manager", executable="spawner", arguments=["joint_state_broadcaster"],
    )
    load_base = Node(
        package="controller_manager", executable="spawner", arguments=["omni_base_controller"],
    )
    load_arm = Node(
        package="controller_manager", executable="spawner", arguments=["arm_controller"], 
    )
    load_gripper = Node(
        package="controller_manager", executable="spawner", arguments=["dofbot_gripper_controller"],
    )

    # === EL SEMÁFORO SECUENCIAL ===
    # 1. Cuando el robot aparezca, inicia el JSB
    jsb_event = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=spawn_entity, on_exit=[load_jsb])
    )
    # 2. Cuando el JSB termine de cargar, inicia la base
    base_event = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=load_jsb, on_exit=[load_base])
    )
    # 3. Cuando la base cargue, inicia el brazo
    arm_event = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=load_base, on_exit=[load_arm])
    )
    # 4. Cuando el brazo cargue, inicia la pinza
    gripper_event = RegisterEventHandler(
        event_handler=OnProcessExit(target_action=load_arm, on_exit=[load_gripper])
    )

    return LaunchDescription([
        gazebo,
        clock_bridge,
        robot_state_publisher,
        spawn_entity,
        jsb_event,
        base_event,
        arm_event,
        gripper_event
    ])
