import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # --- !! YOU MUST CHANGE THIS !! ---
    # Path to your MoveIt config's launch file
    # For the panda demo:
    moveit_config_pkg =get_package_share_directory('moveit_resources_panda_moveit_config')
    moveit_launch_path = os.path.join(
        moveit_config_pkg,
        'launch',
        'demo.launch.py'
    )
    
    # 1. Get paths for our package
    pkg_share = get_package_share_directory('hybrid_planning')
    pddl_domain = os.path.join(pkg_share, 'pddl', 'hybrid_domain.pddl')
    pddl_problem = os.path.join(pkg_share, 'pddl', 'hybrid_problem.pddl')
    
    # 2. Get paths for PlanSys2
    plansys2_pkg = get_package_share_directory('plansys2_bringup')
    
    # 3. Launch MoveIt
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch_path)
    )

    # 4. Launch PlanSys2
    # This single launch file starts all 4 PlanSys2 components
    plansys2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(plansys2_pkg, 'launch', 'plansys2_bringup_launch_distributed.py')
        ),
        launch_arguments={
            'model_file': pddl_domain,
            'problem_file': pddl_problem,
        }.items()
    )

    # 5. Launch our TAMP nodes
    # This node will PROVIDE the '(pick)' action
    monitor_node = Node(
        package='hybrid_planning',
        executable='monitor_motion',
        name='pick_action_node', # The name is important
        output='screen',
        parameters=[{'action_name': 'pick'}]
    )
    
    # This node will PROVIDE the '(move_blocker)' action
    move_blocker_node = Node(
        package='hybrid_planning',
        executable='monitor_motion',
        name='move_blocker_action_node', # A second instance!
        output='screen',
        parameters=[{'action_name': 'move_blocker'}]
    )
    
    # This node will PROVIDE the '(place)' action
    place_node = Node(
        package='hybrid_planning',
        executable='monitor_motion',
        name='place_action_node', # A third instance!
        output='screen',
        parameters=[{'action_name': 'place'}]
    )

    # This node will monitor the plan and trigger replans
    replanner_node = Node(
        package='hybrid_planning',
        executable='replanner',
        name='replanner_node',
        output='screen'
    )

    return LaunchDescription([
        moveit_launch,
        plansys2_launch,
        monitor_node,
        move_blocker_node,
        place_node,
        replanner_node
    ])
