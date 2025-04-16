#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch everything needed for navigation visualization in RViz.
    This includes the main navigation stack and an RViz instance for visualization.
    """
    # Get the launch directory
    pkg_dir = get_package_share_directory('robot_navigation')
    
    # Create RViz configuration for Nav2
    rviz_config_path = os.path.join(pkg_dir, 'config', 'nav2_config.rviz')
    if not os.path.exists(os.path.dirname(rviz_config_path)):
        os.makedirs(os.path.dirname(rviz_config_path))
    
    # Default RViz configuration with Nav2 plugin
    rviz_config = """
    Panels:
      - Class: rviz_common/Displays
        Help Height: 0
        Name: Displays
        Property Tree Widget:
          Expanded:
            - /Global Options1
            - /TF1/Frames1
            - /TF1/Tree1
            - /Global Costmap1/Costmap1
            - /Local Costmap1/Costmap1
            - /Global Planner1/Path1
            - /Local Planner1/Path1
          Splitter Ratio: 0.5833333134651184
        Tree Height: 464
      - Class: rviz_common/Selection
        Name: Selection
      - Class: rviz_common/Tool Properties
        Expanded:
          - /Publish Point1
        Name: Tool Properties
        Splitter Ratio: 0.5886790156364441
      - Class: rviz_common/Views
        Expanded:
          - /Current View1
        Name: Views
        Splitter Ratio: 0.5
      - Class: nav2_rviz_plugins/Navigation 2
        Name: Navigation 2
    Visualization Manager:
      Class: ""
      Displays:
        - Alpha: 0.5
          Cell Size: 1
          Class: rviz_default_plugins/Grid
          Color: 160; 160; 164
          Enabled: true
          Line Style:
            Line Width: 0.029999999329447746
            Value: Lines
          Name: Grid
          Normal Cell Count: 0
          Offset:
            X: 0
            Y: 0
            Z: 0
          Plane: XY
          Plane Cell Count: 10
          Reference Frame: <Fixed Frame>
          Value: true
        - Alpha: 1
          Class: rviz_default_plugins/RobotModel
          Collision Enabled: false
          Description File: ""
          Description Source: Topic
          Description Topic:
            Depth: 5
            Durability Policy: Volatile
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /robot_description
          Enabled: true
          Links:
            All Links Enabled: true
            Expand Joint Details: false
            Expand Link Details: false
            Expand Tree: false
            Link Tree Style: Links in Alphabetic Order
          Name: RobotModel
          TF Prefix: ""
          Update Interval: 0
          Value: true
          Visual Enabled: true
        - Class: rviz_default_plugins/TF
          Enabled: true
          Frame Timeout: 15
          Frames:
            All Enabled: false
          Marker Scale: 1
          Name: TF
          Show Arrows: true
          Show Axes: true
          Show Names: false
          Tree:
            {}
          Update Interval: 0
          Value: true
        - Alpha: 1
          Autocompute Intensity Bounds: true
          Autocompute Value Bounds:
            Max Value: 10
            Min Value: -10
            Value: true
          Axis: Z
          Channel Name: intensity
          Class: rviz_default_plugins/LaserScan
          Color: 255; 0; 0
          Color Transformer: FlatColor
          Decay Time: 0
          Enabled: true
          Invert Rainbow: false
          Max Color: 255; 255; 255
          Max Intensity: 4096
          Min Color: 0; 0; 0
          Min Intensity: 0
          Name: LaserScan
          Position Transformer: XYZ
          Selectable: true
          Size (Pixels): 3
          Size (m): 0.05000000074505806
          Style: Squares
          Topic:
            Depth: 5
            Durability Policy: Volatile
            History Policy: Keep Last
            Reliability Policy: Best Effort
            Value: /scan
          Use Fixed Frame: true
          Use rainbow: true
          Value: true
        - Alpha: 1
          Class: rviz_default_plugins/Map
          Color Scheme: map
          Draw Behind: true
          Enabled: true
          Name: Map
          Topic:
            Depth: 1
            Durability Policy: Transient Local
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /map
          Update Topic:
            Depth: 5
            Durability Policy: Volatile
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /map_updates
          Use Timestamp: false
          Value: true
        - Alpha: 0.699999988079071
          Class: rviz_default_plugins/Map
          Color Scheme: costmap
          Draw Behind: false
          Enabled: true
          Name: Global Costmap
          Topic:
            Depth: 1
            Durability Policy: Transient Local
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /global_costmap/costmap
          Update Topic:
            Depth: 5
            Durability Policy: Volatile
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /global_costmap/costmap_updates
          Use Timestamp: false
          Value: true
        - Alpha: 0.699999988079071
          Class: rviz_default_plugins/Map
          Color Scheme: costmap
          Draw Behind: false
          Enabled: true
          Name: Local Costmap
          Topic:
            Depth: 1
            Durability Policy: Transient Local
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /local_costmap/costmap
          Update Topic:
            Depth: 5
            Durability Policy: Volatile
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /local_costmap/costmap_updates
          Use Timestamp: false
          Value: true
        - Alpha: 1
          Buffer Length: 1
          Class: rviz_default_plugins/Path
          Color: 0; 0; 255
          Enabled: true
          Head Diameter: 0.30000001192092896
          Head Length: 0.20000000298023224
          Length: 0.30000001192092896
          Line Style: Lines
          Line Width: 0.029999999329447746
          Name: Global Planner
          Offset:
            X: 0
            Y: 0
            Z: 0
          Pose Color: 255; 85; 255
          Pose Style: None
          Radius: 0.029999999329447746
          Shaft Diameter: 0.10000000149011612
          Shaft Length: 0.10000000149011612
          Topic:
            Depth: 5
            Durability Policy: Volatile
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /plan
          Value: true
        - Alpha: 1
          Buffer Length: 1
          Class: rviz_default_plugins/Path
          Color: 0; 255; 0
          Enabled: true
          Head Diameter: 0.30000001192092896
          Head Length: 0.20000000298023224
          Length: 0.30000001192092896
          Line Style: Lines
          Line Width: 0.029999999329447746
          Name: Local Planner
          Offset:
            X: 0
            Y: 0
            Z: 0
          Pose Color: 255; 85; 255
          Pose Style: None
          Radius: 0.029999999329447746
          Shaft Diameter: 0.10000000149011612
          Shaft Length: 0.10000000149011612
          Topic:
            Depth: 5
            Durability Policy: Volatile
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /local_plan
          Value: true
        - Alpha: 1
          Class: rviz_default_plugins/Polygon
          Color: 25; 255; 0
          Enabled: true
          Name: Polygon
          Topic:
            Depth: 5
            Durability Policy: Volatile
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /local_costmap/published_footprint
          Value: true
      Enabled: true
      Global Options:
        Background Color: 48; 48; 48
        Fixed Frame: map
        Frame Rate: 30
      Name: root
      Tools:
        - Class: rviz_default_plugins/MoveCamera
        - Class: rviz_default_plugins/Select
        - Class: rviz_default_plugins/FocusCamera
        - Class: rviz_default_plugins/Measure
          Line color: 128; 128; 0
        - Class: rviz_default_plugins/PublishPoint
          Single click: true
          Topic:
            Depth: 5
            Durability Policy: Volatile
            History Policy: Keep Last
            Reliability Policy: Reliable
            Value: /clicked_point
        - Class: nav2_rviz_plugins/GoalTool
      Transformation:
        Current:
          Class: rviz_default_plugins/TF
      Value: true
      Views:
        Current:
          Angle: 0
          Class: rviz_default_plugins/TopDownOrtho
          Enable Stereo Rendering:
            Stereo Eye Separation: 0.05999999865889549
            Stereo Focal Distance: 1
            Swap Stereo Eyes: false
            Value: false
          Invert Z Axis: false
          Name: Current View
          Near Clip Distance: 0.009999999776482582
          Scale: 100
          Target Frame: <Fixed Frame>
          Value: TopDownOrtho (rviz_default_plugins)
          X: 0
          Y: 0
        Saved: ~
    Window Geometry:
      Displays:
        collapsed: false
      Height: 914
      Hide Left Dock: false
      Hide Right Dock: false
      Navigation 2:
        collapsed: false
      QMainWindow State: 000000ff00000000fd00000004000000000000016a000002f4fc0200000009fb0000001200530065006c0065006300740069006f006e00000001e10000009b0000005c00fffffffb0000001e0054006f006f006c002000500072006f007000650072007400690065007302000001ed000001df00000185000000a3fb000000120056006900650077007300200054006f006f02000001df000002110000018500000122fb000000200054006f006f006c002000500072006f0070006500720074006900650073003203000002880000011d000002210000017afb000000100044006900730070006c006100790073010000003d0000020b000000c900fffffffb0000002000730065006c0065006300740069006f006e00200062007500660066006500720200000138000000aa0000023a00000294fb00000014005700690064006500530074006500720065006f02000000e6000000d2000003ee0000030bfb0000000c004b0069006e0065006300740200000186000001060000030c00000261fb00000018004e0061007600690067006100740069006f006e0020003201000002600000006e0000014200ffffff00000001000001110000034afc0200000002fb0000001e0054006f006f006c002000500072006f00700065007200740069006500730100000041000000780000000000000000fb0000001200530065006c0065006300740069006f006e010000025a000000b200000000000000000000000200000490000000a9fc0100000001fb0000000a00560069006500770073030000004e00000080000002e10000019700000003000004420000003efc0100000002fb0000000800540069006d00650100000000000004420000000000000000fb0000000800540069006d00650100000000000004500000000000000000000005ce000002f400000004000000040000000800000008fc0000000100000002000000010000000a0054006f006f006c00730100000000ffffffff0000000000000000
      Selection:
        collapsed: false
      Tool Properties:
        collapsed: false
      Views:
        collapsed: false
      Width: 1500
      X: 60
      Y: 60
    """
    
    # Create the RViz config file
    with open(rviz_config_path, 'w') as f:
        f.write(rviz_config)
    
    # Launch arguments - basic
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    
    # Launch arguments - encoder specific
    encoder_odometry = LaunchConfiguration('encoder_odometry')
    serial_port = LaunchConfiguration('serial_port')
    wheel_radius = LaunchConfiguration('wheel_radius')
    base_width = LaunchConfiguration('base_width')
    ticks_per_rev = LaunchConfiguration('ticks_per_rev')
    max_linear_speed = LaunchConfiguration('max_linear_speed')
    max_angular_speed = LaunchConfiguration('max_angular_speed')
    publish_rate = LaunchConfiguration('publish_rate')
    map_yaml_file = LaunchConfiguration('map')

    # Declare basic launch arguments
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true')

    declare_use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz')
        
    declare_map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_dir, 'maps', 'simple_map.yaml'),
        description='Full path to map YAML file')
        
    # Declare encoder-specific launch arguments
    declare_encoder_odometry_arg = DeclareLaunchArgument(
        'encoder_odometry',
        default_value='false',
        description='Use E38S high-resolution encoders for odometry')
        
    declare_serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for Arduino connection')
        
    declare_wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.0825',
        description='Wheel radius in meters')
        
    declare_base_width_arg = DeclareLaunchArgument(
        'base_width',
        default_value='0.17',
        description='Distance between wheels in meters')
        
    declare_ticks_per_rev_arg = DeclareLaunchArgument(
        'ticks_per_rev',
        default_value='4000.0',
        description='Encoder ticks per revolution (4000 for E38S with quadrature)')
        
    declare_max_linear_speed_arg = DeclareLaunchArgument(
        'max_linear_speed',
        default_value='0.5',
        description='Maximum linear speed in m/s')
        
    declare_max_angular_speed_arg = DeclareLaunchArgument(
        'max_angular_speed',
        default_value='2.0',
        description='Maximum angular speed in rad/s')
        
    declare_publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='20.0',
        description='Odometry publish rate in Hz')

    # Include the main navigation launch file
    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_dir, 'launch', 'navigation.launch.py')
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': map_yaml_file,
            'encoder_odometry': encoder_odometry,
            'serial_port': serial_port,
            'wheel_radius': wheel_radius,
            'base_width': base_width,
            'ticks_per_rev': ticks_per_rev,
            'max_linear_speed': max_linear_speed,
            'max_angular_speed': max_angular_speed,
            'publish_rate': publish_rate
        }.items()
    )
    
    # Launch RViz with our config
    from launch.conditions import IfCondition
    
    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # Return the launch description
    return LaunchDescription([
        declare_use_sim_time_arg,
        declare_use_rviz_arg,
        nav_launch,
        rviz_node
    ])
