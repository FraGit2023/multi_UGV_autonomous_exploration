import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    """
    Questa funzione viene eseguita al momento del lancio per gestire
    la logica dinamica di creazione del file di configurazione del bridge.
    """
    # Ottiene i valori degli argomenti di lancio
    namespace = LaunchConfiguration('namespace').perform(context)
    
    # Definisce i percorsi
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    
    # Percorso al file YAML di configurazione del ponte originale
    bridge_yaml_path = os.path.join(
        pkg_turtlebot3_gazebo,
        'params',
        'turtlebot3_' + TURTLEBOT3_MODEL + '_bridge_robot1.yaml'
    )

    # Carica il contenuto del file YAML
    with open(bridge_yaml_path, 'r') as file:
        bridge_data = yaml.safe_load(file)

    for topic_bridge in bridge_data:
        # I topic Gazebo mantengono il singolo namespace
        topic_bridge['gz_topic_name'] = topic_bridge['gz_topic_name'].replace(TURTLEBOT3_MODEL, namespace)
        
        # I topic ROS mantengono il singolo namespace (Gazebo pubblica qui)
        # Il doppio namespace sarà gestito tramite remapping nei nodi
        topic_bridge['ros_topic_name'] = topic_bridge['ros_topic_name'].replace(TURTLEBOT3_MODEL, namespace)

    # Definisce un percorso per il file YAML temporaneo e specifico del robot
    tmp_yaml_path = os.path.join('/tmp', f'{namespace}_bridge_double.yaml')

    # Scrive la nuova configurazione nel file temporaneo
    with open(tmp_yaml_path, 'w') as file:
        yaml.dump(bridge_data, file, default_flow_style=False)

    # Crea il nodo del ponte di parametri usando il file appena creato
    start_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={tmp_yaml_path}'],
        output='screen'
    )
    
    return [start_bridge_cmd]

def generate_launch_description():
    """
    Funzione principale che genera la descrizione del lancio.
    """
    # 1. Dichiarazione degli argomenti
    declare_namespace_arg = DeclareLaunchArgument('namespace', default_value='', description='Namespace del robot')
    declare_x_pose_arg = DeclareLaunchArgument('x_pose', default_value='0.0', description='Posizione X iniziale')
    declare_y_pose_arg = DeclareLaunchArgument('y_pose', default_value='0.0', description='Posizione Y iniziale')

    # 2. Ottiene i valori e i percorsi necessari
    namespace = LaunchConfiguration('namespace')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')

    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    TURTLEBOT3_MODEL = os.environ.get('TURTLEBOT3_MODEL', 'burger')
    sdf_path = os.path.join(pkg_turtlebot3_gazebo, 'models', 'turtlebot3_' + TURTLEBOT3_MODEL, 'model.sdf')

    # 3. Nodo per generare (spawn) il modello del robot in Gazebo
    spawn_robot_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', namespace,
            '-file', sdf_path,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01'
        ],
        output='screen'
    )

    # 4. Creazione della LaunchDescription
    ld = LaunchDescription()
    
    # Aggiunge gli argomenti dichiarati
    ld.add_action(declare_namespace_arg)
    ld.add_action(declare_x_pose_arg)
    ld.add_action(declare_y_pose_arg)

    # Aggiunge i nodi da lanciare
    ld.add_action(spawn_robot_cmd)
    
    # Aggiunge l'OpaqueFunction per il ponte
    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld