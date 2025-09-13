import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster
import math
import yaml
import os

# QUESTO NODO, PARTENDO DALLE POSIZIONI INIZIALI DEI ROBOT, PASSATE DAL FILE DI CONFIGURAZIONE, VA SEMPLICEMENTE A 
# PUBBLICARE UNA TRASFORMAZIONE CHE PERMETTE DI COLLEGATE I FRAME NAMESPACE/NAMESPACE/MAP AL FRAME GLOBALE MAP IN MODO TALE CHE
# , SU RVIZ, SI POSSA VEDERE INSIEME TUTTO QUANTO.


class GlobalFrameBroadcaster(Node):
    def __init__(self):
        super().__init__('global_frame_broadcaster')
        
        # Broadcaster per le trasformate statiche
        self.tf_broadcaster = StaticTransformBroadcaster(self)
        
        self.declare_parameter('robot_configs_path', 'default_path.yaml')
        config_path = self.get_parameter('robot_configs_path').get_parameter_value().string_value
        
        self.robot_configs = self.load_configs(config_path)

        # Timer per pubblicare le trasformate
        # Un broadcaster statico le invia solo una volta, ma le ripubblichiamo per sicurezza
        self.timer = self.create_timer(1.0, self.publish_transforms)
        
        self.get_logger().info('Global Frame Broadcaster inizializzato.')

    def load_configs(self, path):
        """Carica le configurazioni dei robot da un file YAML."""
        if not os.path.exists(path):
            self.get_logger().error(f'File di configurazione non trovato: {path}')
            return {}
        
        with open(path, 'r') as f:
            try:
                configs = yaml.safe_load(f)
                self.get_logger().info('Configurazioni dei robot caricate con successo.')
                return configs
            except yaml.YAMLError as e:
                self.get_logger().error(f'Errore nel parsing del file YAML: {e}')
                return {}

    def publish_transforms(self):
        """
        Pubblica le trasformate per ogni robot in base alle posizioni iniziali.
        """
        if not self.robot_configs:
            self.get_logger().warn('Nessuna configurazione robot trovata. Salto la pubblicazione delle trasformate.')
            return

        for robot_name, config in self.robot_configs.items():
            if 'initial_position' not in config:
                self.get_logger().warn(f'Posizione iniziale non specificata per il robot {robot_name}.')
                continue
            
            x = config['initial_position']['x']
            y = config['initial_position']['y']
            
            # Crea un messaggio di trasformata
            transform_stamped = TransformStamped()
            transform_stamped.header.stamp = self.get_clock().now().to_msg()
            transform_stamped.header.frame_id = 'map'
            transform_stamped.child_frame_id = f'{robot_name}/{robot_name}/map'
            
            # Imposta la traslazione
            transform_stamped.transform.translation.x = float(x)
            transform_stamped.transform.translation.y = float(y)
            transform_stamped.transform.translation.z = 0.0
            
            # Imposta l'orientamento a 0.0 (nessuna rotazione)
            transform_stamped.transform.rotation.x = 0.0
            transform_stamped.transform.rotation.y = 0.0
            transform_stamped.transform.rotation.z = 0.0
            transform_stamped.transform.rotation.w = 1.0
            
            self.tf_broadcaster.sendTransform(transform_stamped)
            self.get_logger().info(f'Trasformata pubblicata per {robot_name}.')

def main(args=None):
    rclpy.init(args=args)
    node = GlobalFrameBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
