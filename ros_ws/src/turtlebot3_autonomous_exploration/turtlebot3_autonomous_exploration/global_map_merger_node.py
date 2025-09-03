# VERSIONE MODIFICATA CHE UNISCE MAPPE E ANCHE LE COSTMAPE DI PIU' ROBOT IN UNA SOLA MAPPA GLOBALE

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener
import numpy as np
import threading
from rclpy.duration import Duration

class GlobalMapMerger(Node):
    def __init__(self):
        super().__init__('global_map_merger')
        
        self.occupancy_maps = {}
        self.costmaps = {}
        self.map_lock = threading.Lock()
        
        self.declare_parameter('robot_namespaces', ['robot1', 'robot2'])
        self.robot_namespaces = self.get_parameter('robot_namespaces').get_parameter_value().string_array_value

        self.tf_buffer = Buffer(Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        for ns in self.robot_namespaces:
            self.create_subscription(
                OccupancyGrid,
                f'/{ns}/{ns}/map',
                lambda msg, namespace=ns: self.occupancy_map_callback(msg, namespace),
                10
            )
            self.create_subscription(
                OccupancyGrid,
                f'/{ns}/{ns}/global_costmap/costmap',
                lambda msg, namespace=ns: self.costmap_callback(msg, namespace),
                10
            )

        self.global_occupancy_map_pub = self.create_publisher(OccupancyGrid, '/global_map', 10)
        self.global_costmap_pub = self.create_publisher(OccupancyGrid, '/global_costmap', 10)

        self.timer = self.create_timer(1.0, self.merge_and_publish_maps)
        
        self.get_logger().info('Global Map Merger node initialized.')

    def occupancy_map_callback(self, msg, namespace):
        with self.map_lock:
            self.occupancy_maps[namespace] = msg
            self.get_logger().debug(f"Mappa di occupazione di {namespace} ricevuta.")

    def costmap_callback(self, msg, namespace):
        with self.map_lock:
            self.costmaps[namespace] = msg
            self.get_logger().debug(f"Costmap di {namespace} ricevuta.")
            
    def merge_and_publish_maps(self):
        with self.map_lock:
            if not self.occupancy_maps or not self.costmaps:
                self.get_logger().warn('Mappe di occupazione o costmap mancanti. Salto il merge.')
                return
            
            # Utilizza la prima mappa di occupazione disponibile per determinare risoluzione e dimensioni iniziali
            first_occ_map_ns = next(iter(self.occupancy_maps))
            first_occ_map_msg = self.occupancy_maps[first_occ_map_ns]

            global_map_resolution = first_occ_map_msg.info.resolution
            
            min_x, max_x, min_y, max_y = float('inf'), float('-inf'), float('inf'), float('-inf')
            
            # Calcola le dimensioni della bounding box per contenere tutte le mappe e costmap
            all_maps = list(self.occupancy_maps.items()) + list(self.costmaps.items())
            
            successful_merges = False

            for namespace, map_msg in all_maps:
                try:
                    transform: TransformStamped = self.tf_buffer.lookup_transform(
                        'map',
                        map_msg.header.frame_id,
                        rclpy.time.Time(),
                        timeout=Duration(seconds=1.0)
                    )
                except Exception as e:
                    self.get_logger().debug(f'Impossibile ottenere la trasformazione per {namespace}. Errore: {e}')
                    continue
                
                map_origin_x = transform.transform.translation.x
                map_origin_y = transform.transform.translation.y
                map_width = map_msg.info.width * map_msg.info.resolution
                map_height = map_msg.info.height * map_msg.info.resolution
                
                min_x = min(min_x, map_origin_x)
                max_x = max(max_x, map_origin_x + map_width)
                min_y = min(min_y, map_origin_y)
                max_y = max(max_y, map_origin_y + map_height)
                successful_merges = True
            
            if not successful_merges:
                self.get_logger().warn('Nessuna trasformata valida trovata. Salto il merge.')
                return

            margin = 5.0
            min_x -= margin
            max_x += margin
            min_y -= margin
            max_y += margin

            global_map_width_m = max_x - min_x
            global_map_height_m = max_y - min_y
            
            if global_map_width_m <= 0 or global_map_height_m <= 0:
                self.get_logger().error("Dimensioni della mappa globale non valide dopo la trasformata.")
                return

            global_map_width = int(global_map_width_m / global_map_resolution)
            global_map_height = int(global_map_height_m / global_map_resolution)

            global_occupancy_map_data = np.full((global_map_height, global_map_width), -1, dtype=np.int8)
            global_costmap_data = np.full((global_map_height, global_map_width), 0, dtype=np.int8)
            
            # Unisce le mappe di occupazione
            for namespace, map_msg in self.occupancy_maps.items():
                try:
                    transform: TransformStamped = self.tf_buffer.lookup_transform(
                        'map',
                        map_msg.header.frame_id,
                        rclpy.time.Time()
                    )
                except Exception:
                    continue
                
                map_data_2d = np.array(map_msg.data, dtype=np.int8).reshape(map_msg.info.height, map_msg.info.width)
                
                for y in range(map_msg.info.height):
                    for x in range(map_msg.info.width):
                        cell_value = map_data_2d[y, x]
                        if cell_value == -1:
                            continue
                        
                        wx = map_msg.info.origin.position.x + x * map_msg.info.resolution
                        wy = map_msg.info.origin.position.y + y * map_msg.info.resolution
                        
                        tx = transform.transform.translation.x + wx
                        ty = transform.transform.translation.y + wy
                        
                        gx_cell = int((tx - min_x) / global_map_resolution)
                        gy_cell = int((ty - min_y) / global_map_resolution)
                        
                        if 0 <= gx_cell < global_map_width and 0 <= gy_cell < global_map_height:
                            if cell_value == 100:
                                global_occupancy_map_data[gy_cell, gx_cell] = 100
                            elif cell_value == 0 and global_occupancy_map_data[gy_cell, gx_cell] != 100:
                                global_occupancy_map_data[gy_cell, gx_cell] = 0

            # Unisce le costmap
            for namespace, map_msg in self.costmaps.items():
                try:
                    transform: TransformStamped = self.tf_buffer.lookup_transform(
                        'map',
                        map_msg.header.frame_id,
                        rclpy.time.Time()
                    )
                except Exception:
                    continue
                
                map_data_2d = np.array(map_msg.data, dtype=np.int8).reshape(map_msg.info.height, map_msg.info.width)
                
                for y in range(map_msg.info.height):
                    for x in range(map_msg.info.width):
                        cell_value = map_data_2d[y, x]
                        if cell_value == -1:
                            continue
                        
                        wx = map_msg.info.origin.position.x + x * map_msg.info.resolution
                        wy = map_msg.info.origin.position.y + y * map_msg.info.resolution
                        
                        tx = transform.transform.translation.x + wx
                        ty = transform.transform.translation.y + wy
                        
                        gx_cell = int((tx - min_x) / global_map_resolution)
                        gy_cell = int((ty - min_y) / global_map_resolution)
                        
                        if 0 <= gx_cell < global_map_width and 0 <= gy_cell < global_map_height:
                            # Logica di fusione per la costmap: il valore più alto ha la priorità
                            if cell_value > global_costmap_data[gy_cell, gx_cell]:
                                global_costmap_data[gy_cell, gx_cell] = cell_value

            # Pubblica la mappa unificata di occupazione
            global_occupancy_map_msg = OccupancyGrid()
            global_occupancy_map_msg.header.frame_id = 'map'
            global_occupancy_map_msg.header.stamp = self.get_clock().now().to_msg()
            global_occupancy_map_msg.info.width = global_map_width
            global_occupancy_map_msg.info.height = global_map_height
            global_occupancy_map_msg.info.resolution = global_map_resolution
            global_occupancy_map_msg.info.origin.position.x = min_x
            global_occupancy_map_msg.info.origin.position.y = min_y
            global_occupancy_map_msg.info.origin.position.z = 0.0
            global_occupancy_map_msg.data = global_occupancy_map_data.flatten().tolist()
            self.global_occupancy_map_pub.publish(global_occupancy_map_msg)
            
            # Pubblica la costmap unificata
            global_costmap_msg = OccupancyGrid()
            global_costmap_msg.header.frame_id = 'map'
            global_costmap_msg.header.stamp = self.get_clock().now().to_msg()
            global_costmap_msg.info.width = global_map_width
            global_costmap_msg.info.height = global_map_height
            global_costmap_msg.info.resolution = global_map_resolution
            global_costmap_msg.info.origin.position.x = min_x
            global_costmap_msg.info.origin.position.y = min_y
            global_costmap_msg.info.origin.position.z = 0.0
            global_costmap_msg.data = global_costmap_data.flatten().tolist()
            self.global_costmap_pub.publish(global_costmap_msg)
            
            self.get_logger().info('Mappe globali di occupazione e costo unificate e pubblicate.')

def main(args=None):
    rclpy.init(args=args)
    node = GlobalMapMerger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()