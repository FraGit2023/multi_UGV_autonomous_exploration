import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import numpy as np

class GlobalFrontierDetector(Node):
    def __init__(self):
        super().__init__('global_frontier_detector')
        
        # Sottoscrizione alla mappa di griglia di occupazione globale
        self.global_map_sub = self.create_subscription(
            OccupancyGrid, '/global_map', self.map_callback, 10
        )

        # Sottoscrizione alla costmap globale per filtrare le frontiere
        self.global_costmap_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap', self.costmap_callback, 10
        )

        # Publisher per i centroidi delle frontiere come Path message
        self.pub = self.create_publisher(Path, '/global_frontier_centroids', 10)

        self.costmap = None
        self.get_logger().info('GlobalFrontierDetector node initialized.')

    def costmap_callback(self, msg):
        self.costmap = msg

    def map_callback(self, msg):
        if self.costmap is None:
            self.get_logger().warn('Costmap globale non ancora disponibile.')
            return

        # Estrai i metadati della mappa
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin.position

        # Converti i dati della mappa in un array NumPy 2D
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))

        # Identifica le celle libere e sconosciute
        free = (data == 0)
        unknown = (data == -1)

        # Rileva i punti di frontiera
        frontier_points = []
        # Cicla sull'intera mappa (eccetto i bordi) per trovare le frontiere
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if free[y, x]:
                    # Controlla se una cella libera ha una cella sconosciuta nella sua vicinanza 3x3
                    neighborhood_unknown = unknown[y-1:y+2, x-1:x+2]
                    if np.any(neighborhood_unknown):
                        # Converti le coordinate della mappa in coordinate del mondo
                        wx = origin.x + (x + 0.5) * resolution
                        wy = origin.y + (y + 0.5) * resolution
                        frontier_points.append([wx, wy])

        if not frontier_points:
            self.get_logger().info('Nessun punto di frontiera trovato.')
            # Pubblica un messaggio vuoto per indicare che non ci sono frontiere
            self.pub.publish(Path())
            return

        pts = np.array(frontier_points)

        # Esegui il clustering basato sulla distanza per trovare i centroidi
        centroids = self.cluster_frontiers(pts, radius=0.3)

        # Filtra i centroidi usando la costmap globale
        costmap_data = np.array(self.costmap.data, dtype=np.int8).reshape(
            (self.costmap.info.height, self.costmap.info.width)
        )
        costmap_resolution = self.costmap.info.resolution
        costmap_origin = self.costmap.info.origin.position

        safe_centroids = []
        for cx, cy in centroids:
            # Converti le coordinate del mondo in indici di cella della costmap
            mx = int((cx - costmap_origin.x) / costmap_resolution)
            my = int((cy - costmap_origin.y) / costmap_resolution)

            # Controlla se il centroide è all'interno dei limiti della costmap
            if 0 <= mx < self.costmap.info.width and 0 <= my < self.costmap.info.height:
                cost = costmap_data[my, mx]
                # Mantieni solo i centroidi in aree a basso costo (sicure)
                if 0 <= cost < 75:
                    safe_centroids.append([cx, cy])

        # Costruisci e pubblica il messaggio Path con i centroidi sicuri
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()

        for cx, cy in safe_centroids:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = cx
            pose.pose.position.y = cy
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)

        self.pub.publish(path)
        self.get_logger().info(f'Published {len(path.poses)} global frontier centroids (filtered with costmap).')

    def cluster_frontiers(self, points, radius=0.1):
        clusters = []
        used = np.zeros(len(points), dtype=bool)

        for i, pt in enumerate(points):
            if used[i]:
                continue
            cluster = [pt]
            used[i] = True
            for j in range(i + 1, len(points)):
                if not used[j] and np.linalg.norm(pt - points[j]) < radius:
                    cluster.append(points[j])
                    used[j] = True
            clusters.append(np.array(cluster))

        centroids = [c.mean(axis=0) for c in clusters if len(c) > 0]
        return centroids

def main(args=None):
    rclpy.init(args=args)
    node = GlobalFrontierDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()