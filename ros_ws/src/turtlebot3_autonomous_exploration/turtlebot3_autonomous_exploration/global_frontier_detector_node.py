import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped
import numpy as np

# LA LOGICA DI FUNZIONAMENTO DI QUESTO NODO E' RIMASTA IN SOSTANZA INVARIATA DAL CASO DEL SINGOLO ROBOT.


class GlobalFrontierDetector(Node):
    """
    Nodo ROS2 per la rilevazione di frontiere globali in una mappa di occupazione.
    Le frontiere sono punti dove le celle libere incontrano celle sconosciute.
    """
    def __init__(self):
        super().__init__('global_frontier_detector')
        
        
        self.global_map_sub = self.create_subscription(
            OccupancyGrid, '/global_map', self.map_callback, 10
        )

        self.global_costmap_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap', self.costmap_callback, 10
        )

        self.pub = self.create_publisher(Path, '/global_frontier_centroids', 10)

        # Inizializza la costmap come None fino a quando non viene ricevuta
        self.costmap = None
        self.get_logger().info('GlobalFrontierDetector node initialized.')

    def costmap_callback(self, msg):
        """Callback per aggiornare la costmap globale ricevuta"""
        self.costmap = msg

    def map_callback(self, msg):
        """
        Callback principale che processa la mappa di occupazione per trovare le frontiere.
        Identifica i punti di frontiera, li raggruppa in cluster e pubblica i centroidi.
        """
        if self.costmap is None:
            self.get_logger().warn('Costmap globale non ancora disponibile.')
            return

        # Estraazione dei metadati della mappa (dimensioni, risoluzione, origine)
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin = msg.info.origin.position

        # Converte i dati della mappa in un array NumPy 2D 
        data = np.array(msg.data, dtype=np.int8).reshape((height, width))

        # Identificazione delle celle 
        free = (data == 0)
        unknown = (data == -1)

        # Lista per memorizzare tutti i punti di frontiera trovati
        frontier_points = []
        
        # Cicla sull'intera mappa (eccetto i bordi) per trovare le frontiere
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                # Considera solo le celle libere come potenziali frontiere
                if free[y, x]:
                    # Controlla se una cella libera ha una cella sconosciuta nella sua vicinanza 3x3
                    neighborhood_unknown = unknown[y-1:y+2, x-1:x+2]
                    # Se c'è almeno una cella sconosciuta nelle vicinanze, è una frontiera
                    if np.any(neighborhood_unknown):
                        # Converti le coordinate della mappa in coordinate del mondo reale
                        wx = origin.x + (x + 0.5) * resolution
                        wy = origin.y + (y + 0.5) * resolution
                        frontier_points.append([wx, wy])

        # Se non sono stati trovati punti di frontiera, pubblica un messaggio vuoto
        if not frontier_points:
            self.get_logger().info('Nessun punto di frontiera trovato.')
            self.pub.publish(Path())
            return

        # Converti la lista in array NumPy per facilitare l'elaborazione
        pts = np.array(frontier_points)

        # Esegui il clustering basato sulla distanza per trovare i centroidi delle aree di frontiera
        centroids = self.cluster_frontiers(pts, radius=0.3)

        # Prepara i dati della costmap per il filtraggio dei centroidi
        costmap_data = np.array(self.costmap.data, dtype=np.int8).reshape(
            (self.costmap.info.height, self.costmap.info.width)
        )
        costmap_resolution = self.costmap.info.resolution
        costmap_origin = self.costmap.info.origin.position

        # Lista per i centroidi che superano il filtro di sicurezza della costmap
        safe_centroids = []
        for cx, cy in centroids:
            # Conversione delle coordinate del mondo in indici di cella della costmap
            mx = int((cx - costmap_origin.x) / costmap_resolution)
            my = int((cy - costmap_origin.y) / costmap_resolution)

            # Controlla se il centroide è all'interno dei limiti della costmap
            if 0 <= mx < self.costmap.info.width and 0 <= my < self.costmap.info.height:
                cost = costmap_data[my, mx]
                # Mantieni solo i centroidi in aree a basso costo (sicure per la navigazione)
                # Soglia di 75: valori più alti indicano ostacoli o aree pericolose
                if 0 <= cost < 75:
                    safe_centroids.append([cx, cy])

        # Costruisci il messaggio Path con i centroidi sicuri
        path = Path()
        path.header.frame_id = 'map'  # Frame di riferimento della mappa
        path.header.stamp = self.get_clock().now().to_msg()  # Timestamp corrente

        # Aggiungi ogni centroide sicuro come PoseStamped nel Path
        for cx, cy in safe_centroids:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = cx
            pose.pose.position.y = cy
            pose.pose.position.z = 0.0  # Assumiamo navigazione 2D
            pose.pose.orientation.w = 1.0  # Orientamento neutro (quaternione identità)
            path.poses.append(pose)

        # Pubblica il messaggio con i centroidi delle frontiere
        self.pub.publish(path)
        self.get_logger().info(f'Published {len(path.poses)} global frontier centroids (filtered with costmap).')

    def cluster_frontiers(self, points, radius=0.1):
        """
        Raggruppa i punti di frontiera in cluster basandosi sulla distanza euclidea.
        Restituisce i centroidi di ogni cluster per ridurre il numero di frontiere candidate.
        
        Args:
            points: Array NumPy dei punti di frontiera [x, y]
            radius: Raggio massimo per considerare due punti nello stesso cluster
            
        Returns:
            Lista dei centroidi dei cluster
        """
        clusters = []  # Lista per memorizzare tutti i cluster
        used = np.zeros(len(points), dtype=bool)  # Traccia quali punti sono già stati assegnati

        # Per ogni punto non ancora assegnato, crea un nuovo cluster
        for i, pt in enumerate(points):
            if used[i]:
                continue  # Salta i punti già assegnati a un cluster
                
            # Inizia un nuovo cluster con il punto corrente
            cluster = [pt]
            used[i] = True
            
            # Cerca tutti gli altri punti vicini per aggiungerli al cluster
            for j in range(i + 1, len(points)):
                if not used[j] and np.linalg.norm(pt - points[j]) < radius:
                    cluster.append(points[j])
                    used[j] = True
                    
            clusters.append(np.array(cluster))

        # Calcola il centroide (punto medio) di ogni cluster
        centroids = [c.mean(axis=0) for c in clusters if len(c) > 0]
        return centroids

def main(args=None):
    """Funzione principale per inizializzare e far girare il nodo ROS2"""
    rclpy.init(args=args)  
    node = GlobalFrontierDetector()  
    rclpy.spin(node)  
    node.destroy_node()  
    rclpy.shutdown()  

if __name__ == '__main__':
    main()
