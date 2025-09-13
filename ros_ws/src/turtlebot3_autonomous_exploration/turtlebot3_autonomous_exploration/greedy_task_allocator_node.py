# MIGLIORE ATTUALE VERSIONE CON FIX RECOVERY PROTECTION
# Strategia di fallback graduale: Implementa fallback graduali con moltiplicatori [1.2, 1.4, 1.8, 2]
# Gestione robusta degli errori: Include il metodo handle_recovery_goal_failure()
# che gestisce i casi in cui il goal di recovery viene rifiutato o fallisce
# Fallback multipli: Se un goal di recovery fallisce, prova automaticamente con la frontiera più vicina al robot
# FIX RECOVERY PROTECTION: Protezione rigida solo per recovery mode, comportamento normale invariato

import rclpy
import tf2_geometry_msgs
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Pose, Quaternion
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry, Path
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
import numpy as np
import math

class GreedyTaskAllocator(Node):
    """
    Nodo ROS2 che implementa un allocatore di task per più robot autonomi.
    
    La strategia di allocazione è "greedy": assegna i compiti (esplorazione di frontiere)
    in base alla distanza, dando priorità alle frontiere più lontane tra loro per
    massimizzare la copertura dell'area.
    
    """

    def __init__(self):
        
        super().__init__('greedy_task_allocator')
        
        # Elenco dei nomi dei robot gestiti dal nodo
        self.robots = ['robot1', 'robot2']
        # Dizionario per memorizzare l'ultima posizione conosciuta di ogni robot
        self.robot_poses = {}
        # Elenco dei centroidi delle frontiere (punti di interesse per l'esplorazione)
        self.frontier_centroids = []
        
        # Inizializza il listener TF per trasformare le pose tra i frame di coordinate
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        
        self.frontier_sub = self.create_subscription(
            Path, '/global_frontier_centroids', self.frontier_callback, 10)

        
        for robot_name in self.robots:
            self.create_subscription(
                Odometry,
                f'/{robot_name}/{robot_name}/odom/filtered',
                lambda msg, ns=robot_name: self.odom_callback(msg, ns),
                10
            )

        # Client di azione per inviare goal di navigazione (NavigateToPose) a ciascun robot
        self.nav_clients = {
            robot_name: ActionClient(self, NavigateToPose, f'/{robot_name}/{robot_name}/navigate_to_pose')
            for robot_name in self.robots
        }
        
        # Dizionario per tenere traccia del goal assegnato a ciascun robot
        self.assigned_goals = {robot_name: None for robot_name in self.robots}
        # Dizionario per memorizzare l'handle del goal di azione attivo (per cancellazioni/stati)
        self.goal_handles = {robot_name: None for robot_name in self.robots}
        
        # ATTRIBUTI PER LA STRATEGIA DI RECUPERO
        self.goal_history = {robot_name: [] for robot_name in self.robots}  # Storico degli ultimi goal assegnati
        self.goal_completion_status = {robot_name: False for robot_name in self.robots}  # Stato di completamento del goal
        self.recovery_mode = {robot_name: False for robot_name in self.robots}  # Flag per indicare se un robot è in modalità recovery
        
        # POSIZIONI INIZIALI DEI ROBOT (salvate al primo odom ricevuto)
        self.initial_positions = {robot_name: None for robot_name in self.robots}

        # Flag per lo stato generale del sistema
        self.exploration_completed = False
        self.initialization_complete = False
        self.create_timer(5.0, self.complete_initialization)  # Timer di inizializzazione per garantire la disponibilità dei dati iniziali
        
        # Parametri per la strategia di recupero
        self.max_goal_history = 3  # Numero massimo di goal da tenere nello storico
        self.recovery_distance_threshold = 0.5  # Distanza minima per considerare un goal raggiunto
        self.recovery_distance_multiplier = 2  # Moltiplicatore base per la distanza di recovery
        
        # Timer periodico per controllare lo stato dei robot e delle frontiere e avviare l'allocazione
        self.stuck_check_timer = self.create_timer(10.0, self.check_and_allocate)

        self.get_logger().info('Greedy Task Allocator node initialized with recovery strategy.')

    def odom_callback(self, msg, robot_name):
        """
        Callback per la ricezione dei messaggi di Odometry.
        Aggiorna la posizione del robot e salva la posizione iniziale se è il primo messaggio.
        """
        self.robot_poses[robot_name] = msg.pose.pose
        
        # Salva la posizione iniziale al primo messaggio ricevuto
        if self.initial_positions[robot_name] is None:
            self.initial_positions[robot_name] = (
                msg.pose.pose.position.x, 
                msg.pose.pose.position.y
            )
            self.get_logger().info(f"Posizione iniziale salvata per {robot_name}: {self.initial_positions[robot_name]}")

    def complete_initialization(self):
        """
        Callback del timer di inizializzazione. Imposta il flag 'initialization_complete'
        per consentire l'avvio delle operazioni di allocazione.
        """
        self.initialization_complete = True
        self.get_logger().info("Inizializzazione completata, sistema pronto per l'allocazione.")

    def frontier_callback(self, msg):
        """
        Callback per la ricezione dei centroidi delle frontiere.
        Aggiorna l'elenco delle frontiere e avvia un controllo per l'allocazione.
        Gestisce anche lo stato di completamento dell'esplorazione.
        """
        self.frontier_centroids = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        
        # Se non ci sono frontiere e l'esplorazione non è ancora contrassegnata come completata,
        # significa che l'esplorazione è terminata.
        if not self.frontier_centroids and not self.exploration_completed:
            self.get_logger().info("Nessuna frontiera rilevata - Esplorazione completata! Invio robot alle posizioni iniziali.")
            self.exploration_completed = True
            self.send_robots_home()
            return
        
        # Se ci sono nuove frontiere dopo che l'esplorazione era stata completata, la si riattiva.
        if self.frontier_centroids and self.exploration_completed:
            self.get_logger().info("Nuove frontiere rilevate - Riattivazione esplorazione.")
            self.exploration_completed = False
        
        self.check_and_allocate()

    def check_and_allocate(self):
        """
        Controlla lo stato dei robot e delle frontiere e avvia l'allocazione se necessario.
        """
        # Non procedere se l'inizializzazione non è completa
        if not self.initialization_complete:
            return
        
        # Non procedere se l'esplorazione è completata
        if self.exploration_completed:
            return
            
        # Verifica che i dati necessari (posizioni dei robot e frontiere) siano disponibili
        if not all(robot in self.robot_poses for robot in self.robots) or not self.frontier_centroids:
            self.get_logger().warn("Dati mancanti per l'allocazione: posizioni robot o frontiere.")
            return

        # Trova i robot pronti per una nuova assegnazione
        ready_robots = []
        for robot_name in self.robots:
            # PROTEZIONE RECOVERY: Un robot in modalità recovery non viene considerato pronto per
            # nuove assegnazioni normali. Questo evita di interrompere la sua azione di recupero.
            if self.recovery_mode[robot_name]:
                self.get_logger().info(f"Robot {robot_name} è in modalità recovery - PROTETTO fino al completamento.")
                continue
                
            # Controllo normale: un robot è pronto se non ha un goal assegnato
            if self.assigned_goals[robot_name] is None:
                ready_robots.append(robot_name)
        
        # Se non ci sono robot pronti, esci
        if not ready_robots:
            robots_in_recovery = [r for r in self.robots if self.recovery_mode[r]]
            if robots_in_recovery:
                self.get_logger().info(f"Robot in modalità recovery: {robots_in_recovery} - attendendo completamento goal di recovery.")
            else:
                self.get_logger().info("Tutti i robot sono già in movimento o hanno un goal assegnato.")
            return

        self.allocate_tasks(ready_robots)

    def is_goal_reached(self, robot_name, goal_coords):
        """
        Verifica se il robot ha raggiunto il goal specificato confrontando la sua posizione
        attuale con le coordinate del goal. La posizione del robot viene trasformata nel
        frame globale 'map' per un confronto accurato.
        """
        if robot_name not in self.robot_poses:
            return False
        
        robot_pose_stamped = PoseStamped()
        robot_pose_stamped.header.frame_id = f'{robot_name}/{robot_name}/odom'
        robot_pose_stamped.header.stamp = self.get_clock().now().to_msg()
        robot_pose_stamped.pose.position = self.robot_poses[robot_name].position
        robot_pose_stamped.pose.orientation = self.robot_poses[robot_name].orientation

        try:
            latest_time = self.tf_buffer.get_latest_common_time('map', f'{robot_name}/{robot_name}/odom')
            robot_pose_stamped.header.stamp = latest_time.to_msg()
            
            robot_pose_in_map = self.tf_buffer.transform(
                robot_pose_stamped, 
                'map', 
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
            
            distance = math.hypot(
                goal_coords[0] - robot_pose_in_map.pose.position.x,
                goal_coords[1] - robot_pose_in_map.pose.position.y
            )
            
            # Ritorna True se la distanza è minore della soglia di recovery
            return distance < self.recovery_distance_threshold
            
        except Exception as e:
            self.get_logger().error(f"Errore nel calcolo della distanza per {robot_name}: {e}")
            return False

    def needs_recovery(self, robot_name):
        """
        Determina se un robot ha bisogno della strategia di recupero.
        La condizione di recovery è soddisfatta se lo stesso goal è stato assegnato
        due volte consecutive e il robot si trova già nelle vicinanze di quel goal.
        """
        history = self.goal_history[robot_name]
        
        # Serve almeno una storia di 2 goal per il confronto
        if len(history) < 2:
            return False
        
        # Controlla se gli ultimi 2 goal sono uguali
        last_goal = history[-1]
        second_last_goal = history[-2]
        
        if last_goal == second_last_goal:
            # Verifica se il goal è già stato raggiunto
            if self.is_goal_reached(robot_name, last_goal):
                self.get_logger().warn(f"Robot {robot_name} necessita strategia di recupero: goal ripetuto {last_goal} già raggiunto.")
                return True
        
        return False

    def find_recovery_goal(self, robot_name, problematic_goal):
        """
        Trova un goal di recupero per un robot.
        Usa una strategia di "fallback graduale": cerca una frontiera a una distanza
        moltiplicata dalla frontiera problematica. Se fallisce, usa la frontiera
        più vicina al robot.
        """
        # Crea una lista di frontiere disponibili, escludendo quella problematica
        available_frontiers = [f for f in self.frontier_centroids if f != problematic_goal]
        
        if not available_frontiers:
            self.get_logger().error(f"Nessuna frontiera disponibile per il recupero di {robot_name}.")
            return None
        
        # Calcola le distanze di tutte le frontiere dalla frontiera problematica
        frontier_distances = []
        for f in available_frontiers:
            dist = math.hypot(f[0] - problematic_goal[0], f[1] - problematic_goal[1])
            frontier_distances.append((dist, f))
        
        # Ordina le frontiere per distanza crescente dalla frontiera problematica
        frontier_distances.sort(key=lambda x: x[0])
        
        # Se la lista è vuota, esci
        if not frontier_distances:
            return None
            
        min_distance = frontier_distances[0][0]
        
        # STRATEGIA CON FALLBACK GRADUALI
        multipliers = [1.2, 1.4, 1.8, self.recovery_distance_multiplier]  # [1.2, 1.4, 1.8, 2]
        
        for multiplier in multipliers:
            target_min_distance = min_distance * multiplier
            self.get_logger().info(f"Recovery per {robot_name}: tentativo con moltiplicatore {multiplier} (distanza target: {target_min_distance:.2f})")
            
            # Cerca la prima frontiera che soddisfa il criterio di distanza
            for dist, frontier in frontier_distances:
                if dist >= target_min_distance:
                    self.get_logger().info(f"Goal di recupero per {robot_name}: {frontier} (distanza {dist:.2f} >= {target_min_distance:.2f} dal goal problematico {problematic_goal}).")
                    return frontier
            
            self.get_logger().warn(f"Nessuna frontiera trovata con moltiplicatore {multiplier}, provo con il successivo...")
        
        # ULTIMO FALLBACK: Se tutti i moltiplicatori falliscono, prendi la frontiera più vicina al robot
        self.get_logger().warn(f"Tutti i moltiplicatori falliti. Uso la frontiera più vicina al robot {robot_name}.")
        return self.find_closest_frontier_to_robot(robot_name, available_frontiers)
    
    def find_closest_frontier_to_robot(self, robot_name, available_frontiers):
        """
        Trova la frontiera più vicina alla posizione attuale del robot.
        """
        if robot_name not in self.robot_poses:
            self.get_logger().error(f"Posizione del robot {robot_name} non disponibile.")
            return available_frontiers[0] if available_frontiers else None
        
        robot_pose_stamped = PoseStamped()
        robot_pose_stamped.header.frame_id = f'{robot_name}/{robot_name}/odom'
        robot_pose_stamped.header.stamp = self.get_clock().now().to_msg()
        robot_pose_stamped.pose.position = self.robot_poses[robot_name].position
        robot_pose_stamped.pose.orientation = self.robot_poses[robot_name].orientation

        try:
            # Trasforma la posizione del robot nel frame globale
            latest_time = self.tf_buffer.get_latest_common_time('map', f'{robot_name}/{robot_name}/odom')
            robot_pose_stamped.header.stamp = latest_time.to_msg()
            
            robot_pose_in_map = self.tf_buffer.transform(
                robot_pose_stamped, 
                'map', 
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
            
            # Trova la frontiera più vicina tra quelle disponibili
            closest_frontier = min(
                available_frontiers,
                key=lambda f: math.hypot(
                    f[0] - robot_pose_in_map.pose.position.x,
                    f[1] - robot_pose_in_map.pose.position.y
                )
            )
            
            distance_to_robot = math.hypot(
                closest_frontier[0] - robot_pose_in_map.pose.position.x,
                closest_frontier[1] - robot_pose_in_map.pose.position.y
            )
            
            self.get_logger().info(f"Fallback: frontiera più vicina a {robot_name}: {closest_frontier} (distanza: {distance_to_robot:.2f})")
            return closest_frontier
            
        except Exception as e:
            self.get_logger().error(f"Errore nel trovare la frontiera più vicina al robot {robot_name}: {e}")
            # Fallback finale: prima frontiera disponibile
            return available_frontiers[0] if available_frontiers else None

    def update_goal_history(self, robot_name, goal_coords):
        """
        Aggiorna lo storico dei goal per un robot, mantenendo solo gli N goal più recenti.
        """
        self.goal_history[robot_name].append(goal_coords)
        
        # Mantieni solo gli ultimi N goal per la storia
        if len(self.goal_history[robot_name]) > self.max_goal_history:
            self.goal_history[robot_name].pop(0)
        
        self.get_logger().info(f"Storico goal aggiornato per {robot_name}: {self.goal_history[robot_name]}")

    def allocate_tasks(self, ready_robots):
        """
        Funzione principale di allocazione. Assegna le frontiere ai robot pronti.
        La strategia principale assegna le due frontiere più distanti ai due robot
        più vicini ad esse, per massimizzare la separazione.
        """
        # Gestisce il caso in cui ci sono meno frontiere che robot
        if len(self.frontier_centroids) < len(ready_robots):
            self.get_logger().warn('Meno frontiere disponibili che robot da assegnare. Assegno le frontiere rimanenti.')
            for i, robot_name in enumerate(ready_robots):
                if i < len(self.frontier_centroids):
                    goal = self.frontier_centroids[i]
                    self.send_navigation_goal(robot_name, goal)
                    self.assigned_goals[robot_name] = goal
                    self.update_goal_history(robot_name, goal)
            return

        # Controlla se qualche robot necessita di recovery e lo gestisce
        recovery_robots = []
        for robot_name in ready_robots:
            if self.needs_recovery(robot_name):
                recovery_robots.append(robot_name)
                
        # Processa prima i robot che necessitano di recovery, assegnando loro un goal di recupero
        for robot_name in recovery_robots:
            self.recovery_mode[robot_name] = True
            problematic_goal = self.goal_history[robot_name][-1]
            recovery_goal = self.find_recovery_goal(robot_name, problematic_goal)
            
            if recovery_goal:
                self.get_logger().info(f"Attivando strategia di recupero per {robot_name} - GOAL PROTETTO fino al completamento.")
                self.send_navigation_goal(robot_name, recovery_goal)
                self.assigned_goals[robot_name] = recovery_goal
                self.update_goal_history(robot_name, recovery_goal)
                ready_robots.remove(robot_name)  # Rimuovi il robot dalla lista dei pronti per l'allocazione normale

        # Se non ci sono robot rimasti, esci
        if not ready_robots:
            return

        # 1. Trova le due frontiere più distanti tra loro per la strategia "greedy"
        frontier_distances = []
        for i in range(len(self.frontier_centroids)):
            for j in range(i + 1, len(self.frontier_centroids)):
                f1 = self.frontier_centroids[i]
                f2 = self.frontier_centroids[j]
                dist = math.hypot(f1[0] - f2[0], f1[1] - f2[1])
                frontier_distances.append((dist, f1, f2))
        
        if not frontier_distances:
            self.get_logger().info('Nessuna coppia di frontiere valida per il calcolo della distanza. Salto l\'allocazione.')
            return

        frontier_distances.sort(key=lambda x: x[0], reverse=True)
        most_distant_frontiers = [frontier_distances[0][1], frontier_distances[0][2]]
        self.get_logger().info(f"Le due frontiere più distanti sono: {most_distant_frontiers[0]} e {most_distant_frontiers[1]}.")

        # 2. Assegna una delle due frontiere a ogni robot in base a quale è più vicina
        assigned_frontiers = set()
        for robot_name in ready_robots:
            # Reset recovery mode per robot che non sono in recovery
            self.recovery_mode[robot_name] = False
            
            # Ottieni la posa del robot nel frame globale 'map'
            robot_pose_stamped = PoseStamped()
            robot_pose_stamped.header.frame_id = f'{robot_name}/{robot_name}/odom'
            robot_pose_stamped.header.stamp = self.get_clock().now().to_msg()
            
            robot_pose_stamped.pose.position = self.robot_poses[robot_name].position
            robot_pose_stamped.pose.orientation = self.robot_poses[robot_name].orientation

            try:
                latest_time = self.tf_buffer.get_latest_common_time('map', f'{robot_name}/{robot_name}/odom')
                robot_pose_stamped.header.stamp = latest_time.to_msg()
                
                robot_pose_in_map = self.tf_buffer.transform(
                    robot_pose_stamped, 
                    'map', 
                    timeout=rclpy.duration.Duration(seconds=2.0)
                )
            except Exception as e:
                self.get_logger().error(f"Errore di trasformazione della posa del robot {robot_name}: {e}")
                continue

            # Calcola le distanze del robot dalle due frontiere più lontane
            dists = []
            for f in most_distant_frontiers:
                if f not in assigned_frontiers:
                    d = math.hypot(f[0] - robot_pose_in_map.pose.position.x, f[1] - robot_pose_in_map.pose.position.y)
                    dists.append((d, f))
            
            # Se nessuna delle due frontiere più distanti è disponibile, scegli la più vicina tra le rimanenti
            if not dists:
                self.get_logger().warn(f"Nessuna delle frontiere più distanti è disponibile per il robot {robot_name}. Scelgo la più vicina tra quelle rimanenti.")
                available_frontiers = [f for f in self.frontier_centroids if f not in assigned_frontiers]
                if available_frontiers:
                    closest_frontier = min(available_frontiers, key=lambda f: math.hypot(f[0] - robot_pose_in_map.pose.position.x, f[1] - robot_pose_in_map.pose.position.y))
                    self.send_navigation_goal(robot_name, closest_frontier)
                    self.assigned_goals[robot_name] = closest_frontier
                    self.update_goal_history(robot_name, closest_frontier)
                    assigned_frontiers.add(closest_frontier)
                continue

            # Scegli la frontiera più vicina al robot
            dists.sort(key=lambda x: x[0])
            chosen_frontier = dists[0][1]
            
            self.get_logger().info(f"Robot {robot_name} è più vicino a {chosen_frontier}. Assegno questa frontiera.")
            
            self.send_navigation_goal(robot_name, chosen_frontier)
            self.assigned_goals[robot_name] = chosen_frontier
            self.update_goal_history(robot_name, chosen_frontier)
            assigned_frontiers.add(chosen_frontier)

    def send_navigation_goal(self, robot_name, goal_coords):
        """
        Invia un goal di navigazione al Nav2 del robot specificato.
        Il goal viene trasformato dal frame globale 'map' al frame locale del robot
        prima di essere inviato.
        """
        # Crea il goal nel frame globale 'map'
        goal_pose_stamped = PoseStamped()
        goal_pose_stamped.header.frame_id = 'map'
        goal_pose_stamped.header.stamp = self.get_clock().now().to_msg()
        goal_pose_stamped.pose.position.x = goal_coords[0]
        goal_pose_stamped.pose.position.y = goal_coords[1]
        # L'orientamento viene impostato a un valore neutro (w=1.0)
        goal_pose_stamped.pose.orientation.w = 1.0

        try:
            # Trasforma il goal dal frame 'map' al frame locale del robot (es. 'robot1/map')
            latest_time = self.tf_buffer.get_latest_common_time('map', f'{robot_name}/{robot_name}/map')
            goal_pose_stamped.header.stamp = latest_time.to_msg()
            
            transformed_goal = self.tf_buffer.transform(
                goal_pose_stamped,
                f'{robot_name}/{robot_name}/map',
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
            
            # Imposta il frame_id corretto dopo la trasformazione
            transformed_goal.header.frame_id = f'{robot_name}/{robot_name}/map'
            transformed_goal.header.stamp = self.get_clock().now().to_msg()
            
            recovery_info = " (MODALITÀ RECOVERY)" if self.recovery_mode[robot_name] else ""
            self.get_logger().info(f"Trasformazione riuscita per {robot_name}{recovery_info}. Goal globale: ({goal_coords[0]:.2f}, {goal_coords[1]:.2f}) -> Goal locale: ({transformed_goal.pose.position.x:.2f}, {transformed_goal.pose.position.y:.2f}).")
            
            nav_client = self.nav_clients[robot_name]
            if not nav_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().warn(f"Nav2 server non disponibile per {robot_name}.")
                self.assigned_goals[robot_name] = None  # Reset assignment on failure
                return
            
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = transformed_goal
            
            self.get_logger().info(f"Invio del goal trasformato a {robot_name} nel frame {transformed_goal.header.frame_id}{recovery_info}.")
            
            # Invia il goal e registra i callback per la risposta e il risultato
            future = nav_client.send_goal_async(goal_msg)
            future.add_done_callback(lambda future, robot=robot_name: self.goal_response_callback(future, robot))

        except Exception as e:
            self.get_logger().error(f"Errore critico di trasformazione per {robot_name}: {type(e).__name__}: {e}")
            self.get_logger().error(f"Controllare che il tf da 'map' a '{robot_name}/{robot_name}/map' sia pubblicato.")
            self.assigned_goals[robot_name] = None  # Reset assignment on failure
            return

    def goal_response_callback(self, future, robot_name):
        """
        Callback per gestire la risposta del server Nav2 dopo l'invio di un goal.
        Controlla se il goal è stato accettato.
        """
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().warn(f'Goal rifiutato da Nav2 per {robot_name}.')
                
                # Se il goal di recovery è stato rifiutato, si tenta il fallback immediato
                if self.recovery_mode[robot_name]:
                    self.get_logger().warn(f'Goal di recovery rifiutato per {robot_name}, provo con frontiera più vicina al robot.')
                    self.handle_recovery_goal_failure(robot_name)
                else:
                    self.assigned_goals[robot_name] = None
                return
            
            recovery_info = " (MODALITÀ RECOVERY)" if self.recovery_mode[robot_name] else ""
            self.get_logger().info(f'Goal accettato da Nav2 per {robot_name}{recovery_info}.')
            self.goal_handles[robot_name] = goal_handle
            
            # PROTEZIONE SELETTIVA: L'assegnazione viene mantenuta solo in recovery mode.
            # In modalità normale, il goal viene resettato subito per permettere al timer
            # di assegnare un nuovo task a un altro robot mentre il primo è in movimento.
            if not self.recovery_mode[robot_name]:
                self.assigned_goals[robot_name] = None
            
            # Registra il callback per il risultato finale
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(lambda future, robot=robot_name: self.goal_result_callback(future, robot))
            
        except Exception as e:
            self.get_logger().error(f"Errore nella risposta del goal per {robot_name}: {e}")
            
            # Gestione errori in modalità recovery
            if self.recovery_mode[robot_name]:
                self.get_logger().error(f'Errore nel goal di recovery per {robot_name}, provo con frontiera più vicina al robot.')
                self.handle_recovery_goal_failure(robot_name)
            else:
                self.assigned_goals[robot_name] = None

    def handle_recovery_goal_failure(self, robot_name):
        """
        Gestisce il fallimento di un goal di recovery (rifiuto o errore).
        Trova un nuovo goal (la frontiera più vicina al robot) e lo assegna.
        """
        problematic_goal = self.assigned_goals[robot_name]
        available_frontiers = [f for f in self.frontier_centroids if f != problematic_goal]
        
        if available_frontiers:
            fallback_goal = self.find_closest_frontier_to_robot(robot_name, available_frontiers)
            
            if fallback_goal:
                self.get_logger().info(f"Tentativo fallback per {robot_name}: goal più vicino al robot {fallback_goal}")
                self.send_navigation_goal(robot_name, fallback_goal)
                self.assigned_goals[robot_name] = fallback_goal
                self.update_goal_history(robot_name, fallback_goal)
            else:
                self.get_logger().error(f"Impossibile trovare goal di fallback per {robot_name}.")
                self.assigned_goals[robot_name] = None
                self.recovery_mode[robot_name] = False
        else:
            self.get_logger().error(f"Nessuna frontiera disponibile per fallback di {robot_name}.")
            self.assigned_goals[robot_name] = None
            self.recovery_mode[robot_name] = False

    def goal_result_callback(self, future, robot_name):
        """
        Callback per gestire il risultato finale di un goal di navigazione.
        """
        try:
            result = future.result()
            recovery_info = " (MODALITÀ RECOVERY)" if self.recovery_mode[robot_name] else ""
            self.get_logger().info(f'Goal completato per {robot_name}{recovery_info} con stato: {result.status}')
            
            # Segna che il goal è stato completato
            self.goal_completion_status[robot_name] = True
            
        except Exception as e:
            self.get_logger().error(f"Errore nel risultato del goal per {robot_name}: {e}")
        finally:
            # Reset dello stato del robot
            self.assigned_goals[robot_name] = None
            self.goal_handles[robot_name] = None
            self.goal_completion_status[robot_name] = False
            
            # Reset recovery mode solo dopo che il goal di recovery è stato completato
            if self.recovery_mode[robot_name]:
                self.get_logger().info(f'Robot {robot_name} esce dalla modalità recovery.')
                self.recovery_mode[robot_name] = False
            
            self.get_logger().info(f'Robot {robot_name} è ora disponibile per nuovi goal.')

    def send_robots_home(self):
        """
        Invia tutti i robot alle loro posizioni iniziali quando l'esplorazione è completata.
        """
        for robot_name in self.robots:
            # Controlla se il robot ha una posizione iniziale, non ha un goal attivo e non è in recovery
            if (self.initial_positions[robot_name] is not None and 
                self.assigned_goals[robot_name] is None and
                not self.recovery_mode[robot_name]):
                
                initial_pos = self.initial_positions[robot_name]
                self.get_logger().info(f"Invio {robot_name} alla posizione iniziale: {initial_pos}")
                
                # Trasforma la posizione iniziale (che è in odom) al frame globale map
                try:
                    initial_pose_stamped = PoseStamped()
                    initial_pose_stamped.header.frame_id = f'{robot_name}/{robot_name}/odom'
                    initial_pose_stamped.header.stamp = self.get_clock().now().to_msg()
                    initial_pose_stamped.pose.position.x = initial_pos[0]
                    initial_pose_stamped.pose.position.y = initial_pos[1]
                    initial_pose_stamped.pose.orientation.w = 1.0
                    
                    latest_time = self.tf_buffer.get_latest_common_time(f'{robot_name}/{robot_name}/odom', 'map')
                    initial_pose_stamped.header.stamp = latest_time.to_msg()
                    
                    initial_pose_in_map = self.tf_buffer.transform(
                        initial_pose_stamped,
                        'map',
                        timeout=rclpy.duration.Duration(seconds=2.0)
                    )
                    
                    home_coords = (initial_pose_in_map.pose.position.x, initial_pose_in_map.pose.position.y)
                    
                    self.send_navigation_goal(robot_name, home_coords)
                    self.assigned_goals[robot_name] = home_coords
                    
                except Exception as e:
                    self.get_logger().error(f"Errore nell'invio di {robot_name} a casa: {e}")

def main(args=None):
    """
    Funzione principale per l'esecuzione del nodo.
    """
    rclpy.init(args=args)
    node = GreedyTaskAllocator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
