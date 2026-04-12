#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
# CAMBIO 1: Importamos TwistStamped en lugar de Twist
from geometry_msgs.msg import TwistStamped 
from nav_msgs.msg import Odometry
import numpy as np
import time
import math

class GeneticTunerNode(Node):
    def __init__(self):
        super().__init__('ga_tuner_node')
        
        
        self.cmd_pub = self.create_publisher(TwistStamped, '/omni_base_controller/cmd_vel', 10)
        
        self.odom_sub = self.create_subscription(Odometry, '/omni_base_controller/odom', self.odom_callback, 10)
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        
        self.get_logger().info("Nodo de Sintonización Genética Inicializado.")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def evaluate_fitness(self, kp, ki, kd, target_distance=1.0, max_time=5.0):
        start_x = self.current_x
        start_y = self.current_y
        
        # Preparamos el mensaje de alta seguridad
        twist_msg = TwistStamped()
        twist_msg.header.frame_id = "base_footprint"
        twist_msg.twist.linear.x = 0.5 # Velocidad de prueba
        
        start_time = time.time()
        error_integral = 0.0
        
        
        while (time.time() - start_time) < max_time:
            # Actualizamos el reloj interno del mensaje
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            self.cmd_pub.publish(twist_msg)
            
            rclpy.spin_once(self, timeout_sec=0.1)
            
            dist_traveled = math.sqrt((self.current_x - start_x)**2 + (self.current_y - start_y)**2)
            error = target_distance - dist_traveled
            
            current_t = time.time() - start_time
            error_integral += current_t * abs(error) * 0.1 
            
            if dist_traveled >= target_distance:
                break
                
        # Freno controlado al terminar la prueba
        twist_msg.twist.linear.x = 0.0
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        self.cmd_pub.publish(twist_msg)
        time.sleep(0.5) 
        
        deviation_penalty = abs(self.current_y - start_y) * 50.0
        rotation_penalty = abs(self.current_yaw) * 50.0
        
        return error_integral + deviation_penalty + rotation_penalty

def run_genetic_algorithm(node):
    POPULATION_SIZE = 10
    GENERATIONS = 15
    MUTATION_RATE = 0.1
    
    population = np.random.rand(POPULATION_SIZE, 3)
    population[:, 0] *= 10.0
    population[:, 1] *= 2.0
    population[:, 2] *= 5.0
    
    node.get_logger().info("=== INICIANDO EVOLUCIÓN PID ===")
    
    for gen in range(GENERATIONS):
        node.get_logger().info(f"--- Generación {gen+1}/{GENERATIONS} ---")
        fitness_scores = []
        
        for i, ind in enumerate(population):
            kp, ki, kd = ind
            fit = node.evaluate_fitness(kp, ki, kd)
            fitness_scores.append(fit)
            node.get_logger().info(f"Ind {i} [Kp:{kp:.2f} Ki:{ki:.2f} Kd:{kd:.2f}] -> Costo: {fit:.2f}")
            
        fitness_scores = np.array(fitness_scores)
        
        parents_indices = np.argsort(fitness_scores)[:POPULATION_SIZE//2]
        parents = population[parents_indices]
        
        best_cost = fitness_scores[parents_indices[0]]
        node.get_logger().info(f"*** Mejor costo de la gen: {best_cost:.2f} ***")
        
        next_population = []
        for _ in range(POPULATION_SIZE):
            p1, p2 = parents[np.random.choice(len(parents), 2, replace=False)]
            crossover_point = np.random.randint(1, 3)
            child = np.concatenate([p1[:crossover_point], p2[crossover_point:]])
            
            if np.random.rand() < MUTATION_RATE:
                child[np.random.randint(3)] += np.random.uniform(-1.0, 1.0)
            
            child = np.clip(child, 0.0, None)
            next_population.append(child)
            
        population = np.array(next_population)

    node.get_logger().info("=== EVOLUCIÓN TERMINADA ===")
    node.get_logger().info(f"Mejor PID Final: Kp={parents[0][0]:.2f}, Ki={parents[0][1]:.2f}, Kd={parents[0][2]:.2f}")

def main(args=None):
    rclpy.init(args=args)
    tuner_node = GeneticTunerNode()
    
    try:
        run_genetic_algorithm(tuner_node)
    except KeyboardInterrupt:
        pass
    finally:
        tuner_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()