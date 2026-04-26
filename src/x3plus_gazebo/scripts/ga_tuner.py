#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped 
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import numpy as np
import time
import math

class GeneticTunerNode(Node):
    def __init__(self):
        super().__init__('ga_tuner_node')
        
        self.cmd_pub = self.create_publisher(TwistStamped, '/omni_base_controller/reference', 10)
        self.odom_sub = self.create_subscription(Odometry, '/omni_base_controller/odometry', self.odom_callback, 10)
        
        # NUEVO: Suscriptor del Sensor IMU
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        
        self.current_x = 0.0
        self.current_y = 0.0
        self.max_angular_spin = 0.0 # Guardará la perturbación máxima
        
        self.get_logger().info("Nodo de Sintonización Genética con IMU Inicializado.")

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def imu_callback(self, msg):
        # Leemos la velocidad angular en Z real desde el giroscopio virtual
        # El ruido gaussiano hará que esto nunca sea un 0 perfecto
        spin = abs(msg.angular_velocity.z)
        if spin > self.max_angular_spin:
            self.max_angular_spin = spin

    def evaluate_fitness(self, kp, ki, kd, target_distance=1.0, max_time=4.0):
        start_x = self.current_x
        start_y = self.current_y
        self.max_angular_spin = 0.0 # Reseteamos el giroscopio para esta prueba
        
        twist_msg = TwistStamped()
        twist_msg.header.frame_id = "base_footprint"
        
        start_time = time.time()
        error_integral = 0.0
        
        # Bucle de Control (Simulando la respuesta del PID Cinemático)
        while (time.time() - start_time) < max_time:
            dist_traveled = math.sqrt((self.current_x - start_x)**2 + (self.current_y - start_y)**2)
            error = target_distance - dist_traveled
            
            # Aplicamos las ganancias propuestas por el Genoma
            control_signal = (kp * error) + (ki * error_integral) # Simplificado para la prueba
            
            # Saturación de velocidad de seguridad
            twist_msg.twist.linear.x = min(max(control_signal, -0.5), 0.5) 
            twist_msg.header.stamp = self.get_clock().now().to_msg()
            self.cmd_pub.publish(twist_msg)
            
            rclpy.spin_once(self, timeout_sec=0.1)
            
            current_t = time.time() - start_time
            error_integral += current_t * abs(error) * 0.1 # ITAE
            
            if dist_traveled >= target_distance and abs(error) < 0.05:
                break
                
        # Freno 
        twist_msg.twist.linear.x = 0.0
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        self.cmd_pub.publish(twist_msg)
        time.sleep(0.5) 
        
        # === CÁLCULO DE LA FUNCIÓN DE COSTO ===
        # 1. Castigamos si se desvió de su carril
        deviation_penalty = abs(self.current_y - start_y) * 50.0
        
        # 2. Castigamos severamente si el IMU detectó vibraciones o giros anómalos
        # Esto obliga al robot a mantener la orientación perfecta para el Dofbot
        imu_penalty = self.max_angular_spin * 200.0 
        
        return error_integral + deviation_penalty + imu_penalty

def run_genetic_algorithm(node):
    POPULATION_SIZE = 8
    GENERATIONS = 10
    MUTATION_RATE = 0.15
    
    # [Kp, Ki, Kd]
    population = np.random.rand(POPULATION_SIZE, 3)
    population[:, 0] *= 5.0 # Kp entre 0 y 5
    population[:, 1] *= 1.0 # Ki entre 0 y 1
    population[:, 2] *= 1.0 # Kd entre 0 y 1
    
    node.get_logger().info("=== INICIANDO EVOLUCIÓN CON FEEDBACK IMU ===")
    
    for gen in range(GENERATIONS):
        node.get_logger().info(f"\n--- Generación {gen+1}/{GENERATIONS} ---")
        fitness_scores = []
        
        for i, ind in enumerate(population):
            kp, ki, kd = ind
            fit = node.evaluate_fitness(kp, ki, kd)
            fitness_scores.append(fit)
            node.get_logger().info(f"Ind {i} [Kp:{kp:.2f} Ki:{ki:.2f} Kd:{kd:.2f}] -> Costo: {fit:.2f}")
            
        fitness_scores = np.array(fitness_scores)
        
        # Selección por Torneo
        parents_indices = np.argsort(fitness_scores)[:POPULATION_SIZE//2]
        parents = population[parents_indices]
        
        node.get_logger().info(f"*** Mejor costo (Gen {gen+1}): {fitness_scores[parents_indices[0]]:.2f} ***")
        
        next_population = []
        for _ in range(POPULATION_SIZE):
            p1, p2 = parents[np.random.choice(len(parents), 2, replace=False)]
            crossover_point = np.random.randint(1, 3)
            child = np.concatenate([p1[:crossover_point], p2[crossover_point:]])
            
            # Mutación
            if np.random.rand() < MUTATION_RATE:
                child[np.random.randint(3)] += np.random.uniform(-0.5, 0.5)
            
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