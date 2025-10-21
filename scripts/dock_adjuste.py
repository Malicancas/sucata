#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_srvs.srv import Trigger
import math

class DockingAdjust(Node):
    def __init__(self):
        super().__init__('docking_adjust')

        # Publicador de velocidade
        self.cmd_pub = self.create_publisher(TwistStamped, '/diff_cont/cmd_vel', 10)
        # Subscrição da odometria
        self.pose_sub = self.create_subscription(Odometry, '/odom', self.pose_callback, 10)

        # Serviço para ativar recuo
        self.srv = self.create_service(Trigger, 'activate_docking', self.handle_activate)

        # Estado do movimento
        self.moving_back = False
        self.start_x = None
        self.start_y = None
        self.target_distance = 1.0  # metros
        self.speed = 0.5  # m/s

        # Área da docking (onde começa o recuo)
        self.docking_area = {'x_min': -1.0, 'x_max': 1.0, 'y_min': -1.0, 'y_max': 1.0}
        # Última posição conhecida
        self.current_x = 0.0
        self.current_y = 0.0

    def handle_activate(self, request, response):
        # Só ativa se não estiver já a recuar
        if not self.moving_back:
            self.get_logger().info("Serviço chamado: iniciando recuo da dock!")
            self.moving_back = True
            self.start_x = self.current_x
            self.start_y = self.current_y
            response.success = True
            response.message = "Recuo iniciado!"
        else:
            response.success = False
            response.message = "O robô já está a recuar!"
        return response

    def pose_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.current_x = x
        self.current_y = y

        if self.moving_back:
            distance = math.sqrt((x - self.start_x)**2 + (y - self.start_y)**2)
            if distance < self.target_distance:
                twist = TwistStamped()
                twist.twist.linear.x = -self.speed
                self.cmd_pub.publish(twist)
            else:
                twist = TwistStamped()
                twist.twist.linear.x = 0.0
                self.cmd_pub.publish(twist)
                self.get_logger().info(f"Parado após {distance:.2f} m percorridos.")
                self.moving_back = False  # só loga uma vez

def main(args=None):
    rclpy.init(args=args)
    node = DockingAdjust()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
