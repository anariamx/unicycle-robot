# (i) subscrever ao tópico de pose do turtlesim, 
# (ii) subscrever ao tópico /goal 3, onde são publicadas mensagens do tipo geometry_msgs/msg/Pose2D,
# (iii) publicar no tópico de comando de velocidade do turtlesim.

import rclpy
import math 
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from turtlesim.msg import Pose


class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_control')

        print("Aqui será inicializado o nó")

        # Inicializa os componentes do nó
        self.init_publisher()
        self.init_subscribers()
        self.init_variables()


    def init_publisher(self):
        # publica a velocidade da tartaruga a cada 0.5 segundos
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.pub_callback)

    def init_subscribers(self):

        # verifica a posição da tartaruga
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        # verifica o goal 
        self.create_subscription(Pose2D, '/goal', self.goal_callback, 10)


    def init_variables(self):

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.x_goal = None
        self.y_goal = None
        self.theta_goal = None


    def pose_callback(self, msg):

        """Callback para receber a pose atual da tartaruga"""

        self.x = msg.x
        self.y = msg.y
        self.theta = msg.theta

        print(f"Atualizando pose: x={self.x}, y={self.y}, theta={self.theta}")

    def goal_callback(self, msg):

        """Callback para receber a posição desejada"""

        self.x_goal = msg.x
        self.y_goal = msg.y
        self.theta_goal = msg.theta

        print(f"Novo objetivo recebido: xd={self.x_goal}, yd={self.y_goal}, theta={self.theta_goal}")

    def pub_callback(self):

        """Computa erro e publica comandos de velocidade"""
        if self.x_goal is None or self.y_goal is None:
            return  # Sem objetivo definido ainda

        # Calcula erro de posição 
        x_error = self.x_goal - self.x
        y_error = self.y_goal - self.y
        theta_error = self.theta_goal - self.theta
        
        # rho -> distância do robô até o objetivo
        rho = math.sqrt(x_error**2 + y_error**2) #quanto maior rho mais longe o robô está do objetivo
        
        # alfa -> ângulo entre a direção atual do robô e a linha reta até o objetivo
        alpha = math.atan2(y_error, x_error) - self.theta # se for zero o robô já está na direção correta, se não estiver ele precisa girar

        # Definir limiar de erro para parar a tartaruga
        if rho < 0.01:
            if abs(theta_error) > 0.01:  
                v = 0.0
                omega = 1.0 * theta_error 
            else:
                v = 0.0
                omega = 0.0  

        else:

            vmax = 1.0
            k_omega = 2.0
            
            #velocidade linear 
            v = vmax * math.tanh(rho) 

            #velocidade angular
            omega = k_omega * alpha

        # Publica o comando de velocidade
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = omega
        self.publisher_.publish(cmd)
        print(f"Enviando velocidade: v={v}, omega={omega}")


def main():
    rclpy.init()
    node = TurtleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


