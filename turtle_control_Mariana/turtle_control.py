# (i) subscrever ao tópico de pose do turtlesim, 
# (ii) subscrever ao tópico /goal 3, onde são publicadas mensagens do tipo geometry_msgs/msg/Pose2D,
# (iii) publicar no tópico de comando de velocidade do turtlesim.
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from turtlesim.msg import Pose

# TODO
# ele não está conseguindo calcular o erro corretamente 
# exemplo: coloco a meta para x=11 e y=11 e ele consegue ajustar o x rapidamente mas o y ele demora MUITO ...cresce de 0.01 em 0.01
# ele ainda tá com um erro grande...

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_control')

        print("Aqui será inicializado o nó")

        # Inicializa os componentes do nó
        self.init_publisher()
        self.init_subscribers()
        self.init_variables()


    # def timer_callback(self):
    #     msg = String()
    #     msg.data = 'Hello World: %d' % self.i
    #     self.publisher_.publish(msg)
    #     self.get_logger().info('Publishing: "%s"' % msg.data)
    #     self.i += 1


    def init_publisher(self):

        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.pub_callback)

        print("Publisher iniciado")

    def init_subscribers(self):

        # verifica a posição inicial da tartaruga
        self.create_subscription(Pose, '/turtle1/pose', self.pose_callback, 10)
        # verifica o goal 
        self.create_subscription(Pose2D, '/goal', self.goal_callback, 10)

        print("Subscribers iniciados")

    def init_variables(self):

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.x_goal = None
        self.y_goal = None
        self.k_omega = 2.0
        print("Variáveis inicializadas.")

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
        print(f"Novo objetivo recebido: xd={self.x_goal}, yd={self.y_goal}")

    def pub_callback(self):

        """Computa erro e publica comandos de velocidade"""
        if self.x_goal is None or self.y_goal is None:
            return  # Sem objetivo definido ainda

        # Calcula erro de posição 
        x_error = self.x_goal - self.x
        y_error = self.y_goal - self.y
        rho = math.sqrt(x_error**2 + y_error**2)
        alpha = math.atan2(y_error, x_error) - self.theta

        # Definir limiar de erro para parar a tartaruga
        if rho < 0.1:
            v, omega = 0.0, 0.0
        else:
            vmax = 1.0
            k_omega = 2.0
            v = vmax * math.tanh(rho)
            omega = k_omega * alpha

        # Publica o comando de velocidade
        cmd = Twist()
        cmd.linear.x = v
        cmd.angular.z = omega
        self.publisher_.publish(cmd)
        print(f"Enviando velocidade: v={v}, omega={omega}")


def main():
    print('Hi from turtle_control_Mariana.')
    rclpy.init()
    node = TurtleController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


