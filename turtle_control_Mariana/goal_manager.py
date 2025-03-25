import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
import math
import time

class GoalManager(Node):
    def __init__(self):
        super().__init__('goal_manager')

        self.goals = [
            (2.0, 2.0, 0.0),
            (4.0, 4.0, 1.57),
            (6.0, 2.0, -1.57)
        ]
        self.current_goal_idx = 0

        self.goal_publisher = self.create_publisher(Pose2D, '/goal', 10)
        self.create_subscription(Odometry, '/odom', self.pose_callback, 10)

        self.x = 0.0
        self.y = 0.0

        self.publish_goal()

    def pose_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        goal_x, goal_y, _ = self.goals[self.current_goal_idx]
        distance = math.sqrt((goal_x - self.x) ** 2 + (goal_y - self.y) ** 2)

        if distance < 0.3:
            self.get_logger().info(f"Objetivo {self.current_goal_idx+1} alcançado.")
            self.current_goal_idx += 1
            if self.current_goal_idx < len(self.goals):
                self.publish_goal()
            else:
                self.get_logger().info("Todos os objetivos alcançados!")

    def publish_goal(self):
        goal_x, goal_y, goal_theta = self.goals[self.current_goal_idx]
        goal_msg = Pose2D()
        goal_msg.x = goal_x
        goal_msg.y = goal_y
        goal_msg.theta = goal_theta

        self.get_logger().info(f"Enviando novo objetivo: ({goal_x}, {goal_y}, θ={goal_theta})")
        self.goal_publisher.publish(goal_msg)
        time.sleep(1)

def main(args=None):
    rclpy.init(args=args)
    node = GoalManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
