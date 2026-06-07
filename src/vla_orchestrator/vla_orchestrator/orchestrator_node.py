import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import Point
from vla_interfaces.srv import DetectObjects, PlanTask
from vla_interfaces.action import ExecuteAction
from .orchestrator import Orchestrator

class OrchestratorNode(Node):
    def __init__(self):
        super().__init__('vla_orchestrator')

        detect_client = self.create_client(DetectObjects, '/vla/detect_objects')
        plan_client = self.create_client(PlanTask, '/vla/plan_task')
        execute_client = ActionClient(self, ExecuteAction, '/vla/execute_action')
        self.status_pub = self.create_publisher(String, '/vla/status', 10)

        self.orch = Orchestrator(
            logger=self.get_logger(),
            detect_client=detect_client,
            plan_client=plan_client,
            execute_client=execute_client,
            status_cb=self.publish_status,
        )

        self.create_subscription(String, '/vla/command', self.command_cb, 10)
        self.get_logger().info('Orchestrator started!')

    def command_cb(self, msg):
        self.orch.handle_command(msg.data)

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)

def main():
    rclpy.init()
    node = OrchestratorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()