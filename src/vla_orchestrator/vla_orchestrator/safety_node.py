import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

class SafetyNode(Node):
    def __init__(self):
        super().__init__('safety_node')

        # TRANSIENT_LOCAL: 새로 구독해도 마지막 값 받음
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # /emergency_stop 발행
        self.estop_pub = self.create_publisher(Bool, '/emergency_stop', qos)

        # /set_estop 서비스 서버
        self.create_service(SetBool, '/set_estop', self.set_estop_cb)

        # 시작 시 False 발행 (정상 상태)
        self.publish_estop(False)
        self.get_logger().info('SafetyNode started! 비상정지: OFF')

    def set_estop_cb(self, req, res):
        self.publish_estop(req.data)
        state = "ON 🚨" if req.data else "OFF ✅"
        self.get_logger().info(f'비상정지: {state}')
        res.success = True
        res.message = f'estop={req.data}'
        return res

    def publish_estop(self, value: bool):
        msg = Bool()
        msg.data = value
        self.estop_pub.publish(msg)

def main():
    rclpy.init()
    node = SafetyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()