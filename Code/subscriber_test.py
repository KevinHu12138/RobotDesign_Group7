import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import time

class SubscriberControlNode(Node):
    def __init__(self):
        super().__init__('subscriber_control_node')
        
        # 初始化 Interbotix 机械臂
        self.arm = InterbotixManipulatorXS(robot_model='px150', group_name='arm', gripper_name='gripper')
        
        # 订阅机械臂目标位置信息
        self.pose_subscription = self.create_subscription(
            PoseStamped,
            '/armControl/set/pose',
            self.position_callback,
            10)
        self.get_logger().info(f'Subscribed to /armControl/set/pose topic.')

    def position_callback(self, msg: PoseStamped):
        """
        处理接收到的机械臂目标位置信息，并执行相应的运动
        """
        x_3d = msg.pose.position.x
        y_3d = msg.pose.position.y
        z_3d = msg.pose.position.z

        # **修正坐标转换**
        x_corrected = max(0.05, min(0.4, z_3d))  # 机械臂的 x 对应相机的 -y
        y_corrected = max(0.05, min(0.4, -y_3d))   # 机械臂的 y 对应相机的 z（高度）
        z_corrected = max(0.05, min(0.3, abs(x_3d)))  # 机械臂的 z 对应相机的 -x，确保 z 是正值

        self.get_logger().info(f'Received Target Position: x={x_corrected}, y={y_corrected}, z={z_corrected}')
        self.execute_movement(x_corrected, y_corrected, z_corrected)
    
    def execute_movement(self, x, y, z):
        """
        执行机械臂运动到目标位置，并执行抓取/释放操作
        """
        self.get_logger().info(f'Moving arm to position: x={x}, y={y}, z={z}')
        
        # 先移动到目标上方
        self.arm.arm.set_ee_pose_components(x=x, y=y, z=z)
        time.sleep(2)
        
        # 下降到目标位置
        self.arm.arm.set_ee_pose_components(x=x, y=y, z=z)
        time.sleep(1)
        
        # 判断是否执行抓取或释放
        if z > 0.2:
            self.get_logger().info('Moving to pre-grasp position.')
        elif z <= 0.2 and z > 0.1:
            self.get_logger().info('Grabbing the block...')
            self.arm.gripper.grasp()
            time.sleep(1)
            self.get_logger().info('Block successfully grabbed.')
        elif z <= 0.1:
            self.get_logger().info('Releasing the block at destination.')
            self.arm.gripper.release()
        

def main(args=None):
    rclpy.init(args=args)
    node = SubscriberControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
