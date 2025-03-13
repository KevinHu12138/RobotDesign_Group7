import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from interbotix_xs_modules.xs_robot.arm import InterbotixManipulatorXS
import time

class PublisherControlNode(Node):
    def __init__(self):
        super().__init__('publisher_control_node')
        
        # 初始化 Interbotix 机械臂
        self.arm = InterbotixManipulatorXS(robot_model='px150', group_name='arm', gripper_name='gripper')
        
        # 订阅颜色识别结果
        self.subscription = self.create_subscription(
            String,
            'color_detection',
            self.color_callback,
            10)
        self.get_logger().info('Subscribed to color_detection topic.')
        
    def color_callback(self, msg):
        """
        解析 color_detection 结果，并移动机械臂
        """
        self.get_logger().info(f'Received color detection data: {msg.data}')
        
        try:
            parts = msg.data.split(';')
            if len(parts) != 2:
                raise ValueError("Invalid message format received from color_detection")

            color, position_data = parts
            x_3d, y_3d, z_3d = map(float, position_data.split(','))

            # **修正坐标转换**
            x_corrected = max(0.05, min(0.4, z_3d))  # 机械臂的 x 对应相机的 -y，确保 x 在有效范围内
            y_corrected = max(0.05, min(0.4, -y_3d))   # 机械臂的 y 对应相机的 z
            z_corrected = max(0.05, min(0.3, abs(x_3d)))  # 机械臂的 z 对应相机的 -x，确保 z 是正值

            self.move_arm_to_target(x_corrected, y_corrected, z_corrected)
        except Exception as e:
            self.get_logger().error(f'Error processing color detection data: {e}')

    def move_arm_to_target(self, x, y, z):
        """
        让机械臂移动到目标积木块位置
        """
        self.get_logger().info(f'Moving arm to: x={x}, y={y}, z={z}')
        
        # 先移动到目标上方
        self.arm.arm.set_ee_pose_components(x=x, y=y, z=z)
        time.sleep(2)
        
        # 下降到目标位置
        self.arm.arm.set_ee_pose_components(x=x, y=y, z=z)
        time.sleep(1)
        
        # 执行抓取
        self.arm.gripper.grasp()
        time.sleep(1)
        
        # 提升机械臂
        self.arm.arm.set_ee_pose_components(x=x, y=y, z=z)
        time.sleep(1)
        
        self.release_block()
    
    def release_block(self):
        """
        释放积木块到固定位置
        """
        self.get_logger().info('Releasing block at: x=0.5, y=0.0, z=0.2')
        self.arm.arm.set_ee_pose_components(x=0.5, y=0.0, z=0.2)
        time.sleep(2)
        self.arm.gripper.release()
        

def main(args=None):
    rclpy.init(args=args)
    node = PublisherControlNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
