import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped, PoseStamped
from ur_msgs.msg import ToolDataMsg
from std_msgs.msg import Float64
import csv
import time


class ForceEstimationDataLogger(Node):
    def __init__(self):
        super().__init__('force_estimation_data_logger')

        # Subscriptions
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        self.force_torque_sub = self.create_subscription(
            WrenchStamped,
            '/force_torque_sensor_broadcaster/wrench',
            self.force_torque_callback,
            10
        )
        self.tcp_pose_sub = self.create_subscription(
            PoseStamped,
            '/tcp_pose_broadcaster/pose',
            self.tcp_pose_callback,
            10
        )
        self.tool_data_sub = self.create_subscription(
            ToolDataMsg,
            '/io_and_status_controller/tool_data',
            self.tool_data_callback,
            10
        )
        self.speed_scaling_sub = self.create_subscription(
            Float64,
            '/speed_scaling_state_broadcaster/speed_scaling',
            self.speed_scaling_callback,
            10
        )

        # Initialize data storage
        self.joint_data = {
            'positions': [],
            'velocities': [],
            'efforts': []
        }
        self.force_torque_data = {
            'force_x': 0.0,
            'force_y': 0.0,
            'force_z': 0.0,
            'torque_x': 0.0,
            'torque_y': 0.0,
            'torque_z': 0.0
        }
        self.tcp_pose_data = {
            'position_x': 0.0,
            'position_y': 0.0,
            'position_z': 0.0,
            'orientation_x': 0.0,
            'orientation_y': 0.0,
            'orientation_z': 0.0,
            'orientation_w': 0.0
        }
        self.tool_data = {
            'tool_current': 0.0
        }
        self.speed_scaling = 0.0

        # Open CSV file
        self.file = open('force_estimation_data.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.file)

        # Write CSV header
        header = (
            ['timestamp']
            + ['position_' + str(i) for i in range(6)]
            + ['velocity_' + str(i) for i in range(6)]
            + ['effort_' + str(i) for i in range(6)]
            + ['force_x', 'force_y', 'force_z', 'torque_x', 'torque_y', 'torque_z']
            + ['tcp_pos_x', 'tcp_pos_y', 'tcp_pos_z']
            + ['tcp_ori_x', 'tcp_ori_y', 'tcp_ori_z', 'tcp_ori_w']
            + ['tool_current', 'speed_scaling']
        )
        self.csv_writer.writerow(header)

    def joint_state_callback(self, msg):
        self.joint_data['positions'] = msg.position
        self.joint_data['velocities'] = msg.velocity
        self.joint_data['efforts'] = msg.effort

    def force_torque_callback(self, msg):
        self.force_torque_data['force_x'] = msg.wrench.force.x
        self.force_torque_data['force_y'] = msg.wrench.force.y
        self.force_torque_data['force_z'] = msg.wrench.force.z
        self.force_torque_data['torque_x'] = msg.wrench.torque.x
        self.force_torque_data['torque_y'] = msg.wrench.torque.y
        self.force_torque_data['torque_z'] = msg.wrench.torque.z

    def tcp_pose_callback(self, msg):
        self.tcp_pose_data['position_x'] = msg.pose.position.x
        self.tcp_pose_data['position_y'] = msg.pose.position.y
        self.tcp_pose_data['position_z'] = msg.pose.position.z
        self.tcp_pose_data['orientation_x'] = msg.pose.orientation.x
        self.tcp_pose_data['orientation_y'] = msg.pose.orientation.y
        self.tcp_pose_data['orientation_z'] = msg.pose.orientation.z
        self.tcp_pose_data['orientation_w'] = msg.pose.orientation.w

    def tool_data_callback(self, msg):
        self.tool_data['tool_current'] = msg.tool_current

    def speed_scaling_callback(self, msg):
        self.speed_scaling = msg.data

    def write_to_csv(self):
        timestamp = time.time()
        row = (
            [timestamp]
            + list(self.joint_data['positions'])
            + list(self.joint_data['velocities'])
            + list(self.joint_data['efforts'])
            + [
                self.force_torque_data['force_x'],
                self.force_torque_data['force_y'],
                self.force_torque_data['force_z'],
                self.force_torque_data['torque_x'],
                self.force_torque_data['torque_y'],
                self.force_torque_data['torque_z']
            ]
            + [
                self.tcp_pose_data['position_x'],
                self.tcp_pose_data['position_y'],
                self.tcp_pose_data['position_z'],
                self.tcp_pose_data['orientation_x'],
                self.tcp_pose_data['orientation_y'],
                self.tcp_pose_data['orientation_z'],
                self.tcp_pose_data['orientation_w']
            ]
            + [self.tool_data['tool_current'], self.speed_scaling]
        )
        self.csv_writer.writerow(row)
        self.get_logger().info(f"Logged data: {row}")

    def destroy_node(self):
        self.file.close()
        super().destroy_node()

    def timer_callback(self):
        self.write_to_csv()


def main():
    rclpy.init()
    node = ForceEstimationDataLogger()
    timer_period = 0.1  # Log data every 0.1 seconds
    node.create_timer(timer_period, node.timer_callback)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

