import rclpy
from rclpy.node import Node
from control_msgs.msg import DynamicJointState
import csv
import time

class DynamicJointStateLogger(Node):
    def __init__(self):
        super().__init__('dynamic_joint_state_logger')

        # Subscribe to /dynamic_joint_states topic
        self.subscription = self.create_subscription(
            DynamicJointState,
            '/dynamic_joint_states',
            self.joint_state_callback,
            10
        )
        
        # Open the CSV file
        self.file = open('dynamic_joint_states.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.file)

        # Write CSV header
        header = ['timestamp'] + ['position_' + str(i) for i in range(6)] \
                + ['velocity_' + str(i) for i in range(6)] \
                + ['effort_' + str(i) for i in range(6)]
        self.csv_writer.writerow(header)

    def joint_state_callback(self, msg):
    
        positions = []
        velocities = []
        efforts = []
    
        for interface in msg.interface_values:
            if 'position' in interface.interface_names:
                positions = interface.values
            if 'velocity' in interface.interface_names:
                velocities = interface.values
            if 'effort' in interface.interface_names:
                efforts = interface.values


        timestamp = time.time()

    
        row = [timestamp] + list(positions) + list(velocities) + list(efforts)
        self.csv_writer.writerow(row)
        self.get_logger().info(f"Logged joint state: {row}")


    def destroy_node(self):
        # Close the CSV file when shutting down
        self.file.close()
        super().destroy_node()

def main():
    rclpy.init()
    node = DynamicJointStateLogger()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

