#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Quaternion, Pose
from std_srvs.srv import Trigger
from std_msgs.msg import Empty, UInt8
from gazebo_msgs.srv import SpawnEntity, DeleteEntity


class PduNode(Node):
    def __init__(self):
        super().__init__('pdu_node')

        self.probe_number = 5
        self.probe_cnt = 0

        self.declare_parameter('probe_model', '')

        self.probe_basename = 'probe'
        self.spawn_reference_frame = 'base_footprint'

        pdu_translation = {'x': -0.195, 'y': 0.0, 'z': 0.12}
        self.pdu_pose = Pose(
            position=Point(x=float(pdu_translation['x']), y=float(pdu_translation['y']), z=float(pdu_translation['z'])),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )

        self.pdu_dropped_pub = self.create_publisher(UInt8, 'probe_deployment_unit/probes_dropped', 1)
        self.pdu_dropped_pub.publish(UInt8(data=self.probe_cnt))

        self.spawn_probe_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_probe_client = self.create_client(DeleteEntity, '/delete_entity')

        self.pdu_reset_srv = self.create_service(Trigger, 'probe_deployment_unit/home', self.pdu_reset)
        self.pdu_drop_sub = self.create_subscription(Empty, 'probe_deployment_unit/drop', self.pdu_callback, 1)

        probe_model_path = self.get_parameter('probe_model').get_parameter_value().string_value
        try:
            with open(probe_model_path, 'r') as file:
                self.probe_model = file.read()
        except FileNotFoundError:
            self.get_logger().error(f"File not found: {probe_model_path}")
            self.probe_model = ''
        except Exception as e:
            self.get_logger().error(f"Failed to read probe model file: {e}")
            self.probe_model = ''
        print(self.probe_model)
        # Wait for the service to be available
        self.get_logger().info('Waiting for /spawn_entity service...')
        self.spawn_probe_client.wait_for_service()
        self.get_logger().info('/spawn_entity service is now available.')

        self.get_logger().info('Waiting for /delete_entity service...')
        self.delete_probe_client.wait_for_service()
        self.get_logger().info('/delete_entity service is now available.')

    def pdu_reset(self, req: Trigger.Request, res: Trigger.Response):
        for i in range(1, self.probe_cnt + 1):
            probe_name = f'{self.probe_basename}{i}'
            request = DeleteEntity.Request()
            request.name = probe_name
            future = self.delete_probe_client.call_async(request)
            future.add_done_callback(self.delete_callback)

        self.probe_cnt = 0
        self.pdu_dropped_pub.publish(UInt8(data=self.probe_cnt))

        res.success = True
        res.message = "Probes have been reset."
        return res

    def pdu_callback(self, msg: Empty):
        if self.probe_cnt >= self.probe_number or not self.probe_model:
            self.get_logger().info(f"Max probes reached or no probe model. Probe count: {self.probe_cnt}, Model: {self.probe_model != ''}")
            return

        self.probe_cnt += 1
        probe_name = f'{self.probe_basename}{self.probe_cnt}'

        request = SpawnEntity.Request()
        request.name = probe_name
        request.xml = self.probe_model
        request.robot_namespace = ''
        request.initial_pose = self.pdu_pose
        request.reference_frame = self.spawn_reference_frame

        self.get_logger().info(f"Spawning probe: {probe_name}")

        future = self.spawn_probe_client.call_async(request)
        future.add_done_callback(self.spawn_callback)

        self.pdu_dropped_pub.publish(UInt8(data=self.probe_cnt))

    def spawn_callback(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f"Successfully spawned probe: {response.status_message}")
            else:
                self.get_logger().error(f"Failed to spawn probe: {response.status_message}")
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')

    def delete_callback(self, future):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error(f"Failed to delete probe: {response.status_message}")
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = PduNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
