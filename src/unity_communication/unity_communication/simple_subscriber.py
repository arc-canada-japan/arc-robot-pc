from srv import HapticCommand

import rclpy
from rclpy.node import Node

class HapticDeviceService(Node):
    def __init__(self):
        super().__init__('unity_service')
        self.srv = self.create_service(HapticCommand, 'haptic_cmd', self.add_three_ints_callback)

    def add_three_ints_callback(self, request, response):
        response.ack = True
        self.get_logger().info(
            'Joints angles\n[%d ; %d ; %d ; %d ; %d]' % (request.joint1,
                                                              request.joint2,
                                                              request.joint3,
                                                              request.joint5,
                                                              request.joint5))

        return response
    
def main():
    rclpy.init()

    minimal_service = HapticDeviceService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()