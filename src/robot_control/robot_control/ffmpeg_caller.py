import rclpy
from rclpy.node import Node
import subprocess

class FfmpegCaller(Node):

    def __init__(self):
        super().__init__('ffmpeg_caller')
        # Declare and get parameters
        self.declare_parameter('ffmpeg_ip', '192.168.11.50')
        self.declare_parameter('ffmpeg_port', 8080)
        self.declare_parameter('video_x', 1280)
        self.declare_parameter('video_y', 720)
        self.declare_parameter('video_device', '/dev/video0')

        ip = self.get_parameter('ffmpeg_ip').get_parameter_value().string_value
        port = self.get_parameter('ffmpeg_port').get_parameter_value().integer_value
        video_x = self.get_parameter('video_x').get_parameter_value().integer_value
        video_y = self.get_parameter('video_y').get_parameter_value().integer_value
        video_device = self.get_parameter('video_device').get_parameter_value().string_value

        # Create the command string using an f-string
        command = f'ffmpeg -s {video_x}x{video_y} -i {video_device} -preset ultrafast -tune zerolatency -codec libx264 -f mpegts udp://{ip}:{port}'

        # Split the command string into a list of arguments
        command_list = command.split()

        # Run the command using subprocess
        subprocess.run(command_list)


def main(args=None):
    rclpy.init(args=args)

    caller = FfmpegCaller()

    rclpy.spin(caller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    caller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()