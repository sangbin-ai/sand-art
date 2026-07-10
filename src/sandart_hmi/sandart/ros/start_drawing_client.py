import rclpy

from rclpy.node import Node

from sandart_msgs.srv import StartDrawing


class StartDrawingClient(Node):

    def __init__(self):
        super().__init__("start_drawing_client")

        self.client = self.create_client(
            StartDrawing,
            "/dsr01/start_drawing",
        )

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting /dsr01/start_drawing...")

    def send(self, speed, force, tool):

        req = StartDrawing.Request()

        req.speed = float(speed)
        req.force = float(force)
        req.tool = str(tool)

        future = self.client.call_async(req)

        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            return False, "Service call failed"

        return future.result().success, future.result().message