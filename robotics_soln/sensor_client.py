import rclpy
from rclpy.node import Node

from robotics_soln_msgs.srv import FilterSensorVec3D
from robotics_soln_msgs.msg import SensorVec3D


class SensorClient(Node):

    def __init__(self):
        super().__init__('sensor_client')
        self.cli = self.create_client(FilterSensorVec3D, 'sensor_0_filtered_data')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = FilterSensorVec3D.Request()

        # Sensor Topic Publisher
        self.publisher = self.create_publisher(SensorVec3D, 'sensor_data_filtered', 10)
        self.timer_period = float(1/500)  # seconds
        self.timer = self.create_timer(self.timer_period, self.query_sensor_callback)
        self.get_logger().info('Sensor Client is ready to send requests.')

    def query_sensor_callback(self):
        self.get_logger().debug('Sending request to get filtered data...')
        req = FilterSensorVec3D.Request()
        req.x_raw = 1.0
        req.y_raw = 2.0
        req.z_raw = 3.0

        future = self.cli.call_async(self.req)
        future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            if response is not None:
                self.get_logger().info(f"Result of get_filtered_data: x={response.x}, y={response.y}, z={response.z}")
                msg = SensorVec3D()
                msg.x = response.x
                msg.y = response.y
                msg.z = response.z
                self.publisher.publish(msg)
                self.get_logger().info(f'Publishing: {msg.data}')
        except Exception as e:
            # TODO: debug this and properly check resposne
            return
            self.get_logger().error(f"Service call failed: {e}")

def main():
    rclpy.init()

    sensor_client = SensorClient()
    rclpy.spin(sensor_client)
  
    sensor_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()