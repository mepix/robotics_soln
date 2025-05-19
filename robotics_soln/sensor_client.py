import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from robotics_soln_msgs.srv import FilterSensorVec3D

class SensorClient(Node):

    def __init__(self):
        super().__init__('sensor_client')

        # Create data structure to hold the sensor data
        sensor0_name = 'sensor_0_filtered_data'
        sensor1_name = 'sensor_1_filtered_data'
        self.sensor0_data = None
        self.sensor1_data = None

        # Create Clients
        self.sensor0_client = self.create_client(FilterSensorVec3D, sensor0_name)
        while not self.sensor0_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning(f'service {sensor0_name} not available, waiting again...')

        self.sensor1_client = self.create_client(FilterSensorVec3D, sensor1_name)
        while not self.sensor1_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warning(f'service {sensor1_name} not available, waiting again...')

        # Sensor Topic Publisher
        self.publisher = self.create_publisher(Float64MultiArray, 'sensor_data_filtered', 10)
        self.timer_period = float(1/500)  # seconds
        self.timer = self.create_timer(self.timer_period, self.query_sensor0_callback)
        self.get_logger().info('Sensor Client is ready to send requests.')

    def query_sensor0_callback(self):
        """Callback function to query sensor 0 for filtered data.
        """
        self.get_logger().debug('Sending request to get filtered data from Sensor 0...')
        req = FilterSensorVec3D.Request()
        future = self.sensor0_client.call_async(req)
        future.add_done_callback(self.sensor0_response_callback)

    def sensor0_response_callback(self, future):
        """Callback function to handle the response from sensor 0
        and request sensor 1.
        """
        try:
            response = future.result()
            if response is not None:
                self.get_logger().debug(f'Sensor 0: {response.x}, {response.y}, {response.z}')
                self.sensor0_data = response
        except Exception as e:
            self.get_logger().error(f"Service call failed for Sensor 0: {e}")

        # After processing sensor 0, request sensor 1
        req1 = FilterSensorVec3D.Request()
        future1 = self.sensor1_client.call_async(req1)
        future1.add_done_callback(self.sensor1_response_callback)

    def sensor1_response_callback(self, future):
        """Callback function to handle the response from sensor 1
        and publish the combined data.
        """
        try:
            response = future.result()
            if response is not None:
                self.get_logger().debug(f'Sensor 1: {response.x}, {response.y}, {response.z}')
                self.sensor1_data = response
        except Exception as e:
            self.get_logger().error(f"Service call failed for Sensor 1: {e}")
        
        # Publish the combined data
        if self.sensor0_data and self.sensor1_data:
            combined_data = Float64MultiArray()
            combined_data.data = [
                self.sensor0_data.x,
                self.sensor0_data.y,
                self.sensor0_data.z,
                self.sensor1_data.x,
                self.sensor1_data.y,
                self.sensor1_data.z
            ]
            self.publisher.publish(combined_data)
            self.get_logger().info(f'Published combined data: {combined_data.data}')

def main():
    rclpy.init()

    sensor_client = SensorClient()
    rclpy.spin(sensor_client)
  
    sensor_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()