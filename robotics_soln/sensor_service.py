from std_srvs.srv import Trigger
from robotics_soln_msgs.srv import FilterSensorVec3D
from robotics_soln.filter import Filter, FilterType

import rclpy
from rclpy.node import Node

import socket
import numpy as np
from std_msgs.msg import Float64MultiArray

class SensorService(Node):

    def __init__(self):
        super().__init__('sensor_service')

        # Handle Parameters
        self.declare_parameter('addr', '127.0.0.1')
        self.declare_parameter('port', 10000)
        self.declare_parameter('id', 0)
        self.addr = self.get_parameter('addr').get_parameter_value().string_value
        self.port = self.get_parameter('port').get_parameter_value().integer_value
        self.id = self.get_parameter('id').get_parameter_value().integer_value
        filter_name = f"sensor_{self.id}_filtered_data"

        # Create a filter object
        self.filter = Filter(window_size=5, filter_type=FilterType.MOVING_AVG)

        # Create a publisher to publish the sensor data
        self.publisher = self.create_publisher(Float64MultiArray, 'sensor_data_raw', 10)   
        self.sock = None
        self.timer_period = float(1/2000)  # seconds
        self.number_of_samples = 1
        self.connect_to_sensor()
        self.timer = self.create_timer(self.timer_period, self.poll_sensor_api)

        # Create a service to get filtered data
        self.sensor_data = None
        self.srv = self.create_service(FilterSensorVec3D, filter_name, self.get_filtered_data)

    def teardown(self):
        # Close the socket connection
        if self.sock:
            self.sock.close()
            self.sock = None
            self.get_logger().info('Socket closed.')

    def connect_to_sensor(self):
        # Create a TCP/IP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Connect the socket to the port where the server is listening
        server_address = (self.addr, self.port)
        print('connecting to {} port {}'.format(*server_address))
        self.sock.connect(server_address)

    def poll_sensor_api(self):
        """Use the socket to poll the sensor API and get the data.
        """
        if self.sock is None:
            self.get_logger().error('Socket is not connected. Cannot poll sensor API.')
            return

        # Request samples from the sensor
        message_string = str(self.number_of_samples)
        message = message_string.encode()
        self.sock.sendall(message)

        # Receive the data from the sensor
        byte_data = self.sock.recv(10000)
        self.sensor_data =  np.frombuffer(byte_data)

        # Publish the data in a msg
        msg = Float64MultiArray()
        msg.data = self.sensor_data
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')


    def get_filtered_data(self, request, response):
        if self.sensor_data is None:
            self.get_logger().error('No sensor data available.')
            return response

        # Filter the data
        data = np.array([self.sensor_data[0], self.sensor_data[1], self.sensor_data[2]])
        data_filtered = self.filter.update(data)

        response.x = data_filtered[0]
        response.y = data_filtered[1]
        response.z = data_filtered[2]

        return response


def main():
    rclpy.init()

    sensor_service = SensorService()

    rclpy.spin(sensor_service)

    rclpy.shutdown()

    sensor_service.teardown()

if __name__ == '__main__':
    main()