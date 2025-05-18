from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node

import socket
import numpy as np
from std_msgs.msg import String

class SensorService(Node):

    def __init__(self):
        super().__init__('sensor_service')

        # Create a publisher to publish the sensor data
        self.publisher = self.create_publisher(String, 'sensor_data_raw', 10)   
        self.sock = None
        self.timer_period = float(1/2000)  # seconds
        self.number_of_samples = 1
        self.connect_to_sensor()
        self.timer = self.create_timer(self.timer_period, self.poll_sensor_api)

        # Create a service to get filtered data
        # self.srv = self.create_service(GetFilteredData, 'get_filtered_data', self.get_filtered_data)

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
        server_address = ('127.0.0.3', 10000)
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
        data =  np.frombuffer(byte_data)

        # Publish the data in a msg
        msg = String()
        msg.data = str(data)
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


    def get_filtered_data(self, request, response):
        # TODO: filter the data
        
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))

        return response


def main():
    rclpy.init()

    sensor_service = SensorService()

    rclpy.spin(sensor_service)

    rclpy.shutdown()

    sensor_service.teardown()

if __name__ == '__main__':
    main()