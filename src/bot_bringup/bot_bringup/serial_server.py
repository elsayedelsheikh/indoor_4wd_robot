#!/usr/bin/env python3
import serial
import time
import rclpy
from rclpy.node import Node
from bot_msgs.srv import ArduinoComm

class SerialServer(Node):

    def __init__(self):
        super().__init__('arduino_serial_server')

        ## Get parameters
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('serial_timeout', 0.2)
        self.declare_parameter('debug', False)

        serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.serial_timeout = self.get_parameter('serial_timeout').get_parameter_value().double_value
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        
        ## Set up serial port
        self.ser = serial.Serial(serial_port, baud_rate, timeout=self.serial_timeout)
        if self.debug:
            self.get_logger().info("Serial port %s opened" %(serial_port))
        
        while not self.ser.is_open:
            self.ser.open()

        ## Set up service
        self.service = self.create_service(ArduinoComm, 'command_arduino', self.command_callback)

        ## Variables
        self.last_linear = None
        self.last_angular = None

    def __del__(self):
        if self.debug:
            print("Closing serial port")
        if self.ser.is_open:
            self.ser.close()

    def command_callback(self, request, response):
        if self.debug:
            self.get_logger().info("Command received: %0.2f, %0.2f" %(request.linear, request.angular))

        if request.linear != self.last_linear or request.angular != self.last_angular:
            ## Send Velocity Command
            command = f"mn {request.linear},{request.angular}"
            self.ser.write(command.encode())
            if self.debug:
                self.get_logger().info("Sending velocity command")
            
            while (self.ser.inWaiting() == 0):
                if self.debug:
                    self.get_logger().info("Waiting for velocity response")
                time.sleep(self.serial_timeout/4)
            if self.ser.inWaiting() > 0:
                _ = self.ser.read(self.ser.inWaiting())

            self.last_linear  = request.linear
            self.last_angular = request.angular

        ## Read response
        command = "qo"
        self.ser.write(command.encode())
        if self.debug:
            self.get_logger().info("Requesting sensor data")

        while (self.ser.inWaiting() == 0):
            if self.debug:
                self.get_logger().info("Waiting for sensors response")
            time.sleep(self.serial_timeout/4)

        if self.ser.inWaiting() > 0:
            ser_resp = self.ser.readline().decode().strip()
            if ser_resp.startswith("done qo"):
                response.success = True
                if self.debug:
                    self.get_logger().info('Response done qo recieved\n')
            else:
                if self.debug:
                    self.get_logger().warn('Response %s'%(ser_resp))
                response.success = False

        if response.success:
            ser_resp = ser_resp.split(" ")[2].split(",")
            response.quaternion = [float(f)/1000 for f in ser_resp[:4]]
            response.encoders = [int(d) for d in ser_resp[4:]]
        else:
            self.get_logger().warn("Failed to get sensor data\n")

        return response


def main():
    rclpy.init()

    server = SerialServer()

    rclpy.spin(server)
    rclpy.shutdown()


if __name__ == '__main__':
    main()