import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import threading
import time

# UWB modules port
com_ports = ['/dev/ttyACM0', '/dev/ttyACM2', '/dev/ttyACM4', '/dev/ttyACM6']
baud_rate = 1000000  

serial_objs = []

for port in com_ports:
    ser = serial.Serial(port, baud_rate, timeout=1)
    serial_objs.append(ser)


class SerialDataPublisher(Node):
    def __init__(self):
        super().__init__('serial_data_publisher')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'dist_data', 10)
        self.distances = [0.0] * len(com_ports)  
        self.lock = threading.Lock()  

        for idx, serial_obj in enumerate(serial_objs):
            thread = threading.Thread(target=self.read_from_port, args=(serial_obj, idx))
            thread.daemon = True  
            thread.start()
            
	# Publish data using timer
        self.timer = self.create_timer(0.01, self.publish_distances)

    def read_from_port(self, serial_obj, idx):
        while True:
            if serial_obj.in_waiting > 0:  
                data_line = serial_obj.readline().decode('utf-8').strip()
                try:
                    distance = float(data_line.split('|')[1])  
                    with self.lock: 
                        self.distances[idx] = distance
                except ValueError:
                    self.get_logger().warn(f"Send Wrong Data: {data_line}")
            time.sleep(0.01)  

    def publish_distances(self):
        msg = Float32MultiArray()
        with self.lock: 
            msg.data = self.distances.copy()
        self.publisher_.publish(msg)
        self.get_logger().info(f"Distance data: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    serial_data_publisher = SerialDataPublisher()

    try:
        rclpy.spin(serial_data_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        for serial_obj in serial_objs:
            serial_obj.close()
        serial_data_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

