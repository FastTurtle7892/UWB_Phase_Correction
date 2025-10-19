import random
import numpy as np
import math
from filterpy.kalman import KalmanFilter
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from settings import ANCHORS

print(len(ANCHORS))

# Initialize Kalman Filter for 3D
kf = KalmanFilter(dim_x=6, dim_z=3)
kf.x = np.array([0, 0, 0, 0, 0, 0])  # Initial state [x, y, z, vx, vy, vz]
kf.F = np.array([[1, 0, 0, 1, 0, 0],  # State transition matrix
                 [0, 1, 0, 0, 1, 0],
                 [0, 0, 1, 0, 0, 1],
                 [0, 0, 0, 1, 0, 0],
                 [0, 0, 0, 0, 1, 0],
                 [0, 0, 0, 0, 0, 1]])
kf.H = np.array([[1, 0, 0, 0, 0, 0],  # Measurement function
                 [0, 1, 0, 0, 0, 0],
                 [0, 0, 1, 0, 0, 0]])
kf.P *= 1000.0  # Covariance matrix
kf.R = np.eye(3) * 1  # Measurement noise
kf.Q = np.eye(6) * 0.1  # Process noise

class Distance2Coord(Node):
    def __init__(self):
        super().__init__('distance_coord')
        self.distance_sub = self.create_subscription(Float32MultiArray, 'dist_data', self.subscribe_dist_data, 10)
        self.distance_sub
        self.filtered_position = None

    def subscribe_dist_data(self, msg):
        self.get_logger().info(f'Data: {msg.data}')
        position = self.calculate_position(msg)
        
        if position:
            self.filtered_position = self.tuned_position(position)
            #self.filtered_position = position
            self.get_logger().info(f'Filtered Data: {self.filtered_position}')

    
    def calculate_position(self, msg):
        x1, y1 = ANCHORS[0]
        A = []
        B = []

        for i in range(1, len(ANCHORS)):
            x2, y2 = ANCHORS[i]
            d1 = msg.data[0]
            d2 = msg.data[i]

            A.append([2 * (x2 - x1), 2 * (y2 - y1)])
            B.append(d1**2 - d2**2 - x1**2 + x2**2 - y1**2 + y2**2)

        A = np.array(A)
        B = np.array(B).reshape(-1, 1)

        position = np.linalg.lstsq(A, B, rcond=None)[0]
        x = position[0, 0]
        y = position[1, 0]
        z = 0

        for i in range(0, len(ANCHORS)):
            x_i, y_i = ANCHORS[i]
            z = z + math.sqrt(max(0, msg.data[i]**2 - (x-x_i)**2))
    
        z = z / len(ANCHORS)

   
        return x, y, z

    def tuned_position(self, position):
        kf.predict()
        kf.update(np.array(position))
        
        return kf.x[:3]

