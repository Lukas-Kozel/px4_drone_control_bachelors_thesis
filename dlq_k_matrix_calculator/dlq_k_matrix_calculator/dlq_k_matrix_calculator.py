import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import numpy as np
import math
import control

class MyNode(Node):

    def __init__(self):
        super().__init__('dlq_k_matrix_calculator')
        self.publisher_ = self.create_publisher(Float64MultiArray, '/dlq_k_matrix', 10)
        timer_period = 0.05 
        self.timer = self.create_timer(timer_period, self.timer_callback)
        x1max = 4
        x2max = 2.0
        x3max = math.pi/16
        x4max = 1.5
        umax = 2.3
        self.matrix_Q = np.array([
            [1/pow(x1max,2), 0,0, 0],
            [0, 1/pow(x2max,2), 0, 0],
            [0, 0 ,1/pow(x3max,2), 0],
            [0, 0, 0, 1/pow(x4max,2)],            
            ])
        self.matrix_R = np.array([1/pow(umax,2)])
        self.matrix_B = np.array([
            [0.0001],
            [0.01],
            [0.0003],
            [0.0285]
        ])
        self.matrix_A = np.array([
            [1, 0.02, -0.0002, 0],
            [0, 1, -0.0245, -0.0002],
            [0, 0, 0.9937, 0.02],
            [0, 0, -0.6293, 0.9937]
        ])

    def timer_callback(self):
        K = self.calculate_dlqr()
        msg = Float64MultiArray()
        msg.data = K.flatten().tolist()
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing K matrix: {msg.data}')


    def calculate_dlqr(self):
        K,_,_ = control.dlqr(self.matrix_A,self.matrix_B,self.matrix_Q,self.matrix_R)
        return K

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
