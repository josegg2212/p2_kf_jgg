import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped, Twist, PoseWithCovarianceStamped
from irobot_create_msgs.msg import WheelVels

import numpy as np
import math

from .sensor_utils import odom_to_pose2D, get_normalized_pose2D, Odom2DDriftSimulator, generate_noisy_measurement_2
from .visualization import Visualizer
from .filters.kalman_filter import KalmanFilter 
from .motion_models import velocity_motion_model

class KalmanFilterNode(Node):
    def __init__(self):
        super().__init__('kalman_filter_node')

        # TODO: Initialize filter with initial state and covariance
        self.initial_state = np.zeros(3)
        initial_covariance = np.eye(3) * 0.1

        self.prev_time = None
        
        self.kf = KalmanFilter(self.initial_state, initial_covariance)
        
        self.visualizer = Visualizer()
        
        
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/kf_estimate',
            10
        )

    def odom_callback(self, msg):
        # TODO: Extract velocities and timestep   
        current_pose = odom_to_pose2D(msg)  
        self.normalized_pose = np.array(get_normalized_pose2D(self.initial_state , current_pose))   
        
        current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        if self.prev_time is None :
           self.prev_time = current_time
           return
        dt = current_time - self.prev_time
        self.prev_time = current_time
            
        v = msg.twist.twist.linear
        w = msg.twist.twist.angular
            
        z = generate_noisy_measurement_2(self.normalized_pose , v.x , v.y , w.z)
        
        # TODO: Run predict() and update() of KalmanFilter
        self.u = np.array([np.sqrt(v.x**2 + v.y**2) , w.z])
        
        self.kf.predict(self.u , dt)
        
        (mu , sigma) = self.kf.update(z[:3])
        
        self.get_logger().info(f"KF state: {mu}")
        self.get_logger().info(f"Noisy measurement: {z}")

        self.visualizer.update(self.normalized_pose , mu , sigma , step="update")

        # TODO: Publish estimated state
        
        pose = PoseWithCovarianceStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        pose.pose.pose.position.x = mu[0]
        pose.pose.pose.position.y = mu[1]
        pose.pose.pose.orientation.z = math.sin(mu[2] / 2.0)
        pose.pose.pose.orientation.w = math.cos(mu[2] / 2.0)
        cov = np.zeros((6, 6))
        cov[:3, :3] = sigma
        pose.pose.covariance = cov.flatten().tolist()

        self.publisher.publish(pose)
        

def main(args=None):
    rclpy.init(args=args)
    node = KalmanFilterNode()
    rclpy.spin(node)
    rclpy.shutdown()
