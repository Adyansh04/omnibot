#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from time import time

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

     # print(len(range))
        
        # front_dist_lidar = ranges[180]
        # right_dist_lidar = ranges[90]
        # left_dist_lidar = ranges[270]
        # back_dist_lidar = ranges[359]
        
        # print(ranges[90]) #Right 
        # print(ranges[180]) #Front
        # print(ranges[270]) #Left
        # print(ranges[359]) #Back
class LidarToOdom(Node):
    def __init__(self):
        super().__init__('lidar_to_odom_node')
        self.get_logger().info("Lidar to Odom node has been started")

        self.create_subscription(
            LaserScan, 
            '/scan', 
            self.lidar_callback, 
            10
        )
        
        self.odom_publisher = self.create_publisher(
            Odometry, 
            '/odom',
            10)
        
        self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
        )
        
        self.prev_front_dist = None
        self.prev_back_dist = None
        self.prev_left_dist = None
        self.prev_right_dist = None

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        
        self.last_x_update_time = time()
        self.last_y_update_time = time()
        
        self.x_vel = 0.0
        self.y_vel = 0.0

        self.threshold = 0.5  # Threshold to ignore sudden large changes
        
        #quaternion variables
        self.x_quat = 0.0
        self.y_quat = 0.0
        self.z_quat = 0.0
        self.w_quat = 0.0

        
    def lidar_callback(self, msg):
            self.get_logger().info("Lidar data received")
            
            
            current_time = self.get_clock().now().to_msg()
            ranges = msg.ranges
            # print(ranges[0])
            # print(ranges[90])
            # print(ranges[180])
            # print(ranges[270])
            # print(ranges[359])
            print(len(ranges))
            
            front_dist = ranges[180] if ranges[180] < float('inf') else None
            back_dist = ranges[359] if ranges[359] < float('inf') else None
            left_dist = ranges[270] if ranges[270] < float('inf') else None
            right_dist = ranges[90] if ranges[90] < float('inf') else None
        
            # Update x position
            if self.prev_front_dist is not None and self.prev_back_dist is not None:
                if front_dist is not None and abs(front_dist - self.prev_front_dist) < self.threshold:
                    delta_x = self.prev_front_dist - front_dist
                elif back_dist is not None and abs(back_dist - self.prev_back_dist) < self.threshold:
                    delta_x = back_dist - self.prev_back_dist
                else:
                    delta_x = 0.0
                
                self.x += delta_x
                
                current_time_x = time()
                delta_time_x = current_time_x - self.last_x_update_time
                self.x_vel = delta_x / delta_time_x if delta_time_x > 0 else 0.0
                self.x += delta_x
                self.last_x_update_time = current_time_x
        
                
            # Update y position
            if self.prev_left_dist is not None and self.prev_right_dist is not None:
                if left_dist is not None and abs(left_dist - self.prev_left_dist) < self.threshold:
                    delta_y = self.prev_left_dist - left_dist
                elif right_dist is not None and abs(right_dist - self.prev_right_dist) < self.threshold:
                    delta_y = right_dist - self.prev_right_dist
                else:
                    delta_y = 0.0
                
                self.y += delta_y
                
                current_time_y = time()
                delta_time_y = current_time_y - self.last_y_update_time
                self.y_vel = delta_y / delta_time_y if delta_time_y > 0 else 0.0
                self.y += delta_y
                self.last_y_update_time = current_time_y

            self.prev_front_dist = front_dist if front_dist is not None else self.prev_front_dist
            self.prev_back_dist = back_dist if back_dist is not None else self.prev_back_dist
            self.prev_left_dist = left_dist if left_dist is not None else self.prev_left_dist
            self.prev_right_dist = right_dist if right_dist is not None else self.prev_right_dist

            odom_msg = Odometry()
            odom_msg.header.frame_id = "odom"
            odom_msg.header.stamp = current_time
            odom_msg.child_frame_id = "base_footprint"

            odom_msg.pose.pose.position.x = self.x
            odom_msg.pose.pose.position.y = self.y
            odom_msg.pose.pose.position.z = self.z
            
            odom_msg.twist.twist.linear.x = self.x_vel
            odom_msg.twist.twist.linear.y = self.y_vel
            odom_msg.twist.twist.linear.z = 0.0
            
            odom_msg.pose.pose.orientation.x = self.x_quat
            odom_msg.pose.pose.orientation.y = self.y_quat
            odom_msg.pose.pose.orientation.z = self.z_quat
            odom_msg.pose.pose.orientation.w = self.w_quat

            self.odom_publisher.publish(odom_msg)
   
            
    def imu_callback(self, msg):
        self.get_logger().info("IMU data received")
        self.x_quat = msg.orientation.x
        self.y_quat = msg.orientation.y
        self.z_quat = msg.orientation.z
        self.w_quat = msg.orientation.w
        # print(self.x_quat)
        # print(self.y_quat)
        # print(self.z_quat)
        # print(self.w_quat)
        
        
        
        # print(msg.orientation.x)
        # print(msg.orientation.y)
        # print(msg.orientation.z)
        # print(msg.orientation.w)
        
        # print(msg.angular_velocity.x)
        # print(msg.angular_velocity.y)
        # print(msg.angular_velocity.z)
        
        # print(msg.linear_acceleration.x)
        # print(msg.linear_acceleration.y)
        # print(msg.linear_acceleration.z)
        
        # print(msg.orientation_covariance)
        # print(msg.angular_velocity_covariance)
        # print(msg.linear_acceleration
        
        
def main(args=None):
    rclpy.init(args=args)

    lidar_to_odom_node = LidarToOdom()

    rclpy.spin(lidar_to_odom_node)

    rclpy.shutdown()

if __name__ == '__main__':
    main()




