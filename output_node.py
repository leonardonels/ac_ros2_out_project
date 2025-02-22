import numpy as np
import pandas as pd
import time
import rclpy
from nav_msgs.msg import Odometry
import os


class AC_out:
    def __init__(self):
        self.node = rclpy.create_node('pointcloud_filter')
        self.publisher = self.node.create_publisher(Odometry, 'odometry', 10)
        header = [
            "timestamp",
            "engineRPM",
            "limiterRPM",
            "steer",
            "gas",
            "brake",
            "clutch",
            "gear",
            "speed_value",
            "velocity_x",
            "velocity_y",
            "velocity_z",
            "localVelocity_x",
            "localVelocity_y",
            "localVelocity_z",
            "localAngularVelocity_x",
            "localAngularVelocity_y",
            "localAngularVelocity_z",
            "angularVelocity_x",
            "angularVelocity_y",
            "angularVelocity_z",
            "accG_x",
            "accG_y",
            "accG_z",
            "worldMatrix_M11",
            "worldMatrix_M21",
            "worldMatrix_M31",
            "worldMatrix_M41",
            "worldMatrix_M12",
            "worldMatrix_M22",
            "worldMatrix_M32",
            "worldMatrix_M42",
            "worldMatrix_M13",
            "worldMatrix_M23",
            "worldMatrix_M33",
            "worldMatrix_M43",
            "worldMatrix_M14",
            "worldMatrix_M24",
            "worldMatrix_M34",
            "worldMatrix_M44"
        ]
        self.df = pd.read_csv("output_csv/giro-vallelunga.csv", names=header)
        self.df_iter = self.df.iterrows()

        self.start_time = time.time()
        self.previous_timestamp = None
        
        self.node.create_timer(0.01, self.callback)

    def callback(self):
        try:
            index, row = next(self.df_iter)
            current_timestamp = row.iloc[0]
            
            if self.previous_timestamp is not None:
                delay = (current_timestamp - self.previous_timestamp) / 10**7
                print(delay)
                time.sleep(delay)
                
            self.previous_timestamp = current_timestamp


            msg = Odometry()
            self.publisher.publish(msg)

        except StopIteration:
            self.node.get_logger().info("Fine del file CSV")
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    output = AC_out()
    try:
        rclpy.spin(output.node)
    except KeyboardInterrupt:
        pass
    finally:
        output.node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()