import numpy as np
import pandas as pd
from scipy.spatial.transform import Rotation as R
import time
import rclpy
from nav_msgs.msg import Odometry
import sys


class AC_out:
    def __init__(self, loop=False):
        self.node = rclpy.create_node('sim_sim')
        self.publisher = self.node.create_publisher(Odometry, 'odometry', 10)
        self.loop = loop
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
            "worldMatrix_M00",
            "worldMatrix_M10",
            "worldMatrix_M20",
            "worldMatrix_M30",
            "worldMatrix_M01",
            "worldMatrix_M11",
            "worldMatrix_M21",
            "worldMatrix_M31",
            "worldMatrix_M02",
            "worldMatrix_M12",
            "worldMatrix_M22",
            "worldMatrix_M32",
            "worldMatrix_M03",
            "worldMatrix_M13",
            "worldMatrix_M23",
            "worldMatrix_M33"
        ]
        self.df = pd.read_csv("output_csv/vallelunga_lap.csv", names=header)
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
                if delay < 0:
                    delay = 0
                time.sleep(delay)

            self.previous_timestamp = current_timestamp

            odometry = Odometry()
            odometry.header.stamp = self.node.get_clock().now().to_msg()
            odometry.header.frame_id = "base_link"

            odometry.pose.pose.position.x = -row['worldMatrix_M03']
            odometry.pose.pose.position.y = row['worldMatrix_M23']
            odometry.pose.pose.position.z = row['worldMatrix_M13']

            r = R.from_matrix([[row['worldMatrix_M00'], row['worldMatrix_M01'], row['worldMatrix_M02']],
                               [row['worldMatrix_M10'], row['worldMatrix_M11'], row['worldMatrix_M12']],
                               [row['worldMatrix_M20'], row['worldMatrix_M21'], row['worldMatrix_M22']]])
            angles = r.as_euler('zxy')
            odometry.pose.pose.orientation.x = -angles[0]
            odometry.pose.pose.orientation.y = angles[1]
            odometry.pose.pose.orientation.z = angles[2]

            odometry.twist.twist.linear.x = row['velocity_x']
            odometry.twist.twist.linear.y = row['velocity_y']
            odometry.twist.twist.linear.z = row['velocity_z']
            odometry.twist.twist.angular.x = row['angularVelocity_x']
            odometry.twist.twist.angular.y = row['angularVelocity_y']
            odometry.twist.twist.angular.z = row['angularVelocity_z']

            self.publisher.publish(odometry)

        except StopIteration:
            if self.loop:
                print("End of CSV file reached. Restarting.")
                self.df_iter = self.df.iterrows()
            else:
                print("End of CSV file reached. Exiting.")
                rclpy.shutdown()



def main(args=None):
    loop = '-l' in sys.argv

    rclpy.init(args=args)
    output = AC_out(loop=loop)
    try:
        rclpy.spin(output.node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            output.node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
