#!/usr/bin/env python3
"""One-shot diagnostic: log /sdf_map/occupancy_inflate and /sdf_map/unknown
point-cloud widths alongside /ground_truth/odom position, to a CSV, for the
duration it's left running. Not part of the deployed system."""
import sys
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry


class MapMonitor(Node):
    def __init__(self, out_path):
        super().__init__('map_monitor')
        self.f = open(out_path, 'w')
        self.f.write('t_wall,topic,width,odom_x,odom_y,odom_z\n')
        self.last_pos = (float('nan'),) * 3
        self.create_subscription(Odometry, '/ground_truth/odom', self.odom_cb, 10)
        self.create_subscription(PointCloud2, '/sdf_map/occupancy_inflate', self.occ_cb, 10)
        self.create_subscription(PointCloud2, '/sdf_map/unknown', self.unk_cb, 10)

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        self.last_pos = (p.x, p.y, p.z)

    def occ_cb(self, msg):
        self.f.write(f'{time.time():.3f},occupancy_inflate,{msg.width},{self.last_pos[0]},{self.last_pos[1]},{self.last_pos[2]}\n')
        self.f.flush()

    def unk_cb(self, msg):
        self.f.write(f'{time.time():.3f},unknown,{msg.width},{self.last_pos[0]},{self.last_pos[1]},{self.last_pos[2]}\n')
        self.f.flush()


def main():
    rclpy.init()
    node = MapMonitor(sys.argv[1])
    rclpy.spin(node)


if __name__ == '__main__':
    main()
