import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
import numpy as np

import os
import csv


QUALITY = ["0 - Fix not valid", "1 - GPS Fix", "2 - DGPS Fix", "3 - N/A", "4 - RTK Fix", "5 - RTK Float", "6 - INS Dead reckoning", "7 - Manual Input mode", "8 - Simulation mode"]



class Fix2CSV(Node):

    def __init__(self):
        super().__init__('fix_to_csv')
        self.sub = self.create_subscription(NavSatFix, 'fix', self.CallbackNavSatFix, 10)
        # Path to CSV file (you can set this as a class attribute or pass as parameter)
        self.csv_path = '/Users/caillotantoine/Desktop/gps.csv'  # Replace with your desired path

        # Check if file exists to determine if header should be written
        with open(self.csv_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['timestamp', 'alt', 'lat', 'lon', 'r95conf', 'r68conf'])

    def CallbackNavSatFix(self, msg:NavSatFix):

        timestamp = msg.header.stamp
        alt = msg.altitude
        lat = msg.latitude
        lon = msg.longitude

        status = msg.status.status
        covMat:np.ndarray = msg.position_covariance

        sigma_h = np.sqrt(covMat[0])
        r95conf = np.sqrt(5.99) * sigma_h
        r68conf = np.sqrt(3.53) * sigma_h
        self.get_logger().info(
            f"timestamp: {timestamp.sec}.{timestamp.nanosec:09d}, alt: {alt}, lat: {lat}, lon: {lon}, r95conf: {r95conf}, r68conf: {r68conf}"
        )

        # Prepare row data
        row = [
            f"{timestamp.sec}.{timestamp.nanosec:09d}",
            alt,
            lat,
            lon,
            r95conf,
            r68conf
        ]

        # Write to CSV file (overwrite if not exists, append otherwise)
        with open(self.csv_path, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(row)



def main():
    rclpy.init()
    node = Fix2CSV()
    rclpy.spin(node)
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
        


