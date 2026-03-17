import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSDurabilityPolicy

from sensor_msgs.msg import Image
from sensor_msgs.msg import NavSatFix

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from cv_bridge import CvBridge
import cv2
import numpy as np

from typing import List
from functools import partial
import time

class BagContentTopic:
    def __init__(self, topic, qos, expectedProcessedImgs = 1):
        self.topic = topic
        self.qos = qos
        self.expectedProcessedImgs = expectedProcessedImgs
    
    def setFPS(self, fps):
        self.fps = fps

    def fromMetadata(self, metadata, bagDuration = -1):
        if not self.isMe(metadata.topic_metadata.name):
            raise RuntimeError(f"Not me: {metadata}")
        
        self.msgCount = metadata.message_count
        self.type = metadata.topic_metadata.type
        self.serialization = metadata.topic_metadata.serialization_format

        if bagDuration > 0.0 and self.msgCount > 0:
            self.fps = 1.0 / ( bagDuration / self.msgCount)

    def isMe(self, name):
        return name == self.topic
    
    def getType(self):
        return get_message(self.type)
    
    def setPublisher(self, pub):
        self.pub = pub

    def getPubBuildArgs(self):
        return (self.getType(), self.topic, self.qos)
    
    def publish(self, msg):
        self.pub.publish(msg)
        return self.expectedProcessedImgs
    
    def __eq__(self, name):
        return name == self.topic
    
    def __str__(self):
        return f"{self.topic} ({self.type} | {self.serialization}): {self.msgCount} msgs (at {self.fps:.2f} FPS)"

class VideoRecorder:
    def __init__(self, folder, video_name, topic_name, expected_fps):
        self.folder = folder
        self.video_name = video_name
        self.expected_fps = expected_fps
        self.topic_name = topic_name
        self.video = None

    def writeFrame(self, frame:cv2.Mat):
        if self.video is None:
            fourcc = cv2.VideoWriter.fourcc(*'mp4v')
            self.video = cv2.VideoWriter(
                f"{self.folder}/{self.video_name}.mp4",
                fourcc,
                self.expected_fps,
                frame.shape[:2]
            )
        self.video.write(frame)

    def cleanup(self):
        if self.video is not None:
            self.video.release()
            print(f"Video {self.video_name}.mp4 saved")
    

class Bage2Video(Node):
    def __init__(self):
        super().__init__('bag_to_video')
        self.done = False
        self.requestTermination = False
        
        pathToBag = "/Volumes/CAILLOT/Datasets/CAILLOT/RAP/record_2026_03_11/rosbag2_2026_03_11-16_37_57"

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.VOLATILE
        )

        self.topicsInBag = dict()
        self.topicsInBag['/ids_camera/image_raw'] = BagContentTopic('/ids_camera/image_raw', qos, 3) # hdr, short and long -> 3 images expected
        self.topicsInBag['/yap_flir4ros/image_raw'] = BagContentTopic('/yap_flir4ros/image_raw', qos, 1)
        self.topicsInBag['/fix'] = BagContentTopic('/fix', qos, 0)
        self.remainingToRead = 0

        self.img_in_queue = 0
        self.bridge = CvBridge()
        self.video_folder = '/Users/caillotantoine/Desktop/video_out'

        with open(f"{self.video_folder}/gps_log.txt", "w") as f:
            f.write(f"timestamp, latitude, longitude, altitude\n")
        
        

        storage_options = rosbag2_py.StorageOptions(
            uri=pathToBag,
            storage_id='')
        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr'
        )

        info = rosbag2_py.Info()
        metadata = info.read_metadata(storage_options.uri, storage_options.storage_id)

        self.get_logger().info(f"Bag path: {metadata.relative_file_paths}")
        duration = metadata.duration.nanoseconds / 1e9
        self.get_logger().info(f"Duration (s): {duration}")

        for topic in metadata.topics_with_message_count:
            if topic.topic_metadata.name in self.topicsInBag:
                tInBag = self.topicsInBag[topic.topic_metadata.name]
                tInBag.fromMetadata(topic, duration)
                tInBag.setPublisher(self.create_publisher(*tInBag.getPubBuildArgs()))
                self.get_logger().info(f"{tInBag}")

        for tInBag in self.topicsInBag.values():
            self.remainingToRead += tInBag.msgCount

        self.get_logger().info(f"Messages to read: {self.remainingToRead}")
        self.msgRead = 0.0
        self.lastPercent = -1


        self.videoRecorders = dict()
        self.videoRecorders['/contrastor/output'] = VideoRecorder(self.video_folder, "ae_contrasted", '/contrastor/output', self.topicsInBag['/yap_flir4ros/image_raw'].fps)
        self.videoRecorders['/camera/HDR/image_raw'] = VideoRecorder(self.video_folder, "hdr", '/camera/HDR/image_raw', self.topicsInBag['/ids_camera/image_raw'].fps)
        self.videoRecorders['/camera/LDR/short/image_raw'] = VideoRecorder(self.video_folder, "short", '/camera/LDR/short/image_raw', self.topicsInBag['/ids_camera/image_raw'].fps)
        self.videoRecorders['/camera/LDR/long/image_raw'] = VideoRecorder(self.video_folder, "long", '/camera/LDR/long/image_raw', self.topicsInBag['/ids_camera/image_raw'].fps)

        self.sub1 = self.create_subscription(
            Image,
            '/contrastor/output',
            partial(self.receive_and_save_image, topic_name='/contrastor/output'),
            qos
        )
        self.sub2 = self.create_subscription(
            Image,
            '/camera/HDR/image_raw',
            partial(self.receive_and_save_image, topic_name='/camera/HDR/image_raw'),
            qos
        )
        self.sub3 = self.create_subscription(
            Image,
            '/camera/LDR/short/image_raw',
            partial(self.receive_and_save_image, topic_name='/camera/LDR/short/image_raw'),
            qos
        )
        self.sub4 = self.create_subscription(
            Image,
            '/camera/LDR/long/image_raw',
            partial(self.receive_and_save_image, topic_name='/camera/LDR/long/image_raw'),
            qos
        )


        self.reader = rosbag2_py.SequentialReader()
        self.reader.open(storage_options, converter_options)

        self.get_logger().info("Start reading")
        self.once = True
        self.publishNimages = 10
        self.receiveNimages = 10
        self.showNtimes = 10

        self.read_next_and_publish()
        # self.terminate()

    def terminate(self):
        for recorder in self.videoRecorders.values():
            recorder.cleanup()
        self.done = True


    def receive_and_save_image(self, msg:Image, topic_name):
        self.img_in_queue -= 1
        frame = self.bridge.imgmsg_to_cv2(msg, msg.encoding)

        
        if self.receiveNimages > 0:
            self.get_logger().info(f"Received {topic_name} : imgQueue {self.img_in_queue}")
            if self.receiveNimages == 1:
                self.get_logger().info(f"Stop logging, but continue running...")
            self.receiveNimages -= 1

        self.videoRecorders[topic_name].writeFrame(frame)
        if self.img_in_queue == 0:
            if self.requestTermination:
                self.terminate()
                return
            self.read_next_and_publish()
    

    def read_next_and_publish(self):
        
        valid = False
        while not valid or self.publishNimages > 0:
            if not self.reader.has_next():
                self.get_logger().info("Finished! Request termination.")
                if self.img_in_queue == 0:
                    self.terminate()
                else:
                    self.requestTermination = True
                return
        
            topic, data, timestamp = self.reader.read_next()

            if topic in self.topicsInBag:
                valid = True

                

                tInBag = self.topicsInBag[topic]
                msg = deserialize_message(data, tInBag.getType())
                if topic == '/fix':
                    line = f"{timestamp},{msg.latitude},{msg.longitude},{msg.altitude}\n"
                    with open(f"{self.video_folder}/gps_log.txt", "a") as f:
                        f.write(line)
                    self.read_next_and_publish()
                    return
                self.img_in_queue += tInBag.publish(msg)
                self.publishNimages -= 1
                if self.publishNimages > 0:
                    self.img_in_queue = 0
                    self.get_logger().info(f"{self.publishNimages} : {topic}")
                    time.sleep(0.2)

                self.msgRead += 1.0
                currentPercent = int(self.msgRead / self.remainingToRead * 100.0)
                if currentPercent % 3 == 0 and currentPercent > self.lastPercent:
                    self.lastPercent = currentPercent
                    self.get_logger().info(f"Processed {currentPercent}%")

            else:
                valid = False

            if self.showNtimes > 0:
                self.get_logger().info(f"Read {topic} as first")
                if valid:
                    self.showNtimes -= 1


            # self.get_logger().info(f"Published: {topic}, Time: {t}")


        # self.get_logger().info(f"Topic: {topic}, Time: {t}")

def main():
    rclpy.init()
    node = Bage2Video()

    while rclpy.ok() and not node.done:
        try:
            rclpy.spin_once(node)    
        except KeyboardInterrupt:
            node.get_logger().warn("Request termination")
            node.requestTermination = True

    node.destroy_node()
    rclpy.shutdown()