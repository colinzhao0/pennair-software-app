import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

class VideoStreamer(Node):
	def __init__(self):
		super().__init__('video_streamer')
		self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
		self.timer = self.create_timer(0.033, self.timer_callback)  # ~30fps
		self.cap = cv2.VideoCapture('resources/PennAir 2024 App Dynamic Hard.mp4')
		self.bridge = CvBridge()
		self.get_logger().info('VideoStreamer node started. Streaming video...')

	def timer_callback(self):
		ret, frame = self.cap.read()
		if not ret:
			self.get_logger().info('End of video stream.')
			self.cap.release()
			rclpy.shutdown()
			return
		msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
		self.publisher_.publish(msg)
		self.get_logger().info('Published a video frame.')

def main():
	print('Starting VideoStreamer node...')
	rclpy.init()
	node = VideoStreamer()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
