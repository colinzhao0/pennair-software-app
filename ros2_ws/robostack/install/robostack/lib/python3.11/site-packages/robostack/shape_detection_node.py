import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge
import numpy as np

class ShapeDetectionNode(Node):
	def __init__(self):
		super().__init__('shape_detection_node')
		self.subscription = self.create_subscription(
			Image,
			'camera/image_raw',
			self.listener_callback,
			10)
		self.publisher_ = self.create_publisher(String, 'shapes/detections', 10)
		self.bridge = CvBridge()
		self.get_logger().info('ShapeDetectionNode started. Waiting for images...')

	def listener_callback(self, msg):
		frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		img_copy = frame.copy()
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		blurred = cv2.GaussianBlur(gray, (5, 5), 0)

		#get edges
		edges = cv2.adaptiveThreshold(blurred, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2)

		#dilate edges to connect broken lines
		kernel = np.ones((3, 3), np.uint8)
		dilated_edges = cv2.dilate(edges, kernel, iterations=2)
		inverted = cv2.bitwise_not(dilated_edges)

		#morphology to clean noise
		kernel = np.ones((3, 3), np.uint8)
		cleaned = cv2.morphologyEx(inverted, cv2.MORPH_CLOSE, kernel, iterations=2)
		cleaned = cv2.morphologyEx(cleaned, cv2.MORPH_OPEN, kernel, iterations=2)

		contours, _ = cv2.findContours(cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

		#Filter out small shapes caused by remaining noise
		large_contours = []
		for cnt in contours:
			perimeter = cv2.arcLength(cnt, True)
			area = cv2.contourArea(cnt)
			if area > 5000 and (perimeter / area) < 0.15:
				large_contours.append(cnt)
		cv2.drawContours(frame, large_contours, -1, (255, 255, 255), 2)

		#setting up camera matrix
		K = np.array([[2564.3186869, 0, 0], [0, 2569.70273111, 0], [0, 0, 1]])
		fx = K[0, 0]
		fy = K[1, 1]
		cx = K[0, 2]
		cy = K[1, 2]
		real_radius = 10.0  #note that this is in inches

		#find the circle and calibrate Z
		calibrated_Z = None
		for cnt in large_contours:
			(x, y), pixel_radius = cv2.minEnclosingCircle(cnt)
			pixel_radius = float(pixel_radius)
			circularity = cv2.arcLength(cnt, True) / (2 * np.pi * pixel_radius) if pixel_radius > 0 else 0
			if 0.95 < circularity < 1.05 and pixel_radius > 0:
				calibrated_Z = fx * real_radius / pixel_radius
				break

		#apply calibrated Z to all shapes
		detections = []
		for cnt in large_contours:
			M = cv2.moments(cnt)
			if M["m00"] != 0:
				cX = int(M["m10"] / M["m00"])
				cY = int(M["m01"] / M["m00"])
				if calibrated_Z is not None:
					X = (cX - cx) * calibrated_Z / fx
					Y = (cY - cy) * calibrated_Z / fy
					detections.append(f"3D Center (X={X:.1f}, Y={Y:.1f}, Z={calibrated_Z:.1f})")
					cv2.circle(frame, (cX, cY), 3, (255, 255, 255), -1)
					cv2.putText(frame, f"3D Center (X={X:.1f}, Y={Y:.1f}, Z={calibrated_Z:.1f})", (cX + 10, cY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
				else:
					detections.append(f"Center ({cX}, {cY})")
					cv2.circle(frame, (cX, cY), 3, (255, 255, 255), -1)
					cv2.putText(frame, f"Center ({cX}, {cY})", (cX + 10, cY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
			else:
				cX, cY = 0, 0

		# Publish detections as a string (for demo)
		msg_out = String()
		msg_out.data = " | ".join(detections)
		self.publisher_.publish(msg_out)
		self.get_logger().info(f'Published detections: {msg_out.data}')

def main():
	print('Starting ShapeDetectionNode...')
	rclpy.init()
	node = ShapeDetectionNode()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()
