import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO

class HumanDistanceNode(Node):
    def __init__(self):
        super().__init__('human_distance_node')

        self.bridge = CvBridge()
        self.model = YOLO("yolov8n.pt")

        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        self.publisher = self.create_publisher(
            Float32,
            '/human_distance',
            10
        )

        self.REAL_HEIGHT = 170  # cm

        # ✅ PUT YOUR CALCULATED VALUE HERE
        self.FOCAL_LENGTH = 100  

        self.get_logger().info("Human Distance Node Started")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        results = self.model(frame)

        for r in results:
            for box in r.boxes:
                cls = int(box.cls[0])
                conf = float(box.conf[0])

                if cls == 0 and conf > 0.6:

                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    height_pixels = y2 - y1

                    if height_pixels < 100:
                        continue

                    distance_cm = (self.REAL_HEIGHT * self.FOCAL_LENGTH) / height_pixels

                    msg_out = Float32()
                    msg_out.data = float(distance_cm)
                    self.publisher.publish(msg_out)

                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 2)

                    cv2.putText(
                        frame,
                        f"{distance_cm:.1f} cm",
                        (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0,255,0),
                        
                    )

        cv2.imshow("Human Detection", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = HumanDistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()