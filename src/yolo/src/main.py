import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO


import sys

print(sys.executable)


class YOLONode(Node):
    def __init__(self):
        super().__init__("yolo_node")
        self.subscription = self.create_subscription(
            Image, "camera/image", self.listener_callback, 10
        )
        self.bridge = CvBridge()
        self.model = YOLO("/home/nvidia/zhitai/qingling_ros2/src/yolo/src/yolo11x.pt")
        self.model.to("cuda")

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model.predict(source=cv_image, verbose=False)[0]
        for box in results.boxes.xyxy.cpu().numpy():
            x1, y1, x2, y2 = box[:4].astype(int)
            cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

        cv_image_resized = cv2.resize(cv_image, (1280, 720))
        cv2.imshow("YOLO", cv_image_resized)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = YOLONode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
