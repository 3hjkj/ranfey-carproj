import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
import time
from ultralytics import YOLO


class YOLONode(Node):
    def __init__(self):
        super().__init__("yolo_async_node")
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image, "camera/image", self.image_callback, 10
        )

        self.model = YOLO(
            "/home/nvidia/zhitai/qingling_ros2/src/yolo/src/yolo11n.pt"
        ).to("cuda")

        # 缓存图像和框数据
        self.lock = threading.Lock()
        self.latest_frame = None
        self.latest_boxes = []  # 上次推理的框

        # 启动检测线程
        self.det_thread = threading.Thread(target=self.yolo_worker, daemon=True)
        self.det_thread.start()

        # 启动显示线程
        self.running = True
        self.display_thread = threading.Thread(target=self.display_loop)
        self.display_thread.start()

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"图像转换失败: {e}")
            return
        with self.lock:
            self.latest_frame = frame.copy()  # 更新图像缓存

    def yolo_worker(self):
        while rclpy.ok():
            # time.sleep(0.05)  # 避免占满CPU
            with self.lock:
                if self.latest_frame is None:
                    continue
                frame = self.latest_frame.copy()
            results = self.model.predict(source=frame, verbose=False)[0]
            boxes = []
            for box in results.boxes:
                x1, y1, x2, y2 = box.xyxy[0].int().tolist()
                cls_id = int(box.cls[0].item())
                score = float(box.conf[0].item())
                boxes.append((x1, y1, x2, y2, cls_id, score))

            with self.lock:
                self.latest_boxes = boxes  # 更新框坐标缓存

    def display_loop(self):
        names = self.model.names
        while self.running:
            with self.lock:
                frame = (
                    self.latest_frame.copy() if self.latest_frame is not None else None
                )
                boxes = self.latest_boxes.copy()

            if frame is not None:
                for x1, y1, x2, y2, cls_id, score in boxes:
                    label = f"{names[cls_id]} {score:.2f}"
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(
                        frame,
                        label,
                        (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 255, 255),
                        1,
                        cv2.LINE_AA,
                    )

                cv2.imshow("YOLO Async Display", cv2.resize(frame, (1280, 720)))
                if cv2.waitKey(1) == 27:
                    self.running = False
                    break
            else:
                time.sleep(0.01)

        cv2.destroyAllWindows()

    def stop(self):
        self.running = False
        self.display_thread.join()
        self.det_thread.join()


def main(args=None):
    rclpy.init(args=args)
    node = YOLONode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
