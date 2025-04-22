# TO RUN THIS WITH YOLOv4-Tiny
# - Copy model files into RSPI local storage
# - Uncomment content in "Paths"
# - Uncomment content in "node = FaceRecognitionNode"

# TO RUN THIS WITH COMPRESSED IMAGES
# - Uncomment content in "Imports"
# - Uncomment content in "self.subscription"
# - Uncomment content in "image_callback"

# TO RUN THIS ON LAPTOP
# - Check you have "cv_bridge" in ROS + "pip install numpy opencv-python rclpy json"
# - Connect to the same network as RSPI
# - Download desired model + uncomment content in "Paths"
# - Download up-to-date embeddings + uncomment content in "node = FaceRecognitionNode"

# Imports

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# === RAW OPTION ===
from sensor_msgs.msg import Image
# === COMPRESSED OPTION ===
# from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import os

# Paths

# === YOLOv3 OPTION ===
# YOLO_CONFIG = "/home/gyattbot/FinalYearProject/src/face_recog/face_recog/yolov3-face.cfg"
# YOLO_WEIGHTS = "/home/gyattbot/FinalYearProject/src/face_recog/face_recog/yolov3-wider_16000.weights"
# FACE_EMBEDDING_MODEL = "/home/gyattbot/FinalYearProject/src/face_recog/face_recog/openface_nn4.small2.v1.t7"
# === YOLOv4-Tiny OPTION ===
YOLO_CONFIG = "/home/gyattbot/FinalYearProject/src/face_recog/face_recog/yolov4-tiny.cfg"
YOLO_WEIGHTS = "/home/gyattbot/FinalYearProject/src/face_recog/face_recog/yolov4-tiny.weights"
FACE_EMBEDDING_MODEL = "/home/gyattbot/FinalYearProject/src/face_recog/face_recog/openface_nn4.small2.v1.t7"
# === LAPTOP OPTION (choose YOLO model first and update pathname below) ===
# YOLO_CONFIG = "yolov3-face.cfg"
# YOLO_WEIGHTS = "yolov3-wider_16000.weights"
# FACE_EMBEDDING_MODEL = "openface_nn4.small2.v1.t7"

DETECT_CONFIDENCE = 0.5
RECOGNITION_THRESHOLD = 0.8  # Euclidean distance threshold for recognition

def load_yolo_detector():
    net = cv2.dnn.readNetFromDarknet(YOLO_CONFIG, YOLO_WEIGHTS)
    layer_names = net.getLayerNames()
    output_layers = [layer_names[i - 1] for i in net.getUnconnectedOutLayers().flatten()]
    return net, output_layers

def detect_face_yolo(image, net, output_layers, conf_threshold=DETECT_CONFIDENCE, nms_threshold=0.4):
    (H, W) = image.shape[:2]
    blob = cv2.dnn.blobFromImage(image, 1/255.0, (416, 416), swapRB=True, crop=False)
    net.setInput(blob)
    layer_outputs = net.forward(output_layers)

    boxes = []
    confidences = []
    for output in layer_outputs:
        for detection in output:
            confidence = detection[4]
            if confidence > conf_threshold:
                box = detection[0:4] * np.array([W, H, W, H])
                (centerX, centerY, width, height) = box.astype("int")
                x = int(centerX - (width / 2))
                y = int(centerY - (height / 2))
                boxes.append([x, y, int(width), int(height)])
                confidences.append(float(confidence))
    idxs = cv2.dnn.NMSBoxes(boxes, confidences, conf_threshold, nms_threshold)
    if len(idxs) > 0:
        best_box = None
        best_conf = 0.0
        for i in idxs.flatten():
            if confidences[i] > best_conf:
                best_conf = confidences[i]
                best_box = boxes[i]
        return best_box, best_conf
    else:
        return None, None

def undistort_image(cv2_frame):
    coefficients_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'calibration_data.npz')
    data = np.load(coefficients_file)
    camera_matrix = data['camera_matrix']
    distortion_coefficients = data['dist_coeffs']

    h, w = cv2_frame.shape[:2]
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_coefficients, (w, h), 1, (w, h))
    undistorted_image = cv2.undistort(cv2_frame, camera_matrix, distortion_coefficients, None, new_camera_matrix)

    x, y, w, h = roi
    undistorted_image = undistorted_image[y:y+h, x:x+w]
    return undistorted_image

class FaceRecognitionNode(Node):
    def __init__(self, embeddings_file):
        super().__init__('face_recognition_node')
        self.bridge = CvBridge()

        with open(embeddings_file, "r") as f:
            data = json.load(f)
        self.embeddings = {name: np.array(vec) for name, vec in data.items()}

        self.yolo_net, self.yolo_output_layers = load_yolo_detector()
        self.embedder = cv2.dnn.readNetFromTorch(FACE_EMBEDDING_MODEL)

        # === RAW OPTION ===
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        # === COMPRESSED OPTION ===
        # self.subscription = self.create_subscription(
        #     CompressedImage,
        #     '/camera/image_raw/compressed',
        #     self.image_callback,
        #     10
        # )

        self.name_publisher = self.create_publisher(String, 'recognized_person', 10)

        self._timer = self.create_timer(5, self._timer_callback)
        self.last_frame = None
        
    def _timer_callback(self):
        if (self.last_frame is None):
            self.get_logger().info(f"Failed to fetch the most recent frame")
            return
        
        embedding, bbox = self.extract_embedding(self.last_frame)
        if embedding is None:
            self.get_logger().info("No face detected")
            return

        min_distance = float("inf")
        recognized_name = "Unknown"
        for name, stored_embedding in self.embeddings.items():
            distance = np.linalg.norm(embedding - stored_embedding)
            if distance < min_distance:
                min_distance = distance
                recognized_name = name
        if min_distance < RECOGNITION_THRESHOLD:
            out_msg = String()
            out_msg.data = recognized_name
            self.name_publisher.publish(out_msg)
            self.get_logger().info(f"Recognized: {recognized_name} (distance: {min_distance:.3f})")
        else:
            self.get_logger().info(f"No match (minimum distance: {min_distance:.3f})")

    def extract_embedding(self, image):
        (H, W) = image.shape[:2]
        box, conf = detect_face_yolo(image, self.yolo_net, self.yolo_output_layers)
        if box is None:
            return None, None
        (x, y, w, h) = box
        (endX, endY) = (x + w, y + h)
        face = image[max(y, 0):min(endY, H), max(x, 0):min(endX, W)]
        if face.size == 0:
            return None, None
        faceBlob = cv2.dnn.blobFromImage(face, 1.0/255, (96, 96), (0, 0, 0), swapRB=True, crop=False)
        self.embedder.setInput(faceBlob)
        vec = self.embedder.forward().flatten()
        return vec, box

    def image_callback(self, msg):
        # === RAW OPTION ===
        self.last_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        # === COMPRESSED OPTION ===
        # np_arr = np.frombuffer(msg.data, np.uint8)
        # decompressed_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # self.last_frame = decompressed_image
        
def main(args=None):
    rclpy.init(args=args)
    # === YOLOv3 OPTION
    # node = FaceRecognitionNode("/home/gyattbot/FinalYearProject/src/face_recog/face_recog/embeddings.json")
    # === YOLOv4-Tiny OPTION
    node = FaceRecognitionNode("/home/gyattbot/FinalYearProject/src/face_recog/face_recog/embeddingsTiny.json")
    # === LAPTOP OPTION (choose YOLO model first and update pathname below) ===
    # node = FaceRecognitionNode("embeddings.json")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

#Previously in image_callback 
#undistorted_frame = undistort_image(frame)