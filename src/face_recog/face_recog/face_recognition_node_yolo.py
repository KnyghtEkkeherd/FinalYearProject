#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json
import os

# ---- CONFIGURATION: Set the paths for your models (must match the training script) ----
YOLO_CONFIG = "/home/gyattbot/FinalYearProject/src/face_recog/face_recog/yolov3-face.cfg"        # Path to YOLO config file
YOLO_WEIGHTS = "/home/gyattbot/FinalYearProject/src/face_recog/face_recog/yolov3-wider_16000.weights"  # Path to YOLO weights file
FACE_EMBEDDING_MODEL = "/home/gyattbot/FinalYearProject/src/face_recog/face_recog/openface_nn4.small2.v1.t7"  # Path to embedding model

DETECT_CONFIDENCE = 0.5
RECOGNITION_THRESHOLD = 0.6  # Euclidean distance threshold for recognition

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

        # Load the precomputed embeddings and convert them to numpy arrays.
        with open(embeddings_file, "r") as f:
            data = json.load(f)
        self.embeddings = {name: np.array(vec) for name, vec in data.items()}

        # Load YOLO detector and face embedding model.
        self.yolo_net, self.yolo_output_layers = load_yolo_detector()
        self.embedder = cv2.dnn.readNetFromTorch(FACE_EMBEDDING_MODEL)

        # Subscribe to the image topic published by camera_ros.
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        # Publisher to output recognized names.
        self.name_publisher = self.create_publisher(String, 'recognized_person', 10)

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
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        undistorted_frame = undistort_image(frame)
        embedding, bbox = self.extract_embedding(undistorted_frame)
        if embedding is None:
            self.get_logger().info("No face detected")
            return

        # Compare embedding with each stored person.
        min_distance = float("inf")
        recognized_name = "Unknown"
        for name, stored_embedding in self.embeddings.items():
            distance = np.linalg.norm(embedding - stored_embedding)
            if distance < min_distance:
                min_distance = distance
                recognized_name = name
        # If the distance is below the threshold, we consider it a match.
        if min_distance < RECOGNITION_THRESHOLD:
            out_msg = String()
            out_msg.data = recognized_name
            self.name_publisher.publish(out_msg)
            self.get_logger().info(f"Recognized: {recognized_name} (distance: {min_distance:.3f})")
        else:
            self.get_logger().info(f"No match (minimum distance: {min_distance:.3f})")

def main(args=None):
    rclpy.init(args=args)
    # Provide the path to your embeddings JSON file.
    node = FaceRecognitionNode("/home/gyattbot/FinalYearProject/src/face_recog/face_recog/embeddings.json")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
