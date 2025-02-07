# MAKE SURE YOU INSTALL BEFORE RUNNING: pip3 install face_recognition

import rclpy
from rclpy.node import Node
import face_recognition
import cv2
import os
import time

class FacialRecognitionNode(Node):
    def __init__(self):
        super().__init__('facial_recognition_node')
        self.get_logger().info('Facial Recognition Node Started')
        
        # Get the directory where this script is located
        script_dir = os.path.dirname(os.path.abspath(__file__))
        
        # Paths
        # Assuming 'media' folder is located at the workspace root
        self.media_path = os.path.join('../media')
        self.media_path = os.path.abspath(self.media_path)
        self.training_images = ['armaan1.png', 'armaan2.jpeg']
        self.camera_image = os.path.join(self.media_path, 'capture_image.jpg')
        
        # Load training images and encode faces
        self.known_face_encodings = []
        self.known_face_names = []

        for image_name in self.training_images:
            image_path = os.path.join(self.media_path, image_name)
            if not os.path.exists(image_path):
                self.get_logger().error(f'Image not found: {image_path}')
                continue
            image = face_recognition.load_image_file(image_path)
            face_encodings = face_recognition.face_encodings(image)
            if face_encodings:
                self.known_face_encodings.append(face_encodings[0])
                self.known_face_names.append('Armaan')
                self.get_logger().info(f'Loaded and encoded {image_name}')
            else:
                self.get_logger().warn(f'No face found in {image_name}')
        
        # Create a timer to periodically recognize faces
        self.timer = self.create_timer(5.0, self.recognize_face)

    def recognize_face(self):
        # Capture image from Mac camera
        cap = cv2.VideoCapture(0)
        time.sleep(0.5)  # Wait for 0.5 seconds to allow the camera to warm up
        if not cap.isOpened():
            self.get_logger().error('Cannot open camera')
            return
        
        ret, frame = cap.read()
        cap.release()

        if ret:
            cv2.imwrite(self.camera_image, frame)
            self.get_logger().info('Captured image from camera')

            # Load the captured image and find face encodings
            unknown_image = face_recognition.load_image_file(self.camera_image)
            unknown_face_encodings = face_recognition.face_encodings(unknown_image)

            if unknown_face_encodings:
                for face_encoding in unknown_face_encodings:
                    # Compare faces
                    matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding)
                    face_distances = face_recognition.face_distance(self.known_face_encodings, face_encoding)
                    best_match_index = face_distances.argmin()

                    if matches[best_match_index]:
                        name = self.known_face_names[best_match_index]
                    else:
                        name = 'Unknown'

                    self.get_logger().info(f'Recognized: {name}')
            else:
                self.get_logger().info('No face detected in the captured image')
        else:
            self.get_logger().error('Failed to capture image from camera')

def main(args=None):
    rclpy.init(args=args)
    node = FacialRecognitionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Facial Recognition Node')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()