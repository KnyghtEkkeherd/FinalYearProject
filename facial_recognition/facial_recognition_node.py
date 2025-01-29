# 1. Before running:

# Import the required packages
    # sudo apt-get install python3-opencv
    # pip install face_recognition
    # pip install numpy

# And replace "path_to_image.jpg" and "Person Name" with the actual image paths and names

# 2. To run:

# Run the facial recognition node
    # ros2 run your_package_name face_recognition_node

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import face_recognition
import numpy as np

class FaceRecognitionNode(Node):
    def __init__(self):
        super().__init__('face_recognition_node')
        self.image_subscriber = self.create_subscription(Image, 'camera/image_raw', self.image_callback, 10)
        self.face_pub = self.create_publisher(String, 'recognized_face', 10)

        # Load known face encodings and names
        self.known_face_encodings = []
        self.known_face_names = []
        self.load_known_faces()

    def load_known_faces(self):
        # Load images and create encodings
        # Example for one person
        known_image = face_recognition.load_image_file("path_to_image.jpg")
        self.known_face_encodings.append(face_recognition.face_encodings(known_image)[0])
        self.known_face_names.append("Person Name")

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Recognize faces in the image
        self.recognize_faces(image)

    def recognize_faces(self, image):
        # Resize the image for faster recognition
        rgb_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        face_locations = face_recognition.face_locations(rgb_image)
        face_encodings = face_recognition.face_encodings(rgb_image, face_locations)

        for face_encoding in face_encodings:
            matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding)
            name = "Unknown"

            # Use the known face with the smallest distance to the new face
            face_distances = face_recognition.face_distance(self.known_face_encodings, face_encoding)
            best_match_index = np.argmin(face_distances)

            if matches[best_match_index]:
                name = self.known_face_names[best_match_index]

            # Publish recognized face name
            self.face_pub.publish(name)

def main(args=None):
    rclpy.init(args=args)
    face_recognition_node = FaceRecognitionNode()
    rclpy.spin(face_recognition_node)
    face_recognition_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()