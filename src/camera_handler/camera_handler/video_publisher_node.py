import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
import numpy as np
import subprocess
import os
import time
import json
import threading
from camera_interface.srv import PublishImage

# encodings_mutex = threading.Lock()

# def train_model(training_dataset_path="training_data"):
#         if not os.path.exists(training_dataset_path):
#             raise ValueError("Dataset path does not exist")

#         with open(f"{training_dataset_path}/training_data.json", mode='r') as file:
#             data = json.load(file)
#             names = [entry['name'] for entry in data]
#             image_paths = [entry['image'] for entry in data]

#         encodings_data = []
#         for name, image_path in zip(names, image_paths):
#             image = cv2.imread(os.path.join(training_dataset_path, "training_images", image_path))
#             encoding = face_recognition.face_encodings(image)[0]
#             encodings_data.append({
#                 'name': name,
#                 'image': image_path,
#                 'encoding': encoding.tolist()
#             })
#         with encodings_mutex:
#             with open(f"{training_dataset_path}/training_encodings.json", mode='w') as file:
#                 json.dump(encodings_data, file, indent=4)

# def recognize_faces(image_path, encodings_path="training_data/training_encodings.json"):
#     if not os.path.exists(encodings_path):
#         raise ValueError("Dataset path does not exist")

#     with encodings_mutex:
#         with open(encodings_path, mode='r') as file:
#             encodings_data = json.load(file)

#     known_face_encodings = [entry['encoding'] for entry in encodings_data]
#     image = cv2.imread(image_path)
#     small_image= cv2.resize(image, (0, 0), fx=0.25, fy=0.25)
#     small_image_rgb = small_image[:, :, ::-1]
#     face_locations = face_recognition.face_locations(small_image_rgb)
#     face_encodings = face_recognition.face_encodings(small_image_rgb, face_locations)

#     face_names = []
#     for face_encoding in face_encodings:
#         matches = face_recognition.compare_faces(known_face_encodings, face_encoding)

#         name = "Unknown"
#         face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
#         best_match_index = np.argmin(face_distances)
#         if matches[best_match_index]:
#             name = encodings_data[best_match_index]['name']
#         face_names.append(name)

#     return zip(face_locations, face_names)

def camera_callibrate(images):
    # Dimensions of the chessboard (number of internal corners)
    grid_size = (9, 6)

    # The real-world size of each square (2 cm)
    square_size = 2  # cm

    # 3D world coordinates of the chessboard
    obj_points = np.zeros((grid_size[0] * grid_size[1], 3), np.float32)
    obj_points[:, :2] = np.mgrid[0:grid_size[0], 0:grid_size[1]].T.reshape(-1, 2) * square_size

    # Lists for storing the necessary points for calibration
    object_points = []
    image_points = []

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, grid_size, None)

        if ret:
            object_points.append(obj_points)
            image_points.append(corners)
            cv2.drawChessboardCorners(img, grid_size, corners, ret)
            cv2.imshow('Chessboard Corners', img)
            cv2.waitKey(500)

    cv2.destroyAllWindows()
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(object_points, image_points, gray.shape[::-1], None, None)
    np.savez('calibration_data.npz', camera_matrix=camera_matrix, dist_coeffs=dist_coeffs)

def image_restore(image_path, restored_path, coefficients_file='calibration_data.npz'):
    data = np.load(coefficients_file)
    camera_matrix = data['camera_matrix']  # Correct key name
    distortion_coefficients = data['dist_coeffs']  # Correct key name
    image = cv2.imread(image_path)

    h, w = image.shape[:2]
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_coefficients, (w, h), 1, (w, h))
    undistorted_image = cv2.undistort(image, camera_matrix, distortion_coefficients, None, new_camera_matrix)

    x, y, w, h = roi
    undistorted_image = undistorted_image[y:y+h, x:x+w]
    cv2.imwrite(restored_path, undistorted_image)


class CameraHandler(Node):
    def __init__(self):
        super().__init__('camera_handler')
        self.get_image_srv = self.create_service(PublishImage, 'get_image', self.camera_callback)
        #self.facial_recognition_srv = self.create_service(TriggerImageRecognition, 'facial_recognition', self.facial_recognition_callback)

    # def facial_recognition_callback(self, request, response):
    #     try:
    #         timestamp = int(time.time())
    #         temp_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), f'temp/camera_{timestamp}.jpg')
    #         subprocess.run(['sudo', 'rpicam-still', '--output', temp_file, '--width', '2026', '--height', '1520', '--encoding', 'jpg', '--immediate'])

    #         restored_image = f"temp/camera_{timestamp}_restored.jpg"
    #         image_restore(temp_file, restored_image)
    #         bridge = CvBridge()
    #         image_msg = bridge.cv2_to_imgmsg(cv2.imread(restored_image), encoding="bgr8")
    #         face_locations, face_names = recognize_faces(restored_image)
    #         response.image = image_msg
    #         # TODO: publish the face locations and names

    #         os.remove(temp_file)
    #         os.remove(restored_image)
    #         self.get_logger().info('Facial recognition completed')

    #     except Exception as e:
    #         self.get_logger().error(f'Failed to capture image: {e}')

    def camera_callback(self, request, response):
        try:
            timestamp = int(time.time())
            temp_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), f'temp/camera_{timestamp}.jpg')
            subprocess.run(['sudo', 'rpicam-still', '--output', temp_file, '--width', '2026', '--height', '1520', '--encoding', 'jpg', '--immediate'])
            self.get_logger().info('Image capture completed')
            
            restored_image = f"temp/camera_{timestamp}_restored.jpg"
            image_restore(temp_file, restored_image)
            bridge = CvBridge()
            image_msg = bridge.cv2_to_imgmsg(cv2.imread(restored_image), encoding="bgr8")
            response.image = image_msg

            os.remove(temp_file)
            os.remove(restored_image)
            self.get_logger().info('Image published')

        except Exception as e:
            self.get_logger().error(f'Failed to capture image: {e}')

    @staticmethod
    def read_image(file_path):
        from PIL import Image as PILImage
        image = PILImage.open(file_path)
        return np.array(image)

def main(args=None):
    rclpy.init(args=args)
    camera_handler = CameraHandler()
    rclpy.spin(camera_handler)

    camera_handler.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
