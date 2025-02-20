import cv2
import numpy as np
import glob

def camera_callibrate(images):
    # Code from OpenCV tuto:
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

def image_restore(image_in, coefficients_file):
    data = np.load(coefficients_file)
    camera_matrix = data['camera_matrix']
    distortion_coefficients = data['dist_coeffs']
    image = cv2.imread(image_in)

    h, w = image.shape[:2]
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, distortion_coefficients, (w, h), 1, (w, h))
    undistorted_image = cv2.undistort(image, camera_matrix, distortion_coefficients, None, new_camera_matrix)

    x, y, w, h = roi
    undistorted_image = undistorted_image[y:y+h, x:x+w]
    return undistorted_image

def down_sample_img(image_in, scaling_factor=2):
    blurred_image = cv2.GaussianBlur(image_in, (5, 5), 0)
    downsampled_image = cv2.resize(src=blurred_image, dsize=(blurred_image.shape[1] // scaling_factor, blurred_image.shape[0] // scaling_factor), interpolation=cv2.INTER_LINEAR)
    return downsampled_image
