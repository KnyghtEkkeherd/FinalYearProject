import cv2
import numpy as np
import glob

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

# get the camera matrix
# camera_callibrate(images=glob.glob('*.jpg'))

# restore the image
# image_restore(image_path='test.jpg', restored_path='test_norm.jpg', coefficients_file='calibration_data.npz')
