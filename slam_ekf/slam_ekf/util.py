#!/usr/bin/python3
import numpy as np
from rclpy.time import Time
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData
from scipy.optimize import minimize
from scipy.spatial.transform import Rotation
from PIL import Image, ImageFilter
from threading import Lock

map_lock = Lock()
map_filtered_lock = Lock()

def angle_between_yaw(yaw1, yaw2):
    """calculates the angle between two frames
    specified by their yaw angles. Avoid having
    to deal with wrapping the angles by expressing
    frame 2 under frame 1

    Args:
        yaw1 (_type_): yaw angle of the ref frame
        yaw2 (_type_): yaw angle of the query frame/vector

    Returns:
        theta: yaw2 minus yaw1 expressed in yaw1 frame
    """
    s = np.sin(yaw1)
    c = np.cos(yaw1)
    R = np.array([[c,-s],[s,c]])
    p = np.array([np.cos(yaw2),np.sin(yaw2)])[:,np.newaxis]
    p_ = R.T.dot(p) # expressed in the frame of yaw1
    theta = np.arctan2(p_[1,0],p_[0,0])
    return theta

def circle_loss(x, data):
    """calculate the loss for fitting the center
    and radius of the cylinder, based on lidar reflection
    points

    Args:
        x (_type_): a 3 by 1 vector of cylinder center (x,y)
        and radius r
        data (_type_): the lidar points expressed as xy coordinates

    Returns:
        out: the loss to be minimized
    """
    # data is 2d array
    out = (data[:,0]-x[0])**2 + (data[:,1]-x[1])**2 - x[2]**2
    out = np.sum(out**2)
    return out

def initial_value(data : np.ndarray):
    """calculates the initial guess of the cylinder center
    and radius

    Args:
        data (np.ndarray): _description_

    Returns:
        _type_: _description_
    """
    if data.shape[0] >= 4: # at least 4 points
        n = data.shape[0]
        i1,i2,i3,i4 = np.random.permutation(n)[:4]

        # pick two pairs of points
        x1,y1 = data[i1,:]
        x2,y2 = data[i2,:]
        x3,y3 = data[i3,:]
        x4,y4 = data[i4,:]

        A = np.array([[x1-x2,y1-y2],[x3-x4,y3-y4]])
        b = np.array([0.5*(x1**2-x2**2)+0.5*(y1**2-y2**2),\
                      0.5*(x3**2-x4**2)+0.5*(y3**2-y4**2)])[:,np.newaxis]

        c = np.linalg.inv(A).dot(b)
        c : np.ndarray = c.flatten()
        r = np.mean(np.linalg.norm(data-c[np.newaxis,:],axis=1))

        return np.concatenate([c,[r]])
    else:
        return None

def get_center_radius(data : np.ndarray):
    """find the center and radius of the cylinder
    through minimizing the loss function

    Args:
        data (np.ndarray): _description_

    Returns:
        _type_: _description_
    """
    i = 0
    while i <= 10:
        i+=1
        # compute initial guess
        v0 = initial_value(data)
        if v0 is None:
            return None, None
        out = minimize(circle_loss, x0=v0,args=(data,))
        if out.success is True:
            # check the residue
            c = out.x[:2]
            r = out.x[-1]
            res = np.abs(np.linalg.norm(data-c[np.newaxis,:],axis=1)-r)
            if np.mean(res)<=0.1:
                return c, r

    return None, None

def quaternion_to_yaw(msg : Odometry):
    """extract yaw info from odom messages

    Args:
        msg (Odometry): _description_

    Returns:
        _type_: _description_
    """
    r = Rotation.from_quat([msg.pose.pose.orientation.x,
                            msg.pose.pose.orientation.y,
                            msg.pose.pose.orientation.z,
                            msg.pose.pose.orientation.w])
    rpy = r.as_euler('xyz')
    return rpy[-1]

def lidar_points_to_occupancy_grid(lidar_pts_fixedframe, image_size=(1000, 1000), scale=10, file_path="media/occupancy_grid.png"):
    """Save the LiDAR points in fixed frame as an occupancy grid and save to file.

    Args:
        lidar_pts_fixedframe (numpy.ndarray): The fixed frame LiDAR points (Nx2 array).
        image_size (tuple): Size of the grid (width, height).
        scale (int): Scale to convert real-world coordinates to grid cells.
        file_path (str): Path to save the occupancy grid image.

    Returns:
        nav_msgs.msg.OccupancyGrid: The ROS2 OccupancyGrid message containing the occupancy grid.
    """

    # Initialize a blank occupancy grid with unknown values (-1)
    grid = np.ones(image_size, dtype=np.int8) * -1

    # Iterate over each point in the fixed frame LiDAR points
    for point in lidar_pts_fixedframe:
        x, y = point

        # Scale and translate the coordinates to fit into the grid
        x_grid = int(x * scale + image_size[0] // 2)
        y_grid = int(-y * scale + image_size[1] // 2)  # Invert y-axis for grid coordinates

        # Check bounds and set cell to occupied (100)
        if 0 <= x_grid < image_size[0] and 0 <= y_grid < image_size[1]:
            grid[y_grid, x_grid] = 100  # Set cell to occupied

    # Save the occupancy grid to an image file
    img = Image.fromarray(grid.astype(np.uint8) * 255)  # Convert to 0-255 scale
    with map_lock:
        img.save(file_path)

    # Create the ROS2 OccupancyGrid message
    occupancy_grid = OccupancyGrid()
    occupancy_grid.info = MapMetaData()
    occupancy_grid.info.resolution = 1.0 / scale
    occupancy_grid.info.width = image_size[0]
    occupancy_grid.info.height = image_size[1]
    occupancy_grid.info.origin.position.x = float(-image_size[0] // (2 * scale))
    occupancy_grid.info.origin.position.y = float(-image_size[1] // (2 * scale))
    occupancy_grid.info.origin.position.z = 0.0
    occupancy_grid.info.origin.orientation.w = 1.0
    occupancy_grid.data = grid.flatten().tolist()

    print(f"Occupancy grid created and saved to {file_path}")
    gaussian_filter(file_path)
    return occupancy_grid

def gaussian_filter(filepath='occupancy_grid.png'):
    with map_lock:
        image = Image.open(filepath)
    filtered_image = image.filter(ImageFilter.GaussianBlur(radius=2))
    binary_image = filtered_image.point(lambda p: 255 if p > 1 else 0)
    with map_filtered_lock:
        binary_image.save('media/occupancy_grid_filtered.png')

def get_corners():
    pass
