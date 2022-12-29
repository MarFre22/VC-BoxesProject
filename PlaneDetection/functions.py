""" 
	In this script are present
all functions we are going to use
and we find helpfull

"""
import numpy as np
import open3d as o3d
import math




def ReadPlyPoint(fname):
    """ read points

    Args:
        fname (str): path point cloud file

    Returns:
        [ndarray]: N x 3 point clouds
    """

    pcd = o3d.io.read_point_cloud(fname, remove_nan_points=True, remove_infinite_points=True)

    return PCDToNumpy(pcd)


def NumpyToPCD(xyz):
    """ convert numpy ndarray to open3D point cloud 

    Args:
        xyz (ndarray): 

    Returns:
        [open3d.geometry.PointCloud]: 
    """

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)

    return pcd


def PCDToNumpy(pcd):
    """  convert open3D point cloud to numpy ndarray

    Args:
        pcd (open3d.geometry.PointCloud): 

    Returns:
        [ndarray]: 
    """

    return np.asarray(pcd.points)


def RemoveNan(points):
    """ remove nan value of point clouds

    Args:
        points (ndarray): N x 3 point clouds

    Returns:
        [ndarray]: N x 3 point clouds
    """

    return points[~np.isnan(points[:, 0])]


def RemoveNoiseStatistical(pc, nb_neighbors=20, std_ratio=2.0, pcd=False):
    """ remove point clouds noise using statitical noise removal method

    Args:
        pc (ndarray): N x 3 point clouds
        nb_neighbors (int, optional): Defaults to 20.
        std_ratio (float, optional): Defaults to 2.0.

    Returns:
        [ndarray]: N x 3 point clouds
    """

    if pcd==False:
        pcd = NumpyToPCD(pc)

    cl, ind = pcd.remove_statistical_outlier(
        nb_neighbors=nb_neighbors, std_ratio=std_ratio)

    return PCDToNumpy(cl)


def DownSample(pts, voxel_size=0.003):
    """ down sample the point clouds

    Args:
        pts (ndarray): N x 3 input point clouds
        voxel_size (float, optional): voxel size. Defaults to 0.003.

    Returns:
        [ndarray]: 
    """

    p = NumpyToPCD(pts).voxel_down_sample(voxel_size=voxel_size)

    return PCDToNumpy(p)


def PlaneRegression(points, threshold=0.01, init_n=3, iter=1000):
    """ plane regression using ransac

    Args:
        points (ndarray): N x3 point clouds
        threshold (float, optional): distance threshold. Defaults to 0.003.
        init_n (int, optional): Number of initial points to be considered inliers in each iteration
        iter (int, optional): number of iteration. Defaults to 1000.

    Returns:
        [ndarray, List]: 4 x 1 plane equation weights, List of plane point index
    """

    pcd = NumpyToPCD(points)

    w, index = pcd.segment_plane(
        threshold, init_n, iter)

    return w, index


def DrawResult(points, colors=[]):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    if len(colors) != []:
        pcd.colors = o3d.utility.Vector3dVector(colors)

    o3d.visualization.draw_geometries([pcd])




def DetectMultiPlanes(points, min_ratio=0.05, threshold=0.01, iterations=1000):
    """ Detect multiple planes from given point clouds
    Args:
        points (np.ndarray): 
        min_ratio (float, optional): The minimum left points ratio to end the Detection. Defaults to 0.05.
        threshold (float, optional): RANSAC threshold in (m). Defaults to 0.01.
    Returns:
        [List[tuple(np.ndarray, List)]]: Plane equation and plane point index
    """

    plane_list = []
    N = len(points)
    target = points.copy()
    count = 0

    while count < (1 - min_ratio) * N:
        w, index = PlaneRegression(
            target, threshold=threshold, init_n=3, iter=iterations)
    
        count += len(index)
        plane_list.append((w, target[index]))
        target = np.delete(target, index, axis=0)

    return plane_list


def plane_intersect(a, b):
    '''
    a, b   4-tuples/lists
           Ax + By +Cz + D = 0
           A,B,C,D in order  

    output: 2 points on line of intersection, np.arrays, shape (3,)
    '''
    a_vec, b_vec = np.array(a[:3]), np.array(b[:3])

    aXb_vec = np.cross(a_vec, b_vec)

    A = np.array([a_vec, b_vec, aXb_vec])
    d = np.array([-a[3], -b[3], 0.]).reshape(3,1)

    # could add np.linalg.det(A) == 0 test to prevent linalg.solve throwing error

    p_inter = np.linalg.solve(A, d).T

    return p_inter[0], (p_inter + aXb_vec)[0]

def give_color(a, color_name):

    if color_name == 'r':
        r = 1
        g = 0
        b = 0

    r = random.random()
    g = random.random()
    b = random.random()

    color = np.zeros((plane.shape[0], plane.shape[1]))
    color[:, 0] = r
    color[:, 1] = g
    color[:, 2] = b

    colors.append(color)

    return color_array

def boundingBox3D(points, verbose=False):
    ''' 
        Detect the best dimentions of the 3D bounding box to a given points (cloud)
            Input: points
            Output: dimentions in array: [float, float, float]
    '''

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    
    # Flip it, otherwise the pointcloud will be upside down.
    #pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
    axis_aligned_bounding_box = pcd.get_axis_aligned_bounding_box()
    axis_aligned_bounding_box.color = (1, 0, 0)
    oriented_bounding_box = pcd.get_oriented_bounding_box(robust = True)
    #oriented_bounding_box = open3d.geometry.OrientedBoundingBox.create_from_points(points)
    oriented_bounding_box.color = (0, 1, 0)

    if verbose == True:
        print(
            "\nDisplaying the initial axis_aligned_bounding_box in red and oriented bounding box in green ..."
        )
        o3d.visualization.draw(
            [pcd, axis_aligned_bounding_box, oriented_bounding_box])


    # Transpose in order to invert the rotation
    rotation = np.transpose(oriented_bounding_box.R)

    if verbose == True:
        print('\nTransposed rotation matrix:')
        print(rotation)
    
    pcd_transposed =  pcd.rotate(rotation)
    axis_aligned_bounding_box_rotated = pcd_transposed.get_axis_aligned_bounding_box()
    axis_aligned_bounding_box_rotated.color = (1,1,0)
    

    vol_aligned_bounding_box_rotated = axis_aligned_bounding_box_rotated.volume()
    minimized_vol = 'no'
    aligned_bounding_box_rotated_final = axis_aligned_bounding_box_rotated
    dims_final = axis_aligned_bounding_box_rotated.get_extent()

    # Minimize volume of Axis aligned bounding Box
    #for angle in np.arange(0, 10, 0.5):
    for angle in np.arange(0, 50, 0.05):
        
        if verbose == True:
            print(angle)

        pcd_test = pcd_transposed

        angle_rad = angle = math.radians(angle)

        matrix_rot_z = np.array([[math.cos(angle_rad),  -math.sin(angle_rad), 0], 
                            [math.sin(angle_rad), math.cos(angle_rad), 0], 
                            [0, 0, 1]])
        matrix_rot_z =  matrix_rot_z.astype(np.float64)

        aligned_bounding_box_rotated_test = pcd_test.rotate(matrix_rot_z).get_axis_aligned_bounding_box()

        dims = aligned_bounding_box_rotated_test.get_extent()

        vol = aligned_bounding_box_rotated_test.volume()

        # If script already found a volume lower than de initial one 
        if vol_aligned_bounding_box_rotated != axis_aligned_bounding_box_rotated.volume():
            # If the next vol as increased, stop the loop to make it faster
            if vol > vol_aligned_bounding_box_rotated:
                break

        if vol <= vol_aligned_bounding_box_rotated:
            # Save all info
            dims_final = dims
            minimized_vol = 'yes'
            aligned_bounding_box_rotated_final = aligned_bounding_box_rotated_test
            
            # Min volume
            vol_aligned_bounding_box_rotated = vol
        
        

        if (verbose == True):
            # Print
            o3d.visualization.draw(
                [pcd, aligned_bounding_box_rotated_test, axis_aligned_bounding_box_rotated])

    if verbose == True:
        print("\nMinimized volume axis aligned bounding box?  "+ minimized_vol)
        print(dims_final)

        print("\nDisplaying best fit bounding box (White), transposed bounding box (Yellow)")
        o3d.visualization.draw(
            [pcd, aligned_bounding_box_rotated_final, axis_aligned_bounding_box_rotated])
    
    return dims_final
