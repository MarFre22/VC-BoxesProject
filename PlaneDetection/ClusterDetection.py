#libraries used
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt

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

def PCDToNumpy(pcd):
    """  convert open3D point cloud to numpy ndarray

    Args:
        pcd (open3d.geometry.PointCloud): 

    Returns:
        [ndarray]: 
    """

    return np.asarray(pcd.points)

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

# create paths and load data
#input_path="gdrive/My Drive/_MEDIUM/DATA/Point Cloud Sample/"
#dataname="TLS_kitchen.ply"
points = ReadPlyPoint('/home/worka/Downloads/VC-Projeto/VC-BoxesProject/DataSet/FFonseca/VáriasCaixas/Frame374.ply')
#pcd = o3d.io.read_point_cloud('/Users/marfre/VC-BoxesProject/DataSet/FFonseca/VáriasCaixas/Frame371.ply')

points = DownSample(points,voxel_size=0.005)
#points = RemoveNoiseStatistical(points, nb_neighbors=10, std_ratio=0.8)

pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(points)

'''
# [Optional] Normals computation
pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=16), fast_normal_computation=True)
pcd.paint_uniform_color([0.6, 0.6, 0.6])
o3d.visualization.draw_geometries([pcd]) #Works only outside Jupyter/Colab
'''

'''
# [INITIATION] 3D Shape Detection with RANSAC
plane_model, inliers = pcd.segment_plane(distance_threshold=10,ransac_n=3,num_iterations=2000)
[a, b, c, d] = plane_model
print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
inlier_cloud = pcd.select_by_index(inliers)
outlier_cloud = pcd.select_by_index(inliers, invert=True)
#inlier_cloud = pcd1
#outlier_cloud = pcd2
inlier_cloud.paint_uniform_color([1.0, 0, 0])
outlier_cloud.paint_uniform_color([0.6, 0.6, 0.6])
o3d.visualization.draw_geometries([inlier_cloud, outlier_cloud])




# [INITIATION] Clustering with DBSCAN
labels = np.array(pcd.cluster_dbscan(eps=0.05, min_points=10))
max_label = labels.max()
print(f"point cloud has {max_label + 1} clusters")

colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
colors[labels < 0] = 0
pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])

o3d.visualization.draw_geometries([pcd])

'''

'''
# RANSAC loop for multiple planar shapes detection
segment_models={}
segments={}
max_plane_idx=10
rest=pcd
for i in range(max_plane_idx):
    colors = plt.get_cmap("tab20")(i)
    segment_models[i], inliers = rest.segment_plane(distance_threshold=10,ransac_n=3,num_iterations=2000)
    segments[i]=rest.select_by_index(inliers)
    segments[i].paint_uniform_color(list(colors[:3]))
    rest = rest.select_by_index(inliers, invert=True)
    print("pass",i,"/",max_plane_idx,"done.")

o3d.visualization.draw_geometries([segments[i] for i in range(max_plane_idx)]+[rest])

exit()
'''

# Refined RANSAC with Euclidean clustering
segment_models={}
segments={}
max_plane_idx=20
rest=pcd
d_threshold=10
for i in range(max_plane_idx):
    colors = plt.get_cmap("tab20")(i)
    
    segment_models[i], inliers = rest.segment_plane(distance_threshold=10,ransac_n=3,num_iterations=2000)
    segments[i]=rest.select_by_index(inliers)
    
    labels = np.array(segments[i].cluster_dbscan(eps=d_threshold*12, min_points=10))
    candidates=[len(np.where(labels==j)[0]) for j in np.unique(labels)]
    best_candidate=int(np.unique(labels)[np.where(candidates==np.max(candidates))[0]])
    print("the best candidate is: ", best_candidate)


    rest = rest.select_by_index(inliers, invert=True)+segments[i].select_by_index(list(np.where(labels!=best_candidate)[0]))
    segments[i]=segments[i].select_by_index(list(np.where(labels==best_candidate)[0]))
    segments[i].paint_uniform_color(list(colors[:3]))
    print("pass",i+1,"/",max_plane_idx,"done.")

o3d.visualization.draw_geometries([segments[i] for i in range(max_plane_idx)])

exit()


# Euclidean clustering of the rest with DBSCAN
labels = np.array(rest.cluster_dbscan(eps=0.05, min_points=10))
max_label = labels.max()
print(f"point cloud has {max_label + 1} clusters")

colors = plt.get_cmap("tab10")(labels / (max_label if max_label > 0 else 1))
colors[labels < 0] = 0
rest.colors = o3d.utility.Vector3dVector(colors[:, :3])

# o3d.visualization.draw_geometries([segments.values()])
#o3d.visualization.draw_geometries([segments[i] for i in range(max_plane_idx)]+[rest])
#o3d.visualization.draw_geometries([segments[i] for i in range(max_plane_idx)]+[rest],zoom=0.3199,front=[0.30159062875123849, 0.94077325609922868, 0.15488309545553303],lookat=[-3.9559999108314514, -0.055000066757202148, -0.27599999308586121],up=[-0.044411423633999815, -0.138726419067636, 0.98753122516983349])
# o3d.visualization.draw_geometries([rest])


