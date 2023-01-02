# Detect multi planes using clustering
#   3D Shape Detection with RANSAC
#   Clustering with DBSCAN
#   Refined RANSAC with Euclidean clustering loop for multiple planar shapes detection

#libraries used
import numpy as np
import open3d as o3d
import matplotlib.pyplot as plt
from functions import *



def clusterDetection(path, verbose=False):
    ''' 
        Detect planes in multi boxes
            Input: path
            Output: array of points array [[x,y,z],...]
    '''

    # create paths and load data
    points = ReadPlyPoint(path)

    points = DownSample(points,voxel_size=0.005)
    #points = RemoveNoiseStatistical(points, nb_neighbors=10, std_ratio=0.8)

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    DrawResult(pcd.points)

    '''
    # [Optional] Normals computation
    pcd.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=16), fast_normal_computation=True)
    pcd.paint_uniform_color([0.6, 0.6, 0.6])
    o3d.visualization.draw_geometries([pcd]) #Works only outside Jupyter/Colab
    '''

    # Refined RANSAC with Euclidean clustering
    segment_models={}
    segments={}
    max_plane_idx=16
    rest=pcd
    d_threshold=40
    for i in range(max_plane_idx):
        colors = plt.get_cmap("tab20")(i)
        
        segment_models[i], inliers = rest.segment_plane(distance_threshold=10,ransac_n=3,num_iterations=2000)
        #segments[i]=rest.select_by_index(inliers, invert=True)
        segments[i]=rest.select_by_index(inliers)

        #points = ReadPlyPoint(segments[i])
        
        labels = np.array(segments[i].cluster_dbscan(eps=d_threshold*1, min_points=30))
        candidates=[len(np.where(labels==j)[0]) for j in np.unique(labels)]
        best_candidate_test = int(np.unique(labels)[np.where(candidates==np.max(candidates))[0]])

        if (best_candidate_test == -1):
            i -= 1
            continue
        
        best_candidate=best_candidate_test
        #if (verbose == True):
            #print("the best candidate is: ", best_candidate)

        rest = rest.select_by_index(inliers, invert=True)+segments[i].select_by_index(list(np.where(labels!=best_candidate)[0]))
        segments[i]=segments[i].select_by_index(list(np.where(labels==best_candidate)[0]))
        segments[i].paint_uniform_color(list(colors[:3]))
        
        if (verbose == True):
            print("pass",i+1,"/",max_plane_idx,"done.")

    if (verbose == True):
        o3d.visualization.draw_geometries([segments[i] for i in range(max_plane_idx)])


    '''
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
    '''
    planes = []
    for j in range(len(segments)):
        planes.append(PCDToNumpy(segments[j]))
    
    
    return planes
