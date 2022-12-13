from functions import *
import random
import time

def main():
    points = ReadPlyPoint('/Users/marfre/VC-BoxesProject/DataSet/FFonseca/CaixaCastanha/Frame368.ply')
    
    #Axes =  o3d.geometry.TriangleMesh.create_coordinate_frame(1)
    #o3d.visualization.draw_geometries([pcd, Axes])

    
    # pre-processing
    #points = RemoveNan(points)
    points = DownSample(points,voxel_size=0.05)
    points = RemoveNoiseStatistical(points, nb_neighbors=20, std_ratio=0.8)

    DrawResult(points)


    #DrawPointCloud(points, color=(0.4, 0.4, 0.4))
    t0 = time.time()
    results = DetectMultiPlanes(points, min_ratio=0.1, threshold=0.1, iterations=2000)
    print('Time:', time.time() - t0)
    planes = []
    colors = []
    for _, plane in results:

        r = random.random()
        g = random.random()
        b = random.random()

        color = np.zeros((plane.shape[0], plane.shape[1]))
        color[:, 0] = r
        color[:, 1] = g
        color[:, 2] = b

        planes.append(plane)
        colors.append(color)
    
    planes = np.concatenate(planes, axis=0)
    colors = np.concatenate(colors, axis=0)
    DrawResult(planes, colors)


if __name__ == "__main__":
    main()
    

    