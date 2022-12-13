from functions import *
import random
import time

def main():
    points = ReadPlyPoint('/Users/marfre/VC-BoxesProject/DataSet/FFonseca/CaixaCastanha/Frame368.ply')
    
    # pre-processing
    points = RemoveNan(points)
    points = DownSample(points,voxel_size=0.003)
    points = RemoveNoiseStatistical(points, nb_neighbors=50, std_ratio=0.5)

    #DrawPointCloud(points, color=(0.4, 0.4, 0.4))
    t0 = time.time()
    results = DetectMultiPlanes(points, min_ratio=0.05, threshold=0.005, iterations=2000)
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
    

    