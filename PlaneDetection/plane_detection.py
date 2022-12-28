from functions import *
import random
import time
import math

def main():
    #points = ReadPlyPoint('/home/andre/Desktop/4º ANO/VC/Projeto/VC-BoxesProject/DataSet/FFonseca/CaixaCastanha/Frame368.ply')
    points = ReadPlyPoint('/Users/marfre/VC-BoxesProject/DataSet/FFonseca/CaixaCastanha/Frame368.ply')
    
    #Axes =  o3d.geometry.TriangleMesh.create_coordinate_frame(1)
    #o3d.visualization.draw_geometries([pcd, Axes])

    
    # pre-processing
    #points = RemoveNan(points)
    points = DownSample(points,voxel_size=0.05)
    points = RemoveNoiseStatistical(points, nb_neighbors=20, std_ratio=0.8)

    #DrawResult(points)

    t0 = time.time()
    # Got the best parameters to detect all planes
    results = DetectMultiPlanes(points, min_ratio=0.001, threshold=8, iterations=2000)
    print('Time:', time.time() - t0)
    planes = []
    colors = []

    # Array to store a dictionary for every plane
    # The key in a integer; the value is a dictionary
    dictionary = {}

    # Variable to count number of planes
    plane_counter = 0

    # Store the array of plane values
    plane_values = []

    for w, plane in results:

        plane_counter = plane_counter + 1

        if plane_counter != 1:

            r = random.random()
            g = random.random()
            b = random.random()

            color = np.zeros((plane.shape[0], plane.shape[1]))
            color[:, 0] = r
            color[:, 1] = g
            color[:, 2] = b

            planes.append(plane)
            colors.append(color)

            # Print the plan equation
            [a, b, c, d] = w
            print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
            DrawResult(plane, color)

            plane_values.append([a, b, c, d])


        else:
            continue

        ''' 
        # Dictionary for each plane. 
        #Key = distance between; Value = array of the two points
        plane_dic = {}

        # Ignore the first one wish is the table
        if(plane_counter == 1):
            continue

        # Run through every point
        for i in plane:
            for j in plane:

                # To store the value in the plane_dic
                point_Arr = []
                point_Arr.append(i)
                point_Arr.append(j)

                # Point dist
                point_dist = math.dist(i, j)

                plane_dic[point_dist] = point_Arr 

        # Add the plane_dic to the dictionary
        dictionary[plane_counter] = plane_dic
        '''
        
    #planes = np.concatenate(planes, axis=0)
    #colors = np.concatenate(colors, axis=0)
    #DrawResult(planes, colors)

    #inter_points = plane_intersect(plane_values[0], plane_values[1])

    #inter_point1 = inter_points[0]
    #inter_point2 = inter_points[1]
    

    #dist = math.dist(inter_point1, inter_point2)
    #print(dist)

    

    #inter_point1 = np.array([[inter_point1[0], inter_point1[1], inter_point1[2]]])
    #inter_point2 = np.array([[inter_point2[0], inter_point2[1], inter_point2[2]]])
    #inter_point2.append(0)



    #planes.append(inter_point1)
    #planes.append(inter_point2)

    
    '''
    for a in planes:
        print(a)

    print('\nCOLORS')
    for a in color:
        print(a)
    '''

    print('\nPlane 0 dims:')
    print(boundingBox3D(planes[0], False))

    print('\nPlane 1 dims:')
    print(boundingBox3D(planes[1], True))
    
    planes = np.concatenate(planes, axis=0)
    colors = np.concatenate(colors, axis=0)

    
    DrawResult(planes, colors)

    #print(point_intersecpt2)

    '''
    # Dictionary to get the 4 biggest distances of the plane
    big4Dist_plane_Dic = {}
    # Variable to count number of planes
    plane_counter = 0

    # Run through all plane dictionarys
    for key in dictionary:

        plane_counter = plane_counter + 1
        dist_arr = []

        # Run throun one plane dicitionary
        for plane_key in dictionary[key]:
            dist_arr.append(plane_key)

        # Get the biggest distances to the end
        dist_arr.sort()

        # Get the 4 biggest (4 in last) in to the dictionary
        big4Dist_plane_Dic[plane_counter] = dist_arr[-4:]
        print(dist_arr[-4:])
        print('\n')
    '''








if __name__ == "__main__":
    main()
    

    