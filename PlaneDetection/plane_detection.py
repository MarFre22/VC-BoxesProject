from functions import *
import random
import time
import math

#from ClusterDetection import *

def main(p, c, cb_pcd):


    '''
        É IMPORTANTE REFERIR QUE PARA SER POSSÍVEL OBER
    A POSIÇÃO DO KINECT PARA OBTER A NUVEM DE PONTOS TEM DE
    SER CONTANTE, CASO CONTRÁRIO É NECESSÁRIO ESTAR CONSTANTEMENTE
    A ADAPTAR O ALGORITMO PARA SATISFAZER AS NECESSIDADES DO ANGULO
    DE OBTENÇÃO.

        O ALGORTIMO ESTÁ CALIBRADO PARA A OBTENÇÃO DE PONTOS
    CUJO ANGULO DE OBTENÇÃO É COMUM AOS SEGUINTE FICHEIROS:

        CAIXA CASTANHA: Frame368.ply


        CAIXA BRANCA:   Frame355.pcd, 
                        Frame356.pcd,  
                        Frame356.ply,
                        Frame359.pcd,
                        Frame359.ply, (acerta a orientação, mas a PCD é demasiado má para tirar as dimensões)
    '''


    if(p == 0):
        if(c == 0):
            if(cb_pcd == 0):
                path = '/home/andre/Desktop/4º ANO/VC/Projeto/VC-BoxesProject/DataSet/FFonseca/CaixaBranca/Frame355.pcd'
            elif(cb_pcd == 1):
                path = '/home/andre/Desktop/4º ANO/VC/Projeto/VC-BoxesProject/DataSet/FFonseca/CaixaBranca/Frame356.pcd'
            elif(cb_pcd == 2):
                path = '/home/andre/Desktop/4º ANO/VC/Projeto/VC-BoxesProject/DataSet/FFonseca/CaixaBranca/Frame356.ply'
            elif(cb_pcd == 3):
                path = '/home/andre/Desktop/4º ANO/VC/Projeto/VC-BoxesProject/DataSet/FFonseca/CaixaBranca/Frame359.pcd'
            elif(cb_pcd == 4):
                path = '/home/andre/Desktop/4º ANO/VC/Projeto/VC-BoxesProject/DataSet/FFonseca/CaixaBranca/Frame359.ply'
            else:
                path = '/home/andre/Desktop/4º ANO/VC/Projeto/VC-BoxesProject/DataSet/FFonseca/CaixaBranca/Frame361.ply'
        else:
            path = '/home/andre/Desktop/4º ANO/VC/Projeto/VC-BoxesProject/DataSet/FFonseca/CaixaCastanha/Frame368.ply'
    else:
        if(c == 0):
            if(cb_pcd == 0):
                path = '/Users/marfre/VC-BoxesProject/DataSet/FFonseca/CaixaBranca/Frame355.ply'
            elif(cb_pcd == 1):
                path = '/Users/marfre/VC-BoxesProject/DataSet/FFonseca/CaixaBranca/Frame356.pcd'
            elif(cb_pcd == 2):
                path = '/Users/marfre/VC-BoxesProject/DataSet/FFonseca/CaixaBranca/Frame356.ply'
            elif(cb_pcd == 3):
                path = '/Users/marfre/VC-BoxesProject/DataSet/FFonseca/CaixaBranca/Frame359.pcd'
            elif(cb_pcd == 4):
                path = '/Users/marfre/VC-BoxesProject/DataSet/FFonseca/CaixaBranca/Frame359.ply'
            else:
                path = '/Users/marfre/VC-BoxesProject/DataSet/FFonseca/CaixaBranca/Frame361.ply'
        elif (c == 1):
            path = '/Users/marfre/VC-BoxesProject/DataSet/FFonseca/CaixaCastanha/Frame368.ply'
        elif (c == 2):
            path = '/Users/marfre/VC-BoxesProject/DataSet/FFonseca/VáriasCaixas/Frame371.ply'

    
    
    # pre-processing
    if(c == 2):
        planes = clusterDetection(path, verbose=True)

        print(planes)
        
    
    else:
        points = ReadPlyPoint(path)
        #points = RemoveNan(points)
        points = DownSample(points,voxel_size=0.05)
        points = RemoveNoiseStatistical(points, nb_neighbors=10, std_ratio=0.8)

        DrawResult(points)

        t0 = time.time()
        # Got the best parameters to detect all planes
        results = DetectMultiPlanes(points, min_ratio=0.001, threshold=8, iterations=2000)
        print('\nTime to detect all planes:', time.time() - t0, '\n')
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

            # Ignore ground plane
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


    plane_dims = []
    # Run through all planes and apply the bounding box
    for i in range(len(planes)):

            plane_dim = boundingBox3D(planes[i], False)
            plane_dims.append(plane_dim)

            print('\nPlane',i, 'dims:')
            print(plane_dim)


    
    planes = np.concatenate(planes, axis=0)
    colors = np.concatenate(colors, axis=0)

    
    DrawResult(planes, colors)


    # Number of planes
    n_planes = plane_counter-1

    # Final sizes of the box
    LxWxH = []

    # 3 detected planes
    if(n_planes != 2):

        print("Not done yet. Os pontos fornecedostêm mais que 2 planos detetados.")

    # 2 detected planes
    else:
        # Find and compare the biggest and smallest side (exclude the depth of bounding box - [2])
        #   For the first plane (TOP ONE)
        biggest1 = 0
        smallest1 = 0

        if(plane_dims[0][0] < plane_dims[0][1]):
            biggest1 = plane_dims[0][1]
            smallest1 = plane_dims[0][0]
        else:
            biggest1 = plane_dims[0][0]
            smallest1 = plane_dims[0][1]

        #   For the secound plane (Cascades from TOP ONE)
        biggest2 = 0
        smallest2 = 0

        if(plane_dims[1][0] < plane_dims[1][1]):
            biggest2 = plane_dims[1][1]
            smallest2 = plane_dims[1][0]
        else:
            biggest2 = plane_dims[1][0]
            smallest2 = plane_dims[1][1]


        # Find the comun side
        diff1 = abs(biggest1 - biggest2)
        diff2 = abs(smallest1 - smallest2)

        # Box is horizontal because the commun side is the biggest
        if(diff1 < diff2):

            L = (biggest1 + biggest2) / 2
            LxWxH.append(round(L))
            W = smallest1
            LxWxH.append(round(W))
            H = smallest2
            LxWxH.append(round(H))

            print("\nThe box is orientated horizontal.")
            print("Its dimensions (LxWxH) are:")
            print(LxWxH)
        # Box is vertical because the commun side is the smallest 
        else:
            '''
             Lets assum that the plane order extracted maintains.
             In case it differs, maybe the plane that have more points
            is detected first
            '''
            L = biggest1
            LxWxH.append(round(L))
            H = biggest2
            LxWxH.append(round(H))
            W = (smallest1 + smallest2) / 2
            LxWxH.append(round(W))

            print("\nThe box is orientated verticaly.")
            print("Its dimensions (LxWxH) are:")
            print(LxWxH)


    print('\nReal values CaixaCastanha (LxWxH):')
    print([320, 155, 235])
    print('Real values CaixaBranca(LxWxH):')
    print([340, 193, 85])



if __name__ == "__main__":

    '''
    p = 0 -> path do André
    p = 1 -> path do Álvaro
    '''
    p = 1

    '''
    c = 0 -> caixa branca
    c = 1 -> caixa castanha
    c = 2 -> varias caixas
    '''  
    c = 2

    '''
        PCD for white box that have the same angle of 
    extracting the point cloud

    cb_pcd = 0 -> Frame355.pcd
    cb_pcd = 1 -> Frame356.pcd
    cb_pcd = 2 -> Frame356.ply
    cb_pcd = 3 -> Frame359.pcd
    cb_pcd = 4 -> Frame359.ply
    cb_pcd = 5 -> Frame361.ply
    '''
    cb_pcd = 2
    main(p, c, cb_pcd)
    

    