from functions import *
import random
import time
import math

from ClusterDetection import *

def main(p, c, cc_pcd, cb_pcd, mc_pcd):


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
                path = '/home/andre/Desktop/4º ANO/VC/Projeto/VC-BoxesProject/DataSet/FFonseca/CaixaBranca/Frame357.ply'
            elif(cb_pcd == 4):
                path = '/home/andre/Desktop/4º ANO/VC/Projeto/VC-BoxesProject/DataSet/FFonseca/CaixaBranca/Frame358.pcd'
            elif(cb_pcd == 5):
                path = '/home/andre/Desktop/4º ANO/VC/Projeto/VC-BoxesProject/DataSet/FFonseca/CaixaBranca/Frame359.pcd'
            elif(cb_pcd == 6):
                path = '/home/andre/Desktop/4º ANO/VC/Projeto/VC-BoxesProject/DataSet/FFonseca/CaixaBranca/Frame359.ply'
            elif(cb_pcd == 7):
                path = '/home/andre/Desktop/4º ANO/VC/Projeto/VC-BoxesProject/DataSet/FFonseca/CaixaBranca/Frame360.pcd'
            else:
                path = '/home/andre/Desktop/4º ANO/VC/Projeto/VC-BoxesProject/DataSet/FFonseca/CaixaBranca/Frame361.ply'

        elif (c == 1):
            if(cc_pcd == 0):
                path = '/home/andre/Desktop/4º ANO/VC/Projeto/VC-BoxesProject/DataSet/FFonseca/CaixaCastanha/Frame362.pcd'
            elif(cc_pcd == 1):
                path = '/home/andre/Desktop/4º ANO/VC/Projeto/VC-BoxesProject/DataSet/FFonseca/CaixaCastanha/Frame364.ply'
            elif(cc_pcd == 2):
                path = '/home/andre/Desktop/4º ANO/VC/Projeto/VC-BoxesProject/DataSet/FFonseca/CaixaCastanha/Frame365.ply'
            elif(cc_pcd == 3):
                path = '/home/andre/Desktop/4º ANO/VC/Projeto/VC-BoxesProject/DataSet/FFonseca/CaixaCastanha/Frame366.ply'
            elif(cc_pcd == 4):
                path = '/home/andre/Desktop/4º ANO/VC/Projeto/VC-BoxesProject/DataSet/FFonseca/CaixaCastanha/Frame367.ply'
            else:
                path = '/home/andre/Desktop/4º ANO/VC/Projeto/VC-BoxesProject/DataSet/FFonseca/CaixaCastanha/Frame368.ply'

        else:
            if(mc_pcd == 0):
                path = '/home/andre/Desktop/4º ANO/VC/Projeto/VC-BoxesProject/DataSet/FFonseca/VáriasCaixas/Frame371.ply'
            elif(mc_pcd == 1):
                path = '/home/andre/Desktop/4º ANO/VC/Projeto/VC-BoxesProject/DataSet/FFonseca/VáriasCaixas/Frame372.ply'
            elif(mc_pcd == 2):
                path = '/home/andre/Desktop/4º ANO/VC/Projeto/VC-BoxesProject/DataSet/FFonseca/VáriasCaixas/Frame373.ply'
            elif(mc_pcd == 3):
                path = '/home/andre/Desktop/4º ANO/VC/Projeto/VC-BoxesProject/DataSet/FFonseca/VáriasCaixas/Frame374.ply'
            elif(mc_pcd == 4):
                path = '/home/andre/Desktop/4º ANO/VC/Projeto/VC-BoxesProject/DataSet/FFonseca/VáriasCaixas/Frame377.ply'
            else:
                path = '/home/andre/Desktop/4º ANO/VC/Projeto/VC-BoxesProject/DataSet/FFonseca/VáriasCaixas/Frame378.ply'
#'/Users/marfre/VC-BoxesProject/DataSet/FFonseca/CaixaBranca/Frame355.ply'
    else:
        if(c == 0):
            if(cb_pcd == 0):
                path = '/Users/marfre/VC-BoxesProject/DataSet/FFonseca/CaixaBranca/Frame355.pcd'
            elif(cb_pcd == 1):
                path = '/Users/marfre/VC-BoxesProject/DataSet/FFonseca/CaixaBranca/Frame356.pcd'
            elif(cb_pcd == 2):
                path = '/Users/marfre/VC-BoxesProject/DataSet/FFonseca/CaixaBranca/Frame356.ply'
            elif(cb_pcd == 3):
                path = '/Users/marfre/VC-BoxesProject/DataSet/FFonseca/CaixaBranca/Frame357.ply'
            elif(cb_pcd == 4):
                path = '/Users/marfre/VC-BoxesProject/DataSet/FFonseca/CaixaBranca/Frame358.pcd'
            elif(cb_pcd == 5):
                path = '/Users/marfre/VC-BoxesProject/DataSet/FFonseca/CaixaBranca/Frame359.pcd'
            elif(cb_pcd == 6):
                path = '/Users/marfre/VC-BoxesProject/DataSet/FFonseca/CaixaBranca/Frame359.ply'
            elif(cb_pcd == 7):
                path = '/Users/marfre/VC-BoxesProject/DataSet/FFonseca/CaixaBranca/Frame360.pcd'
            else:
                path = '/Users/marfre/VC-BoxesProject/DataSet/FFonseca/CaixaBranca/Frame361.ply'

        elif (c == 1):
            if(cc_pcd == 0):
                path = '/Users/marfre/VC-BoxesProject/DataSet/FFonseca/CaixaCastanha/Frame362.pcd'
            elif(cc_pcd == 1):
                path = '/Users/marfre/VC-BoxesProject/DataSet/FFonseca/CaixaCastanha/Frame364.ply'
            elif(cc_pcd == 2):
                path = '/Users/marfre/VC-BoxesProject/DataSet/FFonseca/CaixaCastanha/Frame365.ply'
            elif(cc_pcd == 3):
                path = '/Users/marfre/VC-BoxesProject/DataSet/FFonseca/CaixaCastanha/Frame366.ply'
            elif(cc_pcd == 4):
                path = '/Users/marfre/VC-BoxesProject/DataSet/FFonseca/CaixaCastanha/Frame367.ply'
            else:
                path = '/Users/marfre/VC-BoxesProject/DataSet/FFonseca/CaixaCastanha/Frame368.ply'

        else:
            if(mc_pcd == 0):
                path = '/Users/marfre/VC-BoxesProject/DataSet/FFonseca/VáriasCaixas/Frame371.ply'
            elif(mc_pcd == 1):
                path = '/Users/marfre/VC-BoxesProject/DataSet/FFonseca/VáriasCaixas/Frame372.ply'
            elif(mc_pcd == 2):
                path = '/Users/marfre/VC-BoxesProject/DataSet/FFonseca/VáriasCaixas/Frame373.ply'
            elif(mc_pcd == 3):
                path = '/Users/marfre/VC-BoxesProject/DataSet/FFonseca/VáriasCaixas/Frame374.ply'
            elif(mc_pcd == 4):
                path = '/Users/marfre/VC-BoxesProject/DataSet/FFonseca/VáriasCaixas/Frame377.ply'
            else:
                path = '/Users/marfre/VC-BoxesProject/DataSet/FFonseca/VáriasCaixas/Frame378.ply'

    
    
    # pre-processing
    if(c == 2):

        # Has the plane coordinates
        planes_points = clusterDetection(path, verbose=True)

        # Variable to count number of planes
        plane_counter = 0

        planes = []
        colors = []

        for plane in planes_points:

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

                #DrawResult(plane, color)

            else:
                continue



        plane_dims = []
        # Run through all planes and apply the bounding box
        for i in range(len(planes)):

                plane_dim = boundingBox3D(planes[i], False)
                plane_dims.append(plane_dim)

                print('\nPlane',i, 'dims:')
                print(plane_dim)


        print("\n\n\n")


        # Vector with boxes dimisions
        boxes_diminsions = []
        boxes_diminsions.append([375, 340, 220, 0])
        boxes_diminsions.append([320, 235, 155, 1])
        boxes_diminsions.append([260, 215, 105, 2])
        boxes_diminsions.append([280, 200, 170, 3])
        boxes_diminsions.append([345, 190, 85, 4])
        print("\n Reference boxes dimisions:")
        for i in range(len(boxes_diminsions)):

            print("Reference box number ", i, " - ", boxes_diminsions[i])

        print("\n")

        # Index for planes used
        i_idx = -1
        j_idx = -1
        # Calibration parameter, used when comparing the differences between values
        cal_par = 19
        # Lets see the diferencies between all biggests and all smallests
        for i in plane_dims:
            i = sorted(i,reverse=True)
            i_idx = i_idx +1
            j_idx = -1
            for j in plane_dims:
                j = sorted(j,reverse=True)
                j_idx = j_idx + 1

                # If i == j diff will allways be equal to 0
                if (i[0] == j[0] and i[1] == j[1] and i[2] == j[2]):
                    continue

                # Now is possible to get the distance diff
                else:

                    # Differences between all measures of boxes
                    diff_big = abs(i[0] - j[0])
                    diff_small = abs(i[1] - j[1])

                    # Check if it i vertical or horizontal
                    # bigger side have the less difference
                    if(diff_big < diff_small):

                        # Check if this side is a candidate
                        for bd in boxes_diminsions:

                            L = (i[0] + j[0]) / 2
                            real_diff_L = abs(L - bd[0])

                            # Check length
                            if(real_diff_L < cal_par):

                                real_diff_W = abs(i[1] - bd[1])
                                # Check width
                                if(real_diff_W < cal_par):

                                    real_diff_H = abs(j[1] - bd[2])
                                    # Check hieght
                                    if(real_diff_H < cal_par):

                                        # Candidate was found
                                        LxWxH = []

                                        L = (i[0] + j[0]) / 2
                                        LxWxH.append(round(L))
                                        W = i[1]
                                        LxWxH.append(round(W))
                                        H = j[1]
                                        LxWxH.append(round(H))

                                        if(W < H):

                                            print("\nThe box is orientated Verticaly.")
                                            print("Its dimensions (LxWxH) are:")
                                            print(LxWxH)
                                            print("Represents box with Reference sizes: ", bd[3])
                                            print("The plane used are: ", i_idx, " and ", j_idx)
                                        else:
                                            print("\nThe box is orientated Horizontaly.")
                                            print("Its dimensions (LxWxH) are:")
                                            print(LxWxH)
                                            print("Represents box with Reference sizes: ", bd[3])
                                            print("The plane used are: ", i_idx, " and ", j_idx)


                                        # Lets see the planes that were chosen
                                        p = []
                                        p.append(planes[i_idx])
                                        p.append(planes[j_idx])
                                        p = np.concatenate(p, axis=0)
                                        DrawResult(p)


                    else:

                        # Check if this side is a candidate
                        for bd in boxes_diminsions:

                            W = (i[1] + j[1]) / 2
                            real_diff_W = abs(W - bd[1])

                            # Check width
                            if(real_diff_W < cal_par):

                                real_diff_L = abs(i[0] - bd[0])

                                # Check length
                                if(real_diff_L < cal_par):

                                    real_diff_H = abs(j[0] - bd[2])

                                    # Check hieght
                                    if(real_diff_H < cal_par):

                                        # Candidate was found
                                        LxWxH = []

                                        L = i[0]
                                        LxWxH.append(round(L))
                                        W = (i[1] + j[1]) / 2
                                        LxWxH.append(round(W))
                                        H = j[0]
                                        LxWxH.append(round(H))

                                        if(L < H):

                                            print("\nThe box is orientated Verticaly.")
                                            print("Its dimensions (LxWxH) are:")
                                            print(LxWxH)
                                            print("Represents box with Reference sizes: ", bd[3])
                                            print("The plane used are: ", i_idx, " and ", j_idx)
                                        else:
                                            print("\nThe box is orientated Horizontaly.")
                                            print("Its dimensions (LxWxH) are:")
                                            print(LxWxH)
                                            print("Represents box with Reference sizes: ", bd[3])
                                            print("The plane used are: ", i_idx, " and ", j_idx)

                                        # Lets see the planes that were chosen
                                        p = []
                                        p.append(planes[i_idx])
                                        p.append(planes[j_idx])
                                        p = np.concatenate(p, axis=0)
                                        DrawResult(p)





################################################################################
        
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
        #dictionary = {}

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

                plane_dim = boundingBox3D(planes[i], True)
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

            print("\nA better point cloud is needed or more then 2 planes were detected or a better calibration is needed.")
            exit()

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

            # commun side is the biggest
            if(diff1 < diff2):

                L = (biggest1 + biggest2) / 2
                LxWxH.append(round(L))
                W = smallest1
                LxWxH.append(round(W))
                H = smallest2
                LxWxH.append(round(H))

                if(W < H):

                    print("\nThe box is orientated Verticaly.")
                    print("Its dimensions (LxWxH) are:")
                    print(LxWxH)
                else:
                    print("\nThe box is orientated Horizontaly.")
                    print("Its dimensions (LxWxH) are:")
                    print(LxWxH)

            # commun side is the smallest 
            else:
                '''
                 Lets assum that the plane order extracted maintains.
                 In case it differs, maybe the plane that have more points
                is detected first
                '''
                L = biggest1
                LxWxH.append(round(L))
                W = (smallest1 + smallest2) / 2
                LxWxH.append(round(W))
                H = biggest2
                LxWxH.append(round(H))

                if(L < H):

                    print("\nThe box is orientated Verticaly.")
                    print("Its dimensions (LxWxH) are:")
                    print(LxWxH)
                else:
                    print("\nThe box is orientated Horizontaly.")
                    print("Its dimensions (LxWxH) are:")
                    print(LxWxH)


        print('\nReal values CaixaCastanha (LxWxH):')
        print([320, 235, 155])
        print('Real values CaixaBranca(LxWxH):')
        print([340, 193, 85])

################################################################################

if __name__ == "__main__":

    '''
    p = 0 -> path do André
    p = 1 -> path do Álvaro
    '''
    p = 0

    '''
    c = 0 -> caixa branca
    c = 1 -> caixa castanha
    c = 2 -> varias caixas
    '''  
    c = 2

    '''
        PCD for brown box that have the same angle of 
    extracting the point cloud

    cc_pcd = 0 -> Frame362.pcd (estima bem as dimensões e a orientação)
    cc_pcd = 1 -> Frame364.ply (Point cloud não dá para o nosso algoritmo as vezes. Quando dá estima bem as dimensões e a orientação)
    cc_pcd = 2 -> Frame365.ply (Point cloud não dá)
    cc_pcd = 3 -> Frame366.ply (Point cloud não dá)
    cc_pcd = 4 -> Frame367.ply (Point cloud não dá)
    cc_pcd = 5 -> Frame368.ply (estima bem as dimensões e a orientação)
    '''
    cc_pcd = 0

    '''
        PCD for white box that have the same angle of 
    extracting the point cloud

    cb_pcd = 0 -> Frame355.pcd (estima bem as dimensões e a orientação)
    cb_pcd = 1 -> Frame356.pcd (estima bem as dimensões e a orientação)
    cb_pcd = 2 -> Frame356.ply (estima bem as dimensões e a orientação)
    cb_pcd = 3 -> Frame357.ply (estima bem as dimensões e a orientação)
    cb_pcd = 4 -> Frame358.pcd (estima mal as dimensões, mas bem a orientação)
    cb_pcd = 5 -> Frame359.pcd (estima bem as dimensões, mas mal a orientação)
    cb_pcd = 6 -> Frame359.ply (Point cloud não dá)
    cb_pcd = 7 -> Frame360.pcd (estima bem as dimensoes, mas mal a orientação)
    cb_pcd = 8 -> Frame361.ply (estima bem as dimensoes, mas mal a orientação)
    '''
    cb_pcd = 7

    '''
        PCD for multiple boxes

    mc_pcd = 0 -> Frame371.ply
    mc_pcd = 1 -> Frame372.ply
    mc_pcd = 2 -> Frame373.ply
    mc_pcd = 3 -> Frame374.ply
    mc_pcd = 4 -> Frame377.ply
    mc_pcd = 5 -> Frame378.ply
    '''
    mc_pcd = 0

    main(p, c, cc_pcd, cb_pcd, mc_pcd)
    

    