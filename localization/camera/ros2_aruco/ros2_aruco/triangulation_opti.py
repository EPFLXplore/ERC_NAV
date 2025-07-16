import math
import numpy as np
import scipy as scp
import rclpy.logging

def debug_log(msg: str):
    rclpy.logging.get_logger('triangulation debug').info(msg)

def func(P, A, B, C, phi):             #objective function, roots are found when angles formed bt calculated position match angles measured by cameras
    PA = np.array([A[0]-P[0], A[1]-P[1]])
    PB = np.array([B[0]-P[0], B[1]-P[1]])
    PC = np.array([C[0]-P[0], C[1]-P[1]])
    return np.array([np.dot(PA,PB)/(np.linalg.norm(PA)*np.linalg.norm(PB)) - math.cos(phi[0]),
                    np.dot(PB,PC)/(np.linalg.norm(PB)*np.linalg.norm(PC)) - math.cos(phi[1]),
                    np.dot(PA,PC)/(np.linalg.norm(PA)*np.linalg.norm(PC)) - math.cos(phi[2])])

def triangulate_opti(landmarks, phi_angles, current_pos):
    #inputs: landmarks in clockwise manner
    #phi_angles: corresponding angles
    #current estimated position in the map frame


    P = None
    #positions of aruco markers in rover's FoV
    OA = np.array(landmarks[0])
    OB = np.array(landmarks[1])
    OC = np.array(landmarks[2])

    #angles as seen by rover: (A to rvr to B = phi_1, B to rvr to C = phi_2, C to rvr to A = phi_3)
    phi_angles = [math.radians(angle) for angle in phi_angles]

    #position of rover estimated by EKF + previous map->odom transform
    P_est = [current_pos[0] , current_pos[1]] 
    x_pos_est = current_pos[0]
    y_pos_est = current_pos[1]
    ###ONLY DO THIS IF WE ARE INSIIIDE THE TRIANGLE !!!!!
    inside_triangle = False

    xa = landmarks[0][0]
    ya = landmarks[0][1]

    xb = landmarks[1][0]
    yb = landmarks[1][1]

    xc = landmarks[2][0]
    yc = landmarks[2][1]
    
    PA = np.array([xa - x_pos_est, ya - y_pos_est])
    PB = np.array([xb - x_pos_est, yb - y_pos_est])
    PC = np.array([xc - x_pos_est, yc - y_pos_est])

    AB = np.array([xb -xa, yb -ya])
    AC = np.array([xc - xa, yc-ya])
    BC = np.array([xc-xb, yc-yb])
    CA = np.array([xa - xc, ya - yc])
    CB = -BC
    BA = -AB

    AP = -PA
    BP = -PB
    CP = -PC

    #define basis change matrices
    M_A = np.vstack((AC,AB))
    M_B = np.vstack((BA,BC))
    M_C = np.vstack((CB,CA))

    if abs(np.linalg.det(M_A)) < 1e-7 or abs(np.linalg.det(M_B)) < 1e-7 or abs(np.linalg.det(M_C)) < 1e-7:
        debug_log(f"not a triangle")
        return None

    #calculate inverses matrices for basis change and obtain coords
    M_Ainv = np.transpose(np.linalg.inv(M_A))
    M_Binv = np.transpose(np.linalg.inv(M_B))
    M_Cinv = np.transpose(np.linalg.inv(M_C))

    #get coords in triangle bases:
    APtri = M_Ainv @ AP
    BPtri = M_Binv @ BP
    CPtri = M_Cinv @ CP
    epsilon = 0.08 #meters, within smaller scaled down triangle to not have problems at the edges

    if APtri[0]>=epsilon and APtri[1]>=epsilon and BPtri[0]>=epsilon and BPtri[1]>=epsilon and CPtri[0]>=epsilon and CPtri[1]>=epsilon:
        inside_triangle = True

    if not inside_triangle:
        debug_log(f"NOT INSIDE TRIANGLE !!!!!!!!!")

        return None

    try:
        P = scp.optimize.root(func, P_est, args=(OA,OB,OC, phi_angles), method = 'lm')
        debug_log(f"OPTI TRIANG: P = {P}")
        P = P.x        # calculated rover position in map frame
        #debug_log(f"OPTI TRIANG: P.x = {P}")
    except Exception as e:
        debug_log(e)
        return None

    if P is not None:
        return P
    else:
        return None
