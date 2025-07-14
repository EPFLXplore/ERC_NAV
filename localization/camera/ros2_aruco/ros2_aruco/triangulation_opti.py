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
    A = np.array(landmarks[0])
    B = np.array(landmarks[1])
    C = np.array(landmarks[2])

    #angles as seen by rover: (A to rvr to B = phi_1, B to rvr to C = phi_2, C to rvr to A = phi_3)
    phi_angles = [math.radians(angle) for angle in phi_angles]

    #position of rover estimated by EKF
    P_est = [current_pos[0] , current_pos[1]]    

    try:
        P = scp.optimize.root(func, P_est, args=(A,B,C, phi_angles), method = 'lm')
        #debug_log(f"OPTI TRIANG: P = {P}")
        P = P.x        # calculated rover position in map frame
        #debug_log(f"OPTI TRIANG: P.x = {P}")
    except Exception as e:
        debug_log(e)
        return None

    if P is not None:
        return P
    else:
        return None
