import math
import numpy as np
from scipy.optimize import fsolve
import rclpy.logging

def debug_log(msg: str):
    rclpy.logging.get_logger('triangulation debug').info(msg)

def angle_finder(zeta, psi, phi_n, phi_m, l_n, l_m):
    return 2*math.pi - psi -phi_n - phi_m - zeta - math.asin(round(math.sin(zeta) * (math.sin(phi_n)/math.sin(phi_m)) * (l_m/l_n), 3))
#page 17
def angle_finder_oob(zeta, psi, phi_n, phi_m, l_n, l_m):
    return 2*math.pi - psi -phi_n - phi_m - zeta - math.pi + math.asin(round(math.sin(zeta) * (math.sin(phi_n)/math.sin(phi_m)) * (l_m/l_n), 3))

#page 16 of "intial yaw estimation" on arnos tablet
def angle_finder_corner(zeta, phi_n, phi_m, l_n, l_m, psi):
    return math.asin(round(math.sin(zeta)*(l_n/l_m)*(math.sin(phi_m)/math.sin(phi_n)), 3)) - psi + phi_n + phi_m + zeta


def triangulate(landmarks, phi_angles, current_pos):
    #in order: A, B, C oriented clockwise !
    #landmarks expects : [A, B, C]
    #phi_angles expects: [APB, BPC, CPA]

    #example:
    #landmarks = [[0.0, 0.0], [0.0, 1.0], [1.0, 0.0]]
    #phi_angles = [60.0, 60.0, 240.0]

    #check if any two landmarks are the same
    if landmarks[0] == landmarks[1] or landmarks[0] == landmarks[2] or landmarks[1] == landmarks[2]:
        debug_log(f"some landmarks are the same ! BAD")
        return None

    x_pos_est = current_pos[0]
    y_pos_est = current_pos[1]
    debug_log(f"current pos est tri: x={x_pos_est}, y={y_pos_est}")
    # x_pos_est = 1.0
    # y_pos_est = -0.5

    OC = np.array(landmarks[2])
    OB = np.array(landmarks[1])
    OA = np.array(landmarks[0])
    debug_log(f"triangle C={OC}, B={OB}, A={OA}")

    OP = None

    xa = landmarks[0][0]
    ya = landmarks[0][1]
    xb = landmarks[1][0]
    yb = landmarks[1][1]
    xc = landmarks[2][0]
    yc = landmarks[2][1]


    phi_angles = [math.radians(angle) for angle in phi_angles]
    debug_log(f"sum phi angles in degrees: {sum(phi_angles) * 180 / math.pi}")

    AC_norm = math.sqrt((landmarks[2][0] - landmarks[0][0]) ** 2 +
              (landmarks[2][1] - landmarks[0][1]) ** 2)
    AB_norm = math.sqrt((landmarks[1][0] - landmarks[0][0]) ** 2 +
              (landmarks[1][1] - landmarks[0][1]) ** 2)
    BC_norm = math.sqrt((landmarks[2][0] - landmarks[1][0]) ** 2 +
              (landmarks[2][1] - landmarks[1][1]) ** 2)
    
    # print(f"AC norm: {AC_norm}")
    # print(f"AB norm: {AB_norm}")
    # print(f"BC norm: {BC_norm}")

    
    # Calculate the angles using the law of cosines
    try:
        psi_a = math.acos((BC_norm**2-(AB_norm**2 + AC_norm**2)) / (-2 * AB_norm * AC_norm))
        psi_b = math.acos((AC_norm**2-(AB_norm**2 + BC_norm**2)) / (-2 * AB_norm * BC_norm))
        psi_c = math.acos((AB_norm**2-(AC_norm**2 + BC_norm**2)) / (-2 * AC_norm * BC_norm))
    except Exception:
        #(f"denominateur de psi_a : {-2 * AB_norm * AC_norm}, AB: {AB_norm}, AC: {AC_norm}")
        return None

    # print(f"sum psi angles in degrees: {(psi_a + psi_b + psi_c) * 180 / math.pi}\n psi_a = {psi_a* 180 / math.pi}, psi_b = {psi_b* 180 / math.pi}, psi_c = {psi_c* 180 / math.pi}")


    # [alpha_1, alpha_2, beta_1, beta_2, gamma_1, gamma_2] = MAT_INV @ [math.pi - phi_angles[0], math.pi - phi_angles[1], math.pi- phi_angles[2], psi_a, psi_b, psi_c]

    # beta_2 = fsolve(angle_finder(psi_b = psi_b, phi_1 = phi_angles[0], phi_2 = phi_angles[1], BC_norm = BC_norm, AB_norm = AB_norm), [math.pi/4.0])

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

    A_corner = False
    B_corner = False
    C_corner = False
    on_ac_out = False
    on_bc_out = False
    on_ab_out = False
    inside = False
    outside_ac = False
    outside_ab = False
    outside_bc = False

    #define basis change matrices
    M_A = np.vstack((AC,AB))
    M_B = np.vstack((BA,BC))
    M_C = np.vstack((CB,CA))

    if abs(np.linalg.det(M_A)) < 1e-7 or abs(np.linalg.det(M_B)) < 1e-7 or abs(np.linalg.det(M_C)) < 1e-7:
        debug_log(f"not a triangle")
        return None


    # print(f"MA : {M_A}")
    # print(f"MB : {M_B}")
    # print(f"MC : {M_C}")

    #calculate inverses matrices for basis change and obtain coords
    
    M_Ainv = np.transpose(np.linalg.inv(M_A))
    M_Binv = np.transpose(np.linalg.inv(M_B))
    M_Cinv = np.transpose(np.linalg.inv(M_C))


    #print(f"MAin : {M_Ainv}")
    #print(f"MBin : {M_Binv}")
    #print(f"MCin : {M_Cinv}")


    #get coords in tringle basis:

    APtri = M_Ainv @ AP
    BPtri = M_Binv @ BP
    CPtri = M_Cinv @ CP

    if APtri[0] <= 0 and APtri[1] <= 0:
        A_corner = True
        debug_log("A corner")
    
    if BPtri[0] <= 0 and BPtri[1] <= 0:
        B_corner = True
        debug_log("B corner")

    if CPtri[0] <= 0 and CPtri[1] <= 0:
        C_corner = True
        debug_log("C corner")

    if BPtri[0] < 0 and BPtri[1]>0 and not C_corner:
        outside_bc = True
        debug_log("outside bc")

    if APtri[0] > 0 and APtri[1] < 0 and not C_corner:
        outside_ac = True
        debug_log("outside ac")

    if BPtri[0] > 0 and BPtri[1] < 0 and not A_corner:
        outside_ab = True
        debug_log("outside ab")

    if APtri[0]>=0 and APtri[1]>=0 and BPtri[0]>=0 and BPtri[1]>=0 and CPtri[0]>=0 and CPtri[1]>=0:
        inside = True
        debug_log("inside")

    if not A_corner and not B_corner and not C_corner and not outside_bc and not outside_ac and not outside_ab and not inside:
        debug_log(f"cant get position relative to triangle")
        return None


    # print (f"outside_ac: {outside_ac}, outside_ab: {outside_ab}, outside_bc: {outside_bc}")
    # print (f"A_corner: {A_corner}, B_corner: {B_corner}, C_corner: {C_corner}")
    # print (f"on_ac_out: {on_ac_out}, on_bc_out: {on_bc_out}, on_ab_out: {on_ab_out}")
    # print (f"inside: {inside}")

    #print(f"Rover map --> base_link : ({x_map}, {y_map})")



    if inside: #points in triangle must be, in CLOCKWISE ORDER: A,B,C
        try:
            beta2_initial = math.pi/4.0

            beta_2 = fsolve(
                angle_finder,                     # 1) the function itself
                [beta2_initial],                  # 2) initial guess
                args=(psi_b,                      # 3) extra parameters in order
                    phi_angles[0],
                    phi_angles[1],
                    AB_norm,
                    BC_norm)
            )
            beta_2 = beta_2[0]

            beta_1 = math.pi - phi_angles[1] - beta_2 
            alpha_2 = psi_b - beta_1

            alpha_1 = math.pi - phi_angles[0] - alpha_2
            gamma_2 = psi_a - alpha_1
            gamma_1 = math.pi - phi_angles[2] - gamma_2

            #print(f"alpha_1: {alpha_1}, alpha_2: {alpha_2}, beta_1: {beta_1}, beta_2: {beta_2}, gamma_1: {gamma_1}, gamma_2: {gamma_2}")
            #print in degrees
            # print(f"alpha_1: {alpha_1 * 180 / math.pi}, alpha_2: {alpha_2 * 180 / math.pi}, beta_1: {beta_1 * 180 / math.pi}, beta_2: {beta_2 * 180 / math.pi}, gamma_1: {gamma_1 * 180 / math.pi}, gamma_2: {gamma_2 * 180 / math.pi}")
        except Exception:
            return None

        # print("inside")
        ca2 = math.cos(alpha_2)
        sa2 = math.sin(alpha_2)
        Rot_alpha2 = np.array([[ca2, -sa2],   #rotation matrix around point B in ccw direction (gives vector pointing in direction BP)
                               [sa2, ca2]])

        ca1 = math.cos(-alpha_1)
        sa1 = math.sin(-alpha_1)
        Rot_alpha1 = np.array([[ca1, -sa1],   #rotation matrix around point A in cw direction (gives vector pointing in direction AP)
                               [sa1, ca1]])
        


        cb1 = math.cos(-beta_1)
        sb1 = math.sin(-beta_1)
        Rot_beta1 = np.array([[cb1, -sb1],   #rotation matrix around point B in cw direction (gives vector pointing in direction BP)
                               [sb1, cb1]])

        cb2 = math.cos(beta_2)
        sb2 = math.sin(beta_2)
        Rot_beta2 = np.array([[cb2, -sb2],   #rotation matrix around point C in ccw direction (gives vector pointing in direction CP)
                               [sb2, cb2]])


        ABrot = Rot_alpha1 @ AB
        BArot = Rot_alpha2 @ BA

        # print(f"ABrot = {ABrot} BArot = {BArot}")

        BCrot = Rot_beta1 @ BC
        CBrot = Rot_beta2 @ CB
        # print(f"BCrot = {BCrot} CBrot = {CBrot}")

        if abs(ABrot[0]) < 1e-7 or abs(alpha_1) < 1e-7 or abs(((BArot[0]*ABrot[1])/ABrot[0] - BArot[1])) < 1e-7:
            if( abs(BCrot[0]) < 1e-7):
                num2b = xb-xc + (yc-yb)*(BCrot[0]/BCrot[1])
                den2b = CBrot[0] - (CBrot[1]*BCrot[0])/BCrot[1]

                t2b = num2b/den2b
                t1b = (yc-yb + t2b*CBrot[1])/BCrot[1]
                # print(f"BCrot x is null")

            else:
                num2b = yb-yc+(xc-xb)*(BCrot[1]/BCrot[0])
                den2b = CBrot[1] - (CBrot[0]*BCrot[1])/BCrot[0]

                t2b = num2b/den2b
                t1b = (xc-xb + t2b*CBrot[0]) / BCrot[0]

            OP = OB + t1b * BCrot
            # print(f"OP from beta (B) {OP} t1b = {t1b}, t2b = {t2b}")

        else:
            t2a = (yb-ya- ((xb-xa)*ABrot[1])/(ABrot[0])) / ((BArot[0]*ABrot[1])/ABrot[0] - BArot[1])
            t1a = (xb-xa + t2a*BArot[0])/ABrot[0]
            OP = OA + t1a * ABrot
            # print(f"OP from alpha (A) {OP}")
    
    elif outside_ab or A_corner:

        if A_corner:
            try:
                gamma1_initial = math.pi/4.0

                gamma_1 = fsolve(
                    angle_finder_corner,              
                    [gamma1_initial],                 
                    args=(phi_angles[2],
                        phi_angles[0],
                        AC_norm,
                        AB_norm, 
                        psi_a)
                )

                gamma_1 = gamma_1[0]
                gamma_2 = math.pi - gamma_1 - phi_angles[2]
                beta_2 = gamma_1 + psi_c
                beta_1 = math.pi - beta_2 - phi_angles[0] - phi_angles[2]
                alpha_2 = beta_1 - psi_b
                alpha_1 = math.pi - alpha_2 - phi_angles[0]
                

                #print(f"alpha_1: {alpha_1}, alpha_2: {alpha_2}, beta_1: {beta_1}, beta_2: {beta_2}, gamma_1: {gamma_1}, gamma_2: {gamma_2}")
                #print in degrees
                # print(f"alpha_1: {alpha_1 * 180 / math.pi}, alpha_2: {alpha_2 * 180 / math.pi}, beta_1: {beta_1 * 180 / math.pi}, beta_2: {beta_2 * 180 / math.pi}, gamma_1: {gamma_1 * 180 / math.pi}, gamma_2: {gamma_2 * 180 / math.pi}")
            except Exception:
                debug_log(f"A corner fsolve failed")
                return None

        if outside_ab:

            prod_ap_ac = np.dot(AP, AC) / (np.linalg.norm(AC)*np.linalg.norm(AP))
            estimate_opp_angle = math.acos(prod_ap_ac)
            # print(f"produit scalaire bp bc en sinus : {prod_ap_ac}, estimate OPP ANGLE: {estimate_opp_angle*180.0/math.pi}")

            if estimate_opp_angle > math.pi/2.0:
                try:
                    beta_1_init = math.acos(np.dot(BC, BP) / (np.linalg.norm(BC)*np.linalg.norm(BP)))

                    beta_1 = fsolve(
                        angle_finder_oob,              
                        [beta_1_init],                 
                        args=(psi_c,
                            phi_angles[2],
                            phi_angles[1],
                            AC_norm,
                            BC_norm)
                    )

                    beta_1 = beta_1[0]
                    alpha_2 = beta_1 - psi_b
                    alpha_1 = math.pi - phi_angles[2] - phi_angles[1] - alpha_2
                    beta_2 = math.pi - phi_angles[1] - beta_1
                    gamma_1 = psi_c - beta_2
                    gamma_2 = math.pi - phi_angles[2] - gamma_1
                    # print(f"alpha_1: {alpha_1 * 180 / math.pi}, alpha_2: {alpha_2 * 180 / math.pi}, beta_1: {beta_1 * 180 / math.pi}, beta_2: {beta_2 * 180 / math.pi}, gamma_1: {gamma_1 * 180 / math.pi}, gamma_2: {gamma_2 * 180 / math.pi}")
                except Exception:
                    debug_log(f"outside AB oob fsolve failed")
                    return None

            else:
                try:

                    beta_1_init = math.pi/4.0
                    beta_1 = fsolve(
                        angle_finder,              
                        [beta_1_init],                 
                        args=(psi_c,
                            phi_angles[2],
                            phi_angles[1],
                            AC_norm,
                            BC_norm)
                    )

                    beta_1 = beta_1[0]
                    alpha_2 = beta_1 - psi_b
                    alpha_1 = math.pi - phi_angles[2] - phi_angles[1] - alpha_2
                    beta_2 = math.pi - phi_angles[1] - beta_1
                    gamma_1 = psi_c - beta_2
                    gamma_2 = math.pi - phi_angles[2] - gamma_1
                    # print(f"alpha_1: {alpha_1 * 180 / math.pi}, alpha_2: {alpha_2 * 180 / math.pi}, beta_1: {beta_1 * 180 / math.pi}, beta_2: {beta_2 * 180 / math.pi}, gamma_1: {gamma_1 * 180 / math.pi}, gamma_2: {gamma_2 * 180 / math.pi}")
                except Exception:
                    debug_log(f"outside AB fsolve failed")
                    return None


        # print("outside AB or A corner, using beta1 and beta2")
        #we use the inner angles whose sign does not change when P goes from inside the triangle to the outside of AB
        # --> we thus use beta1, beta2

        #rot by beta_1 is negative by convention

        cb1 = math.cos(-beta_1)
        cb2 = math.cos(beta_2)
        sb1 = math.sin(-beta_1)
        sb2 = math.sin(beta_2)

        rot_beta1 = np.array([
            [cb1, -sb1],
            [sb1, cb1]
        ])


        rot_beta2 = np.array([
            [cb2, -sb2],
            [sb2, cb2]
        ])

        rotCB2 = rot_beta2 @ CB
        rotBC1 = rot_beta1 @ BC

        # print(f"rotCB : {rotCB2}")
        # print(f"rotBC: {rotBC1}")


        if(abs(rotBC1[1]) < 1e-7) or abs(rotBC1[0] - ((rotBC1[1]*rotCB2[0])/(rotCB2[1]))) < 1e-7:
            
            t1_num = (yc-yb) + (((xb - xc)*rotCB2[1])/rotCB2[0])
            t1_den = rotBC1[1] - ((rotBC1[0]*rotCB2[1])/rotCB2[0])

            t1 = t1_num/t1_den

            t2 = (xb - xc + (t1*rotBC1[0]))/(rotCB2[0])

            OP = OC + t2*rotCB2
        else:
            
            t1_num = xc - xb + (((yb - yc) * rotCB2[0])/rotCB2[1])

            t1_den = rotBC1[0] - ((rotBC1[1] * rotCB2[0])/rotCB2[1])

            t1 = t1_num/t1_den

            t2 = (yb - yc + t1*rotBC1[1])/rotCB2[1]

            OP = OB + t1 * rotBC1

    elif outside_bc or B_corner:

        if B_corner:
            try:
                alpha1_initial = math.acos(np.dot(AB, AP) / (np.linalg.norm(AB)*np.linalg.norm(AP)))
                alpha_1 = fsolve(
                    angle_finder_corner,              
                    [alpha1_initial],                 
                    args=(phi_angles[0],
                        phi_angles[1],
                        AB_norm,
                        BC_norm, 
                        psi_b)
                )

                alpha_1 = alpha_1[0]
                alpha_2 = math.pi - alpha_1 - phi_angles[0]
                gamma_2 = alpha_1 + psi_a
                gamma_1 = math.pi - gamma_2 - phi_angles[1] - phi_angles[0]
                beta_2 = gamma_1 - psi_c
                beta_1 = math.pi - beta_2 - phi_angles[1]

                #print(f"alpha_1: {alpha_1}, alpha_2: {alpha_2}, beta_1: {beta_1}, beta_2: {beta_2}, gamma_1: {gamma_1}, gamma_2: {gamma_2}")
                #print in degrees
                # print(f"alpha_1: {alpha_1 * 180 / math.pi}, alpha_2: {alpha_2 * 180 / math.pi}, beta_1: {beta_1 * 180 / math.pi}, beta_2: {beta_2 * 180 / math.pi}, gamma_1: {gamma_1 * 180 / math.pi}, gamma_2: {gamma_2 * 180 / math.pi}")
            except Exception:
                debug_log(f"B corner fsolve failed")
                return None

        if outside_bc:

            prod_bp_ba = np.dot(BP, BA) / (np.linalg.norm(BP)*np.linalg.norm(BA))
            estimate_opp_angle = math.acos(prod_bp_ba)
            # print(f"produit scalaire bp ba en cos : {prod_bp_ba}, estimate OPP ANGLE: {estimate_opp_angle*180.0/math.pi}")

            if estimate_opp_angle > math.pi/2.0:
                try:
                    gamma_1_init = math.acos(np.dot(CA, CP) / (np.linalg.norm(CA)*np.linalg.norm(CP)))

                    gamma_1 = fsolve(
                        angle_finder_oob,              
                        [gamma_1_init],                 
                        args=(psi_a,
                            phi_angles[0],
                            phi_angles[2],
                            AB_norm,
                            AC_norm)
                    )

                    gamma_1 = gamma_1[0]
                    beta_2 = gamma_1 - psi_c
                    beta_1 = math.pi - phi_angles[0] - phi_angles[2] - beta_2
                    gamma_2 = math.pi - phi_angles[2] - gamma_1
                    alpha_1 = psi_a - gamma_2
                    alpha_2 = math.pi - phi_angles[0] - alpha_1
                    # print(f"alpha_1: {alpha_1 * 180 / math.pi}, alpha_2: {alpha_2 * 180 / math.pi}, beta_1: {beta_1 * 180 / math.pi}, beta_2: {beta_2 * 180 / math.pi}, gamma_1: {gamma_1 * 180 / math.pi}, gamma_2: {gamma_2 * 180 / math.pi}")
                except Exception:
                    debug_log(f"outside BC oob fsolve failed")
                    return None

            else:
                try:
                    gamma_1_init = math.acos(np.dot(CA, CP) / (np.linalg.norm(CA)*np.linalg.norm(CP)))

                    gamma_1 = fsolve(
                        angle_finder,              
                        [gamma_1_init],                 
                        args=(psi_a,
                            phi_angles[0],
                            phi_angles[2],
                            AB_norm,
                            AC_norm)
                    )

                    gamma_1 = gamma_1[0]
                    beta_2 = gamma_1 - psi_c
                    beta_1 = math.pi - phi_angles[0] - phi_angles[2] - beta_2
                    gamma_2 = math.pi - phi_angles[2] - gamma_1
                    alpha_1 = psi_a - gamma_2
                    alpha_2 = math.pi - phi_angles[0] - alpha_1
                    # print(f"alpha_1: {alpha_1 * 180 / math.pi}, alpha_2: {alpha_2 * 180 / math.pi}, beta_1: {beta_1 * 180 / math.pi}, beta_2: {beta_2 * 180 / math.pi}, gamma_1: {gamma_1 * 180 / math.pi}, gamma_2: {gamma_2 * 180 / math.pi}")
                except Exception:
                    debug_log(f"outside BC fsolve failed")
                    return None

        cg1 = math.cos(-gamma_1)
        cg2 = math.cos(gamma_2)
        sg1 = math.sin(-gamma_1)
        sg2 = math.sin(gamma_2)

        rot_gamma1 = np.array([
            [cg1, -sg1],
            [sg1, cg1]
        ])


        rot_gamma2 = np.array([
            [cg2, -sg2],
            [sg2, cg2]
        ])

        rotAC2 = rot_gamma2 @ AC
        rotCA1 = rot_gamma1 @ CA


        if(abs(rotCA1[1]) < 1e-7) or abs(rotCA1[0] - ((rotCA1[1]*rotAC2[0])/(rotAC2[1]))) < 1e-7:
            
            t1_num = (ya-yc) + (((xc - xa)*rotAC2[1])/rotAC2[0])
            t1_den = rotCA1[1] - ((rotCA1[0]*rotAC2[1])/rotAC2[0])

            t1 = t1_num/t1_den

            t2 = (xc - xa + (t1*rotCA1[0]))/(rotAC2[0])

            OP = OA + t2*rotAC2
        else:
            
            t1_num = xa - xc + (((yc - ya) * rotAC2[0])/rotAC2[1])

            t1_den = rotCA1[0] - ((rotCA1[1] * rotAC2[0])/rotAC2[1])

            t1 = t1_num/t1_den

            t2 = (yc - ya + t1*rotCA1[1])/rotAC2[1]

            OP = OC + t1 * rotCA1



    elif outside_ac or C_corner:

        if C_corner:
            try:
                beta1_initial = math.acos(np.dot(BC, BP) / (np.linalg.norm(BC)*np.linalg.norm(BP)))
                beta_1 = fsolve(
                    angle_finder_corner,              
                    [beta1_initial],                 
                    args=(phi_angles[1],
                        phi_angles[2],
                        BC_norm,
                        AC_norm, 
                        psi_c)
                )

                beta_1 = beta_1[0]
                beta_2 = math.pi - beta_1 - phi_angles[1]
                alpha_2 = beta_1 + psi_b
                alpha_1 = math.pi - alpha_2 - phi_angles[2] - phi_angles[1]
                gamma_2 = alpha_1 - psi_a
                gamma_1 = math.pi - gamma_2 - phi_angles[2]

                #print(f"alpha_1: {alpha_1}, alpha_2: {alpha_2}, beta_1: {beta_1}, beta_2: {beta_2}, gamma_1: {gamma_1}, gamma_2: {gamma_2}")
                #print in degrees
                # print(f"alpha_1: {alpha_1 * 180 / math.pi}, alpha_2: {alpha_2 * 180 / math.pi}, beta_1: {beta_1 * 180 / math.pi}, beta_2: {beta_2 * 180 / math.pi}, gamma_1: {gamma_1 * 180 / math.pi}, gamma_2: {gamma_2 * 180 / math.pi}")
            except Exception:
                debug_log(f"C corner fsolve failed")
                return None

        if outside_ac:

            prod_cp_cb = np.dot(CP, CB) / (np.linalg.norm(CP)*np.linalg.norm(CB))
            estimate_opp_angle = math.acos(prod_cp_cb)
            # print(f"produit scalaire cp cp en cos : {prod_cp_cb}, estimate OPP ANGLE: {estimate_opp_angle*180.0/math.pi}")

            if estimate_opp_angle > math.pi/2.0:
                try:
                    alpha_1_init = math.acos(np.dot(AB, AP) / (np.linalg.norm(AB)*np.linalg.norm(AP)))

                    alpha_1 = fsolve(
                        angle_finder_oob,              
                        [alpha_1_init],                 
                        args=(psi_b,
                            phi_angles[1],
                            phi_angles[0],
                            BC_norm,
                            AB_norm)
                    )

                    alpha_1 = alpha_1[0]
                    gamma_2 = alpha_1 - psi_a
                    gamma_1 = math.pi - phi_angles[1] - phi_angles[0] - gamma_2
                    alpha_2 = math.pi - phi_angles[0] - alpha_1
                    beta_1 = psi_b - alpha_2
                    beta_2 = math.pi - phi_angles[1] - beta_1
                    # print(f"alpha_1: {alpha_1 * 180 / math.pi}, alpha_2: {alpha_2 * 180 / math.pi}, beta_1: {beta_1 * 180 / math.pi}, beta_2: {beta_2 * 180 / math.pi}, gamma_1: {gamma_1 * 180 / math.pi}, gamma_2: {gamma_2 * 180 / math.pi}")
                except Exception:
                    debug_log(f"outside AC oob fsolve failed")
                    return None

            else:
                try:
                    alpha_1_init = math.acos(np.dot(AB, AP) / (np.linalg.norm(AB)*np.linalg.norm(AP)))

                    alpha_1 = fsolve(
                        angle_finder,              
                        [alpha_1_init],                 
                        args=(psi_b,
                            phi_angles[1],
                            phi_angles[0],
                            BC_norm,
                            AB_norm)
                    )

                    alpha_1 = alpha_1[0]
                    gamma_2 = alpha_1 - psi_a
                    gamma_1 = math.pi - phi_angles[1] - phi_angles[0] - gamma_2
                    alpha_2 = math.pi - phi_angles[0] - alpha_1
                    beta_1 = psi_b - alpha_2
                    beta_2 = math.pi - phi_angles[1] - beta_1
                    # print(f"alpha_1: {alpha_1 * 180 / math.pi}, alpha_2: {alpha_2 * 180 / math.pi}, beta_1: {beta_1 * 180 / math.pi}, beta_2: {beta_2 * 180 / math.pi}, gamma_1: {gamma_1 * 180 / math.pi}, gamma_2: {gamma_2 * 180 / math.pi}")

                except Exception:
                    debug_log(f"outside AC fsolve failed")
                    return None


        ca1 = math.cos(-alpha_1)
        ca2 = math.cos(alpha_2)
        sa1 = math.sin(-alpha_1)
        sa2 = math.sin(alpha_2)

        rot_alpha1 = np.array([
            [ca1, -sa1],
            [sa1, ca1]
        ])


        rot_alpha2 = np.array([
            [ca2, -sa2],
            [sa2, ca2]
        ])

        rotBA2 = rot_alpha2 @ BA
        rotAB1 = rot_alpha1 @ AB


        if(abs(rotAB1[1]) < 1e-7) or abs(rotAB1[0] - ((rotAB1[1]*rotBA2[0])/(rotBA2[1]))) < 1e-7:
            
            t1_num = (yb-ya) + (((xa - xb)*rotBA2[1])/rotBA2[0])
            t1_den = rotAB1[1] - ((rotAB1[0]*rotBA2[1])/rotBA2[0])

            t1 = t1_num/t1_den

            t2 = (xa - xb + (t1*rotAB1[0]))/(rotBA2[0])

            OP = OB + t2*rotBA2
        else:
            
            t1_num = xb - xa + (((ya - yb) * rotBA2[0])/rotBA2[1])

            t1_den = rotAB1[0] - ((rotAB1[1] * rotBA2[0])/rotBA2[1])

            t1 = t1_num/t1_den

            t2 = (ya - yb + t1*rotAB1[1])/rotBA2[1]

            OP = OA + t1 * rotAB1


    if OP is not None:
        debug_log(f"triangulation found: {OP[0], OP[1]}")
        return OP
    else:
        debug_log("triangulation failed")
        return None



