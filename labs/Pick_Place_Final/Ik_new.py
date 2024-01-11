import numpy as np
def franka_IK_EE(O_T_EE_array, q7, q_actual_array):
    q_all_NAN = np.full((4, 7), np.nan)
    q_NAN = np.array([np.nan, np.nan, np.nan, np.nan, np.nan, np.nan, np.nan])
    q_all = np.copy(q_all_NAN)

    O_T_EE = np.array(O_T_EE_array).reshape((4, 4))

    d1 = 0.3330
    d3 = 0.3160
    d5 = 0.3840
    d7e = 0.2104
    a4 = 0.0825
    a7 = 0.0880

    LL24 = 0.10666225  # a4^2 + d3^2
    LL46 = 0.15426225  # a4^2 + d5^2
    L24 = 0.326591870689  # sqrt(LL24)
    L46 = 0.392762332715  # sqrt(LL46)

    thetaH46 = 1.35916951803  # atan(d5/a4)
    theta342 = 1.31542071191  # atan(d3/a4)
    theta46H = 0.211626808766  # acot(d5/a4)

    q_min = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
    q_max = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])

    if q7 <= q_min[6] or q7 >= q_max[6]:
        return q_all_NAN
    else:
        q_all[:, 6] = q7

    R_EE = O_T_EE[:3, :3]
    z_EE = O_T_EE[:3, 2]
    p_EE = O_T_EE[:3, 3]
    p_7 = p_EE - d7e * z_EE

    x_EE_6 = np.array([np.cos(q7 - np.pi/4), -np.sin(q7 - np.pi/4), 0.0])
    x_6 = R_EE @ x_EE_6
    x_6 /= np.linalg.norm(x_6)
    p_6 = p_7 - a7 * x_6

    p_2 = np.array([0.0, 0.0, d1])
    V26 = p_6 - p_2

    LL26 = np.sum(V26**2)
    L26 = np.sqrt(LL26)

    if L24 + L46 < L26 or L24 + L26 < L46 or L26 + L46 < L24:
        return q_all_NAN

    theta246 = np.arccos((LL24 + LL46 - LL26) / (2.0 * L24 * L46))
    q4 = theta246 + thetaH46 + theta342 - 2.0 * np.pi

    if q4 <= q_min[3] or q4 >= q_max[3]:
        return q_all_NAN
    else:
        q_all[:, 3] = q4

    theta462 = np.arccos((LL26 + LL46 - LL24) / (2.0 * L26 * L46))
    theta26H = theta46H + theta462
    D26 = -L26 * np.cos(theta26H)

    Z_6 = np.cross(z_EE, x_6)
    Y_6 = np.cross(Z_6, x_6)
    R_6 = np.column_stack((x_6, Y_6 / np.linalg.norm(Y_6), Z_6 / np.linalg.norm(Z_6)))
    V_6_62 = R_6.T @ (-V26)

    Phi6 = np.arctan2(V_6_62[1], V_6_62[0])
    Theta6 = np.arcsin(D26 / np.sqrt(V_6_62[0]**2 + V_6_62[1]**2))

    q6 = np.array([np.pi - Theta6 - Phi6, Theta6 - Phi6])

    for i in range(2):
        if q6[i] <= q_min[5]:
            q6[i] += 2.0 * np.pi
        elif q6[i] >= q_max[5]:
            q6[i] -= 2.0 * np.pi

        if q6[i] <= q_min[5] or q6[i] >= q_max[5]:
            q_all[2*i, 5] = q_NAN
            q_all[2*i + 1, 5] = q_NAN
        else:
            q_all[2*i, 5] = q6[i]
            q_all[2*i + 1, 5] = q6[i]

    if np.isnan(q_all[0, 5]) and np.isnan(q_all[2, 5]):
        return q_all_NAN

    thetaP26 = 3.0 * np.pi / 2 - theta462 - theta246 - theta342
    thetaP = np.pi - thetaP26 - theta26H
    LP6 = L26 * np.sin(thetaP26) / np.sin(thetaP)

    z_5_all = np.zeros((4, 3))
    V2P_all = np.zeros((4, 3))

    for i in range(2):
        z_6_5 = np.array([np.sin(q6[i]), np.cos(q6[i]), 0.0])
        z_5 = R_6 @ z_6_5
        V2P = p_6 - LP6 * z_5 - p_2

        z_5_all[2*i] = z_5
        z_5_all[2*i + 1] = z_5
        V2P_all[2*i] = V2P
        V2P_all[2*i + 1] = V2P

        L2P = np.linalg.norm(V2P)

        if np.abs(V2P[2] / L2P) > 0.999:
            q_all[2*i, 0] = q_actual_array[0]
            q_all[2*i, 1] = 0.0
            q_all[2*i + 1, 0] = q_actual_array[0]
            q_all[2*i + 1, 1] = 0.0
        else:
            q_all[2*i, 0] = np.arctan2(V2P[1], V2P[0])
            q_all[2*i, 1] = np.arccos(V2P[2] / L2P)
            if q_all[2*i, 0] < 0:
                q_all[2*i + 1, 0] = q_all[2*i, 0] + np.pi
            else:
                q_all[2*i + 1, 0] = q_all[2*i, 0] - np.pi
            q_all[2*i + 1, 1] = -q_all[2*i, 1]

    for i in range(4):
        if (q_all[i, 0] <= q_min[0] or q_all[i, 0] >= q_max[0]
                or q_all[i, 1] <= q_min[1] or q_all[i, 1] >= q_max[1]):
            q_all[i] = q_NAN
            continue

        z_3 = V2P_all[i] / np.linalg.norm(V2P_all[i])
        Y_3 = -np.cross(V26, V2P_all[i])
        y_3 = Y_3 / np.linalg.norm(Y_3)
        x_3 = np.cross(y_3, z_3)
        R_1 = np.array([[np.cos(q_all[i, 0]), -np.sin(q_all[i, 0]), 0.0],
                        [np.sin(q_all[i, 0]), np.cos(q_all[i, 0]), 0.0],
                        [0.0, 0.0, 1.0]])
        R_1_2 = np.array([[np.cos(q_all[i, 1]), -np.sin(q_all[i, 1]), 0.0],
                          [0.0, 0.0, 1.0],
                          [-np.sin(q_all[i, 1]), -np.cos(q_all[i, 1]), 0.0]])
        R_2 = R_1 @ R_1_2
        x_2_3 = R_2.T @ x_3
        q_all[i, 2] = np.arctan2(x_2_3[2], x_2_3[0])

        if q_all[i, 2] <= q_min[2] or q_all[i, 2] >= q_max[2]:
            q_all[i] = q_NAN
            continue

        VH4 = p_2 + d3 * z_3 + a4 * x_3 - p_6 + d5 * z_5_all[i]
        R_5_6 = np.array([[np.cos(q_all[i, 5]), -np.sin(q_all[i, 5]), 0.0],
                          [0.0, 0.0, -1.0],
                          [np.sin(q_all[i, 5]), np.cos(q_all[i, 5]), 0.0]])
        R_5 = R_6 @ R_5_6.T
        V_5_H4 = R_5.T @ VH4

        q_all[i, 4] = -np.arctan2(V_5_H4[1], V_5_H4[0])
        if q_all[i, 4] <= q_min[4] or q_all[i, 4] >= q_max[4]:
            q_all[i] = q_NAN
            continue

    return q_all