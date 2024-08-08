#!/usr/bin/env python3

import numpy as np
import pandas as pd
import scipy 

def shah(AA, BB):
    m, n = AA.shape
    n = n // 4
    A = np.zeros((9 * n, 18))
    T = np.zeros((9, 9))
    b = np.zeros((9 * n, 1))

    for i in range(n):
        Ra = AA[0:3, 4 * i:4 * i + 3]
        Rb = BB[0:3, 4 * i:4 * i + 3]
        T = T + np.kron(Rb, Ra)

    u, s, v = np.linalg.svd(T)
    u = np.round(u,decimals = 4)
    s = np.round(s,decimals = 4)
    v = np.round(v,decimals = 4)
    s = np.diag(s)
    x = v[0, :]
    y = u[:,0]
    
    
    

    X = np.reshape(x,(3,3))
    X = np.sign(np.linalg.det(X))/pow(abs(np.linalg.det(X)),1/3)*X
    X = X.T
    #SVD X
    u, s, v = np.linalg.svd(X)
    s = np.diag(s)
    v = v.T
    
    X = u @ v.T
    X = np.round(X,decimals = 4)


    #### Y
    Y = np.reshape(y,(3,3))
    Y = Y.T
    Y = np.sign(np.linalg.det(Y))/pow(abs(np.linalg.det(Y)),1/3)*Y

    u, s, v = np.linalg.svd(Y)
    s = np.diag(s)
    sign_u = np.linalg.det(u)
    if sign_u < 0:
        sign_u = -1
    elif sign_u > 0:
        sign_u = 1

    # Applico il segno
    u = u * sign_u
    v = v * sign_u
    Y = u @ v   
    Y = np.round(Y,decimals = 4)
    
    A = np.zeros((3*n,6))
    b = np.zeros((3*n,1))
    for i in range(n):
        A[3 * i:3 * i + 3, :] = np.hstack([-AA[0:3, 4 * i:4 * i + 3], np.eye(3)])
        b[3 * i:3 * i + 3, :] = AA[0:3, 4 * i + 3].reshape((3,1)) - np.kron(BB[0:3, 4 * i + 3].T, np.eye(3)) @ np.reshape(Y.T, (9, 1))
    
    t = np.linalg.lstsq(A, b, rcond=None)[0]

    X = np.vstack([np.hstack([X, t[0:3]]), [0, 0, 0, 1]])
    Y = np.vstack([np.hstack([Y, t[3:6]]), [0, 0, 0, 1]])

    return X, Y

# Caricamento del file CSV
file_in = '/home/franka/test_dual_arm/src/gc_calibration/workdir/calib_matrices.csv'
matrices = pd.read_csv(file_in, delim_whitespace=True, header=None)
matrices = np.array(matrices)

nn = 8

T_RT = matrices[0:4, :]
T_CT = matrices[4:8, :]



T_wt2 = [np.linalg.inv(T_RT[0:4, 4 * k:4 * k + 4]) for k in range(nn)] #8 matrix of shape 4x4
T_ct1 = [np.linalg.inv(T_CT[0:4, 4 * k:4 * k + 4]) for k in range(nn)]


T_wt2_n = np.concatenate(T_wt2, axis=1)
T_ct1_n = np.concatenate(T_ct1, axis=1)


X, Y = shah(T_wt2_n, T_ct1_n)

print("X:")
print(X)
print("\nY:")
print(Y)

#X = np.round(X,decimals = 4) scommentare per avere una matrice con 4 cifre significative


file_out = '/home/franka/test_dual_arm/src/gc_calibration/camera_calib_debug_dual.csv'
np.savetxt(file_out, X, delimiter=' ')

