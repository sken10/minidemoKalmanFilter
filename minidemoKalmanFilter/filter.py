# -*- coding: utf-8 -*-
""" minimul Kalman Filter package

Copyright (c) 2015 Kenichi SHIRAKAWA
This is licensed under MIT license.
"""
import numpy as np

def gauss2(m, p):
    return np.random.multivariate_normal(m, p)
    
def _jacobi(f, x, d):
    """ Approximate jacobian (using forward differences). """
    dd = np.diag(d)
    f0 = f(x)
    jt = [(f(x + dd[j]) - f0)/d[j] for j in range(len(x))]
    return np.array(jt).transpose()
    
class LKF:
    """ Linear Kalman Filter """
    
    def __init__(self, A, B, C, Q, R):
        self.name = 'LKF'
        self.A = A    # System matrix
        self.B = B    #   x = A x + B u
        self.C = C    # Mesurement matrix
        self.Q = Q    # process noise
        self.R = R    # observation noise
        
    def set_init(self, x, p):
        self.x = x    # initial state
        self.p = p    # initial covariance
        
    def predict(self, u):
        """ predict with control input 'u' """
        self.x = np.dot(self.A, self.x) + np.dot(self.B, u)
        self.p = self.Q + np.dot(self.A, np.dot(self.p, self.A.T))
        
    def update(self, y):
        """ update with mesurement 'y' """
        o_c = y - np.dot(self.C, self.x)
        s = self.R + np.dot(self.C, np.dot(self.p, self.C.T))
        k = np.dot(self.p, np.dot(self.C.T, np.linalg.inv(s)))
        self.x = self.x + np.dot(k, o_c)
        self.p = self.p - np.dot(k, np.dot(self.C, self.p))
        self.k = k # last gain (for reference)
        
class EKF:
    """Extended Kalman Filter """
    
    def __init__(self, f_sys, h_obs, Q, R, dt, dx):
        self.name = 'EKF'
        self.f_sys = f_sys # system function
        self.h_obs = h_obs # observation function
        self.Q = Q         # process noise
        self.R = R         # observation noise
        self.dt = dt       # time step
        self.dx = dx       # delta-x for jacobian.
        
    def set_init(self, x, p):
        self.x = x # initial state
        self.p = p # initial covariance
        
    def predict(self, u):
        """ predict with control input 'u' """
        def f_udt(x):
            return self.f_sys(x, u, self.dt)
        self.x = f_udt(self.x)              # also computed in _jacobi().
        F = _jacobi(f_udt, self.x, self.dx) # may better for previous state ?
        self.p = self.Q + np.dot(F, np.dot(self.p, F.T))
        
    def update(self, y):
        """ update with mesurement 'y' """
        o_c = y - self.h_obs(self.x)
        H = _jacobi(self.h_obs, self.x, self.dx)
        s = self.R + np.dot(H, np.dot(self.p, H.T))
        k = np.dot(self.p, np.dot(H.T, np.linalg.inv(s)))
        self.x = self.x + np.dot(k, o_c)
        self.p = self.p - np.dot(k, np.dot(H, self.p))
        self.k = k # last gain (for reference)
        
class UKF(object):
    """Unscented Kalman Filter """
    
    def __init__(self, f_sys_dt, h_obs, Q, R, dt, w0=1.0/3):
        self.name = 'UKF'
        def f_sys(x, u):
            return f_sys_dt(x, u, dt)
        self.utrn_sys = U_TRN(f_sys, Q.shape[0], Q.shape[0], w0)
        self.utrn_obs = U_TRN(h_obs, Q.shape[0], R.shape[0], w0)
        self.Q = Q    # process noise
        self.R = R    # observation noise
        
    def set_init(self, x, p):
        self.x = x.reshape(x.shape[0], 1) # initial state
        self.p = p                        # initial covariance
        
    def predict(self, u):
        """ predict with control input 'u' """
        x0, px, pyx = self.utrn_sys(self.x, self.p, u)
        self.x = x0
        self.p = px + self.Q
        
    def update(self, y):
        """ update with mesurement 'y' """
        y0, py, pyx = self.utrn_obs(self.x, self.p)
        o_c = y.reshape((y.shape[0], 1)) - y0
        ss = py + self.R
        k = np.dot(pyx.T, np.linalg.inv(ss))
        self.x = self.x + np.dot(k, o_c)
        self.p = self.p - np.dot(np.dot(k, ss), k.T)
        self.k = k # last gain (for reference)
        
class U_TRN(object):
    """Unscented Transform """
    
    def __init__(self, fun, n, m, w0):
        self.fun = fun
        self.w = np.vstack(([w0], np.tile([(1 - w0)/(2*n)], (2*n, 1))))
        self.wf = n/(1 - w0)
        
    def vec_func(self, xx, *u):
        yy = [self.fun(xx[:,j], *u) for j in range(xx.shape[1])]
        return np.array(yy).T
        
    def __call__(self, x, x_cov, *u):
        r = np.linalg.cholesky(self.wf*x_cov)
        xx = np.hstack((x, x - r, x + r))
        yy = self.vec_func(xx, *u)
        y = np.dot(yy, self.w)
        dd = yy - y
        w_dd = self.w.T*dd
        return y, np.dot(w_dd, dd.T), np.dot(w_dd, xx.T)
        
