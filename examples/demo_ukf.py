# -*- coding: utf-8 -*-
import numpy as np
from minidemoKalmanFilter.filter import LKF, EKF, UKF, gauss2

class DYN:
    """Dynamics Model example.
    Two-dimensional linear uniform motion.
    """
    
    def __init__(self, x, dt):
        """ initial vallue and time-step """
        self.x = x     # [Px, Py, Vx, Vy]
        self.dt = dt
        
    def next(self, u):
        x, y, xd, yd = self.x
        self.x = np.array(
                     [x + xd*self.dt,
                      y + yd*self.dt,
                      xd,
                      yd]
                 )
        
    def state(self):
        return self.x
        
#
# SIMULATION
#
def f_sys(x, u, dt):
    """ dynamics : x = f(x, u) """
    px, py, vx, vy = x
    return np.array([px + vx*dt, py + vy*dt, vx, vy])
    
def h_obs(x):
    """ observation : z = h(x) """
    # gives exact position.
    #   observation noise should be added separately.
    return np.array([x[0], x[1]])  
    
def sim_loop(dyn, kf, dt, R):
    """ simulation loop """
    rr = [] # results
    t = 0.0
    for j in range(144):
        # observation
        y = h_obs(dyn.state()) + gauss2([0, 0], R)
        kf.update(y)
        
        # record status.  # Time [s], Position [m], Velocity [m/s]
        r = (
            [t]                      # Time
            + list(dyn.x)            # True(model)  Px,  Py,  Vx,  Vy 
            + list(kf.x.flat)        # Estimated   <Px>,<Py>,<Vx>,<Vy>
            + list(y)                # Observation {Px},{Py}          
            + [kf.k[0,0], kf.k[2,0]] # Kalman Gain
            + list(kf.p.flat)        # Covariance
        )
        rr.append(r)
        
        # predict
        u = np.zeros(4)
        kf.predict(u)        # with control input 'u'.
        
        # update dynamics model
        dyn.next(u)          # with control input 'u'.
        t = t + dt
        
    return rr

def build_kf(kf_type, dt, w_pos, w_vel, w_obs):
    """ utility : create a filter object. """
    Q = np.diag([w_pos, w_pos, w_vel, w_vel]) # process noise
    R = np.diag([w_obs, w_obs])               # observation noise (for filter)
    
    if kf_type == 'LKF':
        A = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0,0,1,0], [0,0,0,1]])
        B = np.eye(A.shape[0])
        C = np.array([[1,0,0,0], [0,1,0,0]])
        kf = LKF(A, B, C, Q, R)
        
    elif kf_type == 'EKF':
        d = np.ones(Q.shape[0])*1e-6 # delta for jacobi.
        kf = EKF(f_sys, h_obs, Q, R, dt, d)
        
    elif kf_type == 'UKF':
        kf = UKF(f_sys, h_obs, Q, R, dt)
        
    return kf, R
    
def proc_simulate(kf_type, w_pos, w_vel, w_obs, c_pos, c_vel):
    """ do simulation.
    kf_type       - type of kalman filter ('LKF', 'EKF', 'UKF').
    w_pos, w_vel  - process noise in position and velocity.
    w_obs         - observation noise.
    c_pos, c_vel  - initial covariance in position and velocity.
    """
    
    # time step
    dt = 600.0                                         # [s]
    
    # true model
    xt = np.array([10.0, 10.0, 0.1, 0.0])              # initial state (px,py,vx,vy)
    dyn = DYN(xt, dt)                                  # build model.
    
    # build a state estimation filter 'kf'.
    kf, R = build_kf(kf_type, dt, w_pos, w_vel, w_obs) # build filter
    x = np.array([0.0, 0.0, 0.0, 0.0])                 # initial state for estimate.
    p = np.diag([c_pos, c_pos, c_vel, c_vel])          # initial covariance.
    kf.set_init(x, p)                                  # set initials
    
    # loop
    # observation noise 'R' is shared with simulation and a filter.
    rr = sim_loop(dyn, kf, dt, R)
    
    return rr, kf
    
def proc_print(rr):
    """ output to console (text)"""
    tt = ( 't',
           'px', 'py', 'vx', 'vy',
           'est.<px>',  '<py>', '<vx>', '<vy>',
           'obs.[px]', '[py]',
           'Gain Kp', 'Kv'
         )
    print('#' + ' '.join([a.center(10) for a in tt]))  # title
    for r in rr:
        print(' '.join([('%10.3e'%a).center(10) for a in r]))
    
    
if __name__ == '__main__':
    import sys
    
    def main():
        # control the random seed to replicate the simulation results.
        np.random.seed(1)
        
        # select parameter(s) to evaluate/designe a filter.
        # may controlled from command arguments or GUI.
        #
        w_pos = 1e-3   # [m] process noise in position.
        w_vel = 1e-9   # [m/s]             in velocity.
        w_obs = 1.0    # [m] observation noise (in position).
        c_pos = 1.0    # [m] initial covariance in position.
        c_vel = 1e-5   # [m/s]                  in velocity.
        
        #
        # do-simulation and output
        #
        if len(sys.argv) == 1:
            filter = 'UKF'
        else:
            filter = sys.argv[1]
        
        if not filter in ['LKF', 'EKF', 'UKF']:
            print('usage : %s [<filters>]' % sys.argv[0])
            print('<filters>  LKF or EKF or UKF.  (default UKF)')
            sys.exit()
        rr, kf = proc_simulate(filter, w_pos, w_vel, w_obs, c_pos, c_vel)
        proc_print(rr)
        print('# ', kf.name)
        
    main()
