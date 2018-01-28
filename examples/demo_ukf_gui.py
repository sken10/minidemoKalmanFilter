# -*- coding: utf-8 -*-
#  (Japanese comments included.)
#
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider
import demo_ukf

k_wvel_nom = -9  # プロセスノイズ(指標)のノミナル値
k_wobs_nom =  0  # 観測ノイズ(指標)のノミナル値
k_cvel_nom = -5  # 初期分散(指標)のノミナル値

def proc(k_wvel, k_wobs, k_cvel):
    np.random.seed(1)
    w_pos = 1e-3                   # [m] プロセスノイズ 位置 
    w_vel = math.pow(10.0, k_wvel) # [m/s] プロセスノイズ 速度
    w_obs = math.pow(10.0, k_wobs) # [m] (位置)観測ノイズ
    c_pos = 1.0                    # [m] 初期分散 位置
    c_vel = math.pow(10.0, k_cvel) # [m/s] 初期分散 速度
    rr, flt = demo_ukf.proc_simulate('UKF', w_pos, w_vel, w_obs, c_pos, c_vel)
    t = [a[0]/3600.0 for a in rr]
    p = [a[5]-a[1] for a in rr]  # EST. Err (C - true)
    o = [a[9]-a[1] for a in rr]  # OBS. Err (O - true)
    k1 = [a[11] for a in rr]  # gain
    c  = [a[13] for a in rr]  # covariance
    return t, p, o, k1, c

plt.figure(figsize=(8, 6))
plt.subplots_adjust(left=0.1, bottom=0.25)

t, p, o, k1, c = proc(k_wvel_nom, k_wobs_nom, k_cvel_nom)  # 1e-9, 1e-5

plt.subplot(311)
p_est, = plt.plot(t, p, lw=2, color='red')
p_obs, = plt.plot(t, o, 'o')
plt.legend(('Est.', 'Obs.'), loc='upper right')
plt.xlabel('Time [h]')
plt.ylabel('Position Error [m]')
plt.axis([0, 24, -4, 4])
plt.grid()

plt.subplot(312)
p_k1, = plt.plot(t, k1, lw=2, color='red')
plt.legend(('K1'))
plt.xlabel('Time [h]')
plt.ylabel('Gain')
plt.axis([0, 24, 0, 1])
plt.grid()

plt.subplot(313)
p_cov, = plt.plot(t, c, lw=2, color='red')
plt.legend(('Cov'))
plt.xlabel('Time [h]')
plt.ylabel('Cov')
plt.autoscale(axis='y')
plt.grid()

plt.annotate("UKF Demo : Touch sliders bellow to change param.",
    verticalalignment='top',
    horizontalalignment='left',
    xy=(0.05, 0.97), xycoords='figure fraction', fontsize=10)

def update(val):
    wvel = s_wvel.val
    wobs = k_wobs_nom
    cvel = s_cvel.val
    t, p, o, k1, c = proc(wvel, wobs, cvel)
    p_est.set_ydata(p)
    p_obs.set_ydata(o)
    p_k1.set_ydata(k1)
    p_cov.set_ydata(c)
    plt.autoscale(axis='y')
    plt.draw()

#
# Controls
#

ax_wvel  = plt.axes([0.25, 0.15, 0.65, 0.03])
s_wvel = Slider(ax_wvel, 'Process Noise\n(Velocity)', -12, 3, valinit=k_wvel_nom)
s_wvel.on_changed(update)

ax_cvel  = plt.axes([0.25, 0.05, 0.65, 0.03])
s_cvel = Slider(ax_cvel, 'Initial Covariance\n(Velocity)', -12, 3, valinit=k_cvel_nom)
s_cvel.on_changed(update)

plt.show()
