#!/usr/bin/env python
# coding: utf-8

import numpy as np
from numpy.linalg import pinv
from time import time
import matplotlib.pyplot as plt

# lengths in m 
l0, l1, l2, l3 = 0.12225, 0.056, 0.177, 0.135952

# origin
x0, y0, z0 = 0, 0, 0

# Metric
metric = 0.01 

# Destination
#e_des = [-0.04127370851174543,-0.1709845251223143]
#e_l_des = [0.04127370851174543,-0.1709845251223143]
e_des_r = [-0,0.25]
e_des_l = [0,0.35]

# Initial theta
t_r = [0,0,0,0]
t_l = [0,0,0,0]

# Error
err_r,err_l = 10,10
threshold = 0.001
# Time period
dt = 0.001

dt0, dt1, dt2, dt3 = (0,0,0,0), (0,dt,0,0), (0,0,dt,0), (0,0,0,dt)

# linearisation val
e_lin = 0.001

# Restrictions
# Right arm
#tr_min = []



class ForwardKinematics():
    def get_right_end_eff(self,t=(0,0,0,0)):
        (t0,t1,t2,t3) = t
        x = np.cumsum([x0,0,0,0,0])
        y = np.cumsum([y0 , -(l0*np.cos(t0)) , -(l1*np.cos(t0 + t1)) ,  l2*np.sin(t0 + t1 + t2)   ,   l3*np.sin(t0 + t1 + t2 + t3)  ])
        z = np.cumsum([z0 ,  l0*np.sin(t0)   ,   l1*np.sin(t0 + t1)  , -(l2*np.cos(t0 + t1 + t2)) , -(l3*np.cos(t0 + t1 + t2 + t3)) ])
        #print("right posi",x,y,z,sep='\n', end='\n\n')
        return list(x),list(y),list(z) 

    def get_right_orientation(self,t=(0,0,0,0)):
        (t0,t1,t2,t3) = t
        xt = np.cumsum([t0,  t1-1.5708,  t2,  t3])
        yt = np.cumsum([ 0,  0 ,     0     ,   0])
        zt = np.cumsum([ 0,  0 ,     0     ,   0])
        #print("right orient",xt,yt,zt,sep='\n', end='\n\n')
        return list(xt),list(yt),list(zt) 
    
    def get_left_end_eff(self,t=(0,0,0,0)):
        (t0,t1,t2,t3) = t
        x = np.cumsum([x0,0,0,0,0])
        y = np.cumsum([y0, l0*np.cos(t0) , l1*np.cos(t0 + t1) ,  l2*np.sin(t0 + t1 + t2) ,  l3*np.sin(t0 + t1 + t2 + t3) ])
        z = np.cumsum([z0, l0*np.sin(t0) , l1*np.sin(t0 + t1) , -l2*np.cos(t0 + t1 + t2) , -l3*np.cos(t0 + t1 + t2 + t3) ])
        #print("left posi",x,y,z,sep='\n',end='\n\n')
        return list(x),list(y),list(z)    

    def get_left_orientation(self,t=(0,0,0,0)):
        (t0,t1,t2,t3) = t
        xt = np.cumsum([t0,  t1-1.5708,  t2,  t3])
        yt = np.cumsum([ 0,  0 ,     0     ,   0])
        zt = np.cumsum([ 0,  0 ,     0     ,   0])
        #print("left orient",xt,yt,zt,sep='\n', end='\n\n')

fk = ForwardKinematics()

plt.figure()
plt.ion()

while not (err_l < threshold and err_r < threshold): #if True:

    # Right arm parameters
    # Right arm e'(right e = er)
    tr0 = list(map(lambda x1,x2: x1+x2,t_r,dt0))
    tr1 = list(map(lambda x1,x2: x1+x2,t_r,dt1))
    tr2 = list(map(lambda x1,x2: x1+x2,t_r,dt2))
    tr3 = list(map(lambda x1,x2: x1+x2,t_r,dt3))
    # Left arm parameters
    # Left arm e'(right e = er)
    tl0 = list(map(lambda x1,x2: x1+x2,t_l,dt0))
    tl1 = list(map(lambda x1,x2: x1+x2,t_l,dt1))
    tl2 = list(map(lambda x1,x2: x1+x2,t_l,dt2))
    tl3 = list(map(lambda x1,x2: x1+x2,t_l,dt3))
    ############
    end_eff_right_t = fk.get_right_end_eff(t_r)
    end_eff_right_t0 = fk.get_right_end_eff(tr0)
    end_eff_right_t1 = fk.get_right_end_eff(tr1)
    end_eff_right_t2 = fk.get_right_end_eff(tr2)
    end_eff_right_t3 = fk.get_right_end_eff(tr3)
    
    end_eff_left_t = fk.get_left_end_eff(t_l)
    end_eff_left_t0 = fk.get_left_end_eff(tl0)
    end_eff_left_t1 = fk.get_left_end_eff(tl1)
    end_eff_left_t2 = fk.get_left_end_eff(tl2)
    end_eff_left_t3 = fk.get_left_end_eff(tl3)
    #############
    # Right arm del_e
    del_r_t0_x,del_r_t0_y,del_r_t0_z = [map(lambda x1,x2: x1-x2,i,j) for i,j in zip(end_eff_right_t0,end_eff_right_t)]
    del_r_t1_x,del_r_t1_y,del_r_t1_z = [map(lambda x1,x2: x1-x2,i,j) for i,j in zip(end_eff_right_t1,end_eff_right_t)]
    del_r_t2_x,del_r_t2_y,del_r_t2_z = [map(lambda x1,x2: x1-x2,i,j) for i,j in zip(end_eff_right_t2,end_eff_right_t)]
    del_r_t3_x,del_r_t3_y,del_r_t3_z = [map(lambda x1,x2: x1-x2,i,j) for i,j in zip(end_eff_right_t3,end_eff_right_t)]    
    # Jacobian elements for right arm
    del_er_dt0 = [list(map(lambda x: x/dt, list(del_r_t0_x))),list(map(lambda x: x/dt, list(del_r_t0_y))),list(map(lambda x: x/dt, list(del_r_t0_z)))]
    del_er_dt1 = [list(map(lambda x: x/dt, list(del_r_t1_x))),list(map(lambda x: x/dt, list(del_r_t1_y))),list(map(lambda x: x/dt, list(del_r_t1_z)))]
    del_er_dt2 = [list(map(lambda x: x/dt, list(del_r_t2_x))),list(map(lambda x: x/dt, list(del_r_t2_y))),list(map(lambda x: x/dt, list(del_r_t2_z)))]
    del_er_dt3 = [list(map(lambda x: x/dt, list(del_r_t3_x))),list(map(lambda x: x/dt, list(del_r_t3_y))),list(map(lambda x: x/dt, list(del_r_t3_z)))]
    # Left arm del_e
    del_l_t0_x,del_l_t0_y,del_l_t0_z = [map(lambda x1,x2: x1-x2,i,j) for i,j in zip(end_eff_left_t0,end_eff_left_t)]
    del_l_t1_x,del_l_t1_y,del_l_t1_z = [map(lambda x1,x2: x1-x2,i,j) for i,j in zip(end_eff_left_t1,end_eff_left_t)]
    del_l_t2_x,del_l_t2_y,del_l_t2_z = [map(lambda x1,x2: x1-x2,i,j) for i,j in zip(end_eff_left_t2,end_eff_left_t)]
    del_l_t3_x,del_l_t3_y,del_l_t3_z = [map(lambda x1,x2: x1-x2,i,j) for i,j in zip(end_eff_left_t3,end_eff_left_t)]
    # Jacobian elements for left arm
    del_el_dt0 = [list(map(lambda x: x/dt, list(del_l_t0_x))),list(map(lambda x: x/dt, list(del_l_t0_y))),list(map(lambda x: x/dt, list(del_l_t0_z)))]
    del_el_dt1 = [list(map(lambda x: x/dt, list(del_l_t1_x))),list(map(lambda x: x/dt, list(del_l_t1_y))),list(map(lambda x: x/dt, list(del_l_t1_z)))]
    del_el_dt2 = [list(map(lambda x: x/dt, list(del_l_t2_x))),list(map(lambda x: x/dt, list(del_l_t2_y))),list(map(lambda x: x/dt, list(del_l_t2_z)))]
    del_el_dt3 = [list(map(lambda x: x/dt, list(del_l_t3_x))),list(map(lambda x: x/dt, list(del_l_t3_y))),list(map(lambda x: x/dt, list(del_l_t3_z)))]
    # Right arm Jacobian
    j_r = [[del_er_dt0[1][4] , del_er_dt1[1][4] , del_er_dt2[1][4] , del_er_dt3[1][4]],
           [del_er_dt0[2][4] , del_er_dt1[2][4] , del_er_dt2[2][4] , del_er_dt3[2][4]]]
    # Right arm inverse Jacobian
    J_r_inv = pinv(j_r)
    # Left arm Jacobian
    j_l = [[del_el_dt0[1][4] , del_el_dt1[1][4] , del_el_dt2[1][4] , del_el_dt3[1][4]],
           [del_el_dt0[2][4] , del_el_dt1[2][4] , del_el_dt2[2][4] , del_el_dt3[2][4]]]
    # Left arm inverse Jacobian
    J_l_inv = pinv(j_l)
    ###########################
    er = [end_eff_right_t[1][4],end_eff_right_t[2][4]]
    diff_r = list(np.array(e_des_r) - np.array(er))
    #de_lin_r = list(min(max(diff_r*e_lin,-e_lin),elin))
    de_lin_r = list(np.sign(diff_r)*e_lin)
    dt_new_r = np.dot(J_r_inv,de_lin_r)
    
    el = [end_eff_left_t[1][4],end_eff_left_t[2][4]]
    diff_l = list(np.array(e_des_l) - np.array(el))
    de_lin_l = list(np.sign(diff_l)*e_lin)
    #de_lin_l = list(min(max(diff_l*e_lin,-e_lin),elin))
    dt_new_l = np.dot(J_l_inv,de_lin_l)
    #########################
    t_r = list(map(lambda x1,x2: x1+x2,t_r,dt_new_r))
    pos_cur_right = fk.get_right_end_eff(t_r)
    cur_end_eff_pos_r = [pos_cur_right[1][4],pos_cur_right[2][4]]
    err_r = np.sqrt(np.sum(((np.array(cur_end_eff_pos_r)-np.array(e_des_r))**2),axis=0))
    (_,yr,zr) = pos_cur_right

    t_l = list(map(lambda x1,x2: x1+x2,t_l,dt_new_l))
    pos_cur_left = fk.get_left_end_eff(t_l)
    cur_end_eff_pos_l = [pos_cur_left[1][4],pos_cur_left[2][4]]
    err_l = np.sqrt(np.sum(((np.array(cur_end_eff_pos_l)-np.array(e_des_l))**2),axis=0))
    (_,yl,zl) = pos_cur_left

    plt.clf()
    #rline, = ax.plot([y], [z], 'o-', lw=2)
    plt.plot(yr,zr, 'o-', lw=2)
    plt.plot(yl,zl, 'o-', lw=2)
    plt.plot(e_des_r[0],e_des_r[1], 'o')
    plt.plot(e_des_l[0],e_des_l[1], 'o')
    plt.grid(True)
    plt.axis((-0.6, 0.6, -0.6, 0.6))
    plt.pause(0.01)
    
plt.pause(10)


