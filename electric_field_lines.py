# -*- coding: utf-8 -*-
"""
Created on Wed Sep  8 09:31:03 2021

@author: rel80

Point charge electric field lines
"""

import matplotlib.pyplot as plt
import numpy as np
from scipy.integrate import ode as ode
from matplotlib import cm
from itertools import product
from scipy import ndimage
from skimage.morphology import medial_axis

class charge:
    def __init__(self, q, pos):
        self.q=q
        self.pos=pos
 
def E_point_charge(q, a, x, y):
    return q*(x-a[0])/((x-a[0])**2+(y-a[1])**2)**(1.5), \
        q*(y-a[1])/((x-a[0])**2+(y-a[1])**2)**(1.5)
 
def E_total(x, y, charges):
    Ex, Ey=0, 0
    for C in charges:
        E=E_point_charge(C.q, C.pos, x, y)
        Ex=Ex+E[0]
        Ey=Ey+E[1]
    return [ Ex, Ey ]

def E_dir(t, y, charges):
    Ex, Ey=E_total(y[0], y[1], charges)
    n=np.sqrt(Ex**2+Ey*Ey)
    return [Ex/n, Ey/n]

def V_point_charge(q, a, x, y):
    return q/((x-a[0])**2+(y-a[1])**2)**(0.5)

def V_total(x, y, charges):
    V=0
    for C in charges:
        Vp=V_point_charge(C.q, C.pos, x, y)
        V = V+Vp
    return V

def electrode_dims(num_el, grid_x):
    # Function that returns the location for all electrode dimensions given 
    #   num of electrodes on each side and size of square grid sides
    edge_dist = grid_x / ((num_el-1)*np.sqrt(2) + 2)
    delta_el = np.sqrt(2) * edge_dist
    return edge_dist, delta_el

def gen_electrode_loc(d_grid, d_edge, d_el, num_el):
    el_list = np.zeros((num_el*4,2))
    el_index = 0
    # top
    for i in range(num_el):
        el_list[el_index] = np.array([-d_grid/2+d_edge+d_el*i, d_grid/2])
        el_index = el_index + 1
    # right
    for i in range(num_el):
        el_list[el_index] = np.array([d_grid/2, d_grid/2-d_edge-d_el*i])
        el_index = el_index + 1
    # bottom
    for i in range(num_el):
        el_list[el_index] = np.array([d_grid/2-d_edge-d_el*i, -d_grid/2])
        el_index = el_index + 1
    # left
    for i in range(num_el):
        el_list[el_index] = np.array([-d_grid/2, -d_grid/2+d_edge+d_el*i])
        el_index = el_index + 1
        
    return el_list

# Define area and electrodes
d_grid = 50
n_el = 4
d_edge, d_el = electrode_dims(n_el, d_grid)
el_list = gen_electrode_loc(d_grid, d_edge, d_el, n_el)

# fig2, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))

for i in range(len(el_list)):
    # charges and positions
    if i != 15: # move the dipole (current source successively around the 16 electrode pairs)
        charges=[ charge(1, el_list[i]), charge(-1, el_list[i+1]) ]
    else:
        charges=[ charge(1, el_list[i]), charge(-1, el_list[0]) ]
     
    # calculate field lines
    x0, x1 = -d_grid/2, d_grid/2
    y0, y1 = -d_grid/2, d_grid/2
    R=0.01
    # loop over all charges
    xs,ys = [],[]
    for C in charges:
        # plot field lines starting in current charge
        dt=0.8*R
        if C.q<0:
            dt=-dt
        # loop over field lines starting in different directions 
        # around current charge
        for alpha in np.linspace(0, 2*np.pi*15/16, 16):
            r=ode(E_dir)
            r.set_integrator('vode')
            r.set_f_params(charges)
            x=[ C.pos[0] + np.cos(alpha)*R ]
            y=[ C.pos[1] + np.sin(alpha)*R ]
            r.set_initial_value([x[0], y[0]], 0)
            while r.successful():
                r.integrate(r.t+dt)
                x.append(r.y[0])
                y.append(r.y[1])
                hit_charge=False
                # check if field line left drwaing area or ends in some charge
                for C2 in charges:
                    if np.sqrt((r.y[0]-C2.pos[0])**2+(r.y[1]-C2.pos[1])**2)<R:
                        hit_charge=True
                if hit_charge or (not (x0<r.y[0] and r.y[0]<x1)) or \
                        (not (y0<r.y[1] and r.y[1]<y1)):
                    break
            xs.append(x)
            ys.append(y)
            
            
    # calculate electric potential
    vvs = []
    xxs = []
    yys = []
    numcalcv = 300
    for xx,yy in product(np.linspace(x0,x1,numcalcv),np.linspace(y0,y1,numcalcv)):
        xxs.append(xx)
        yys.append(yy)
        vvs.append(V_total(xx,yy,charges))
    xxs = np.array(xxs)
    yys = np.array(yys)
    vvs = np.array(vvs)
    
    # Find equipotential lines at each electrode
    el_pots = np.empty(0)
    for el in el_list:
        # Find V for el[i,j]...
        for i in range(len(xxs)):
            # find x index
            if abs(xxs[i] - el[0]) < 0.167/2 and abs(yys[i] - el[1]) < 0.167/2:
                el_pots = np.append(el_pots, vvs[i])
    print("Electrode potentials:",el_pots)
    
    iso_line  = np.empty((0,2))
    vvs_bin = np.zeros((300,300))
    for el_pot in el_pots:
        for i in range(len(vvs)):
            xi = i % numcalcv
            yi = i // numcalcv
            vvs_bin[xi][yi] = abs(vvs[i] - el_pot) < 0.001
            if abs(vvs[i] - el_pot) < 0.001:
                iso_line = np.append(iso_line,np.array([[xxs[i],yys[i]]]),axis=0)
    skel, distance = medial_axis(vvs_bin, return_distance=True)
        
    fig2, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))

    # Plot bin isoblob and isoline 
    ax1.imshow(vvs_bin+skel, cmap=plt.cm.gray, interpolation='nearest')

    # plot electrode isopotential lines
    iso_line_t = np.transpose(iso_line)
    ax2.plot(iso_line_t[0],iso_line_t[1], "b.")

    # plot field line
    # for x, y in zip(xs,ys):
    #     ax2.plot(x, y, color="k")
        
    # plot electrode points
    for el in el_list:
        ax2.plot(el[0], el[1], 'ko', ms=4)
    
    # plot point charges
    for C in charges:
        if C.q>0:
            ax2.plot(C.pos[0], C.pos[1], 'ro', ms=8*np.sqrt(C.q))
        if C.q<0:
            ax2.plot(C.pos[0], C.pos[1], 'bo', ms=8*np.sqrt(-C.q))
    
    # plot electric potential
    ax2.tricontour(xxs,yys,vvs,300,colors="0.3") # tricontour likely uses a marching squares algorithm
    # ax2.tricontourf(xxs,yys,vvs,100,cmap=cm.jet)
    # ax2.xlabel('$x$')
    # ax2.ylabel('$y$')
    # ax2.xlim(x0, x1)
    # ax2.ylim(y0, y1)
    # ax2.axes().set_aspect('equal','datalim')
    plt.show()

cbar = plt.colorbar()
# cbar.set_ticks([-2,-1.5,-1,-0.5,0,0.5,1,1.5,2])
cbar.set_label("Electric Potential")
plt.savefig('electric_force_lines_1.png',dpi=250,bbox_inches="tight",pad_inches=0.02)
