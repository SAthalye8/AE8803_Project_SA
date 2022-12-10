# -*- coding: utf-8 -*-
"""
Created on Sat Dec 10 03:49:59 2022

@author: SA
"""

# =============================================================================
# ECE 6563 PROJECT; DECENTRALIZED SOLUTIONS FOR NAVIGATION
# =============================================================================

# Original author: Meng Guo. Code adapted for project.

import sys
import os

import matplotlib
import matplotlib.pyplot as pyplot
from matplotlib.path import Path
import matplotlib.patches as patches
from matplotlib.patches import Polygon
import matplotlib.cm as cmx
import matplotlib.colors as colors

import copy
import numpy

from math import cos, sin, tan, atan2, asin, sqrt, ceil, floor
from math import pi as PI

import cv2
import imageio

pyplot.close("all")
# =============================================================================
# Visualization functions
# =============================================================================
def visualize_traj_dynamic(ws_model, X, U, goal, time = None, name=None):
    figure = pyplot.figure()
    ax = figure.add_subplot(1,1,1)
    cmap = get_cmap(len(X))
    # Plot obstacles
    for hole in ws_model['circular_obstacles']:
        srec = matplotlib.patches.Rectangle(
                (hole[0]-hole[2], hole[1]-hole[2]),
                2*hole[2], 2*hole[2],
                facecolor= 'red',
                fill = True,
                alpha=1)
        ax.add_patch(srec)
    # Plot traj
    for i in range(0,len(X)):
        robot = matplotlib.patches.Circle(
            (X[i][0],X[i][1]),
            radius = ws_model['robot_radius'],
            facecolor=cmap(i),
            edgecolor='black',
            linewidth=1.0,
            ls='solid',
            alpha=1,
            zorder=2)
        ax.add_patch(robot)
        # Plot velocity
        ax.arrow(X[i][0], X[i][1], U[i][0], U[i][1], head_width=0.05, head_length=0.1, fc=cmap(i), ec=cmap(i))
        ax.text(X[i][0]-0.1, X[i][1]-0.1, r'$%s$' %i, fontsize=15, fontweight = 'bold',zorder=3)
        ax.plot([goal[i][0]], [goal[i][1]], '*', color=cmap(i), markersize =15,linewidth=3.0)
    if time:
        ax.text(2,5.5,'$t=%.1f s$' %time,
                fontsize=20, fontweight ='bold')                
    ax.set_aspect('equal')
    ax.set_xlim(-1.0, 6.0)
    ax.set_ylim(-1.0, 6.0)
    ax.set_xlabel(r'$x (m)$')
    ax.set_ylabel(r'$y (m)$')
    ax.grid(True)
    if name:
        pyplot.savefig(name, dpi = 200)
    pyplot.cla()
    pyplot.close(figure)
    return figure

def get_cmap(N):
    '''Returns a function that maps each index in 0, 1, ... N-1 to a distinct RGB color.'''
    color_norm  = colors.Normalize(vmin=0, vmax=N-1)
    scalar_map = cmx.ScalarMappable(norm=color_norm, cmap='hsv') 
    def map_index_to_rgb_color(index):
        return scalar_map.to_rgba(index)
    return map_index_to_rgb_color    

# =============================================================================
# # Control Algorithm Function
# =============================================================================
# Euclidian distance in 2-D
def distance(pos1, pos2):
    return sqrt((pos1[0]-pos2[0])**2+(pos1[1]-pos2[1])**2)+0.001

# Compute best velicty given desired velocity and current velocity
def Vel_opt(X, V_des, V_current, ws_model, nav_mode):
    ROB_RAD = ws_model['robot_radius']+0.1
    V_opt = list(V_current)    
    for i in range(len(X)):
        vA = [V_current[i][0], V_current[i][1]]
        pA = [X[i][0], X[i][1]]
        RVO_BA_all = []
        for j in range(len(X)):
            if i!=j:
                vB = [V_current[j][0], V_current[j][1]]
                pB = [X[j][0], X[j][1]]
                if nav_mode == "RVO":    
                    transl_vB_vA = [pA[0]+0.5*(vB[0]+vA[0]), pA[1]+0.5*(vB[1]+vA[1])]
                if nav_mode == "VO":
                    transl_vB_vA = [pA[0]+vB[0], pA[1]+vB[1]]
                dist_BA = distance(pA, pB)
                theta_BA = atan2(pB[1]-pA[1], pB[0]-pA[0])
                if 2*ROB_RAD > dist_BA:
                    dist_BA = 2*ROB_RAD
                theta_BAort = asin(2*ROB_RAD/dist_BA)
                theta_ort_left = theta_BA+theta_BAort
                bound_left = [cos(theta_ort_left), sin(theta_ort_left)]
                theta_ort_right = theta_BA-theta_BAort
                bound_right = [cos(theta_ort_right), sin(theta_ort_right)]
                if nav_mode == "HRVO":
                    dist_dif = distance([0.5*(vB[0]-vA[0]),0.5*(vB[1]-vA[1])],[0,0])
                    transl_vB_vA = [pA[0]+vB[0]+cos(theta_ort_left)*dist_dif, pA[1]+vB[1]+sin(theta_ort_left)*dist_dif]
                RVO_BA = [transl_vB_vA, bound_left, bound_right, dist_BA, 2*ROB_RAD]
                RVO_BA_all.append(RVO_BA)                
        for hole in ws_model['circular_obstacles']:
            vB = [0, 0]
            pB = hole[0:2]
            transl_vB_vA = [pA[0]+vB[0], pA[1]+vB[1]]
            dist_BA = distance(pA, pB)
            theta_BA = atan2(pB[1]-pA[1], pB[0]-pA[0])
            # Over-approximation of square to circular
            OVER_APPROX_C2S = 1.5
            rad = hole[2]*OVER_APPROX_C2S
            if (rad+ROB_RAD) > dist_BA:
                dist_BA = rad+ROB_RAD
            theta_BAort = asin((rad+ROB_RAD)/dist_BA)
            theta_ort_left = theta_BA+theta_BAort
            bound_left = [cos(theta_ort_left), sin(theta_ort_left)]
            theta_ort_right = theta_BA-theta_BAort
            bound_right = [cos(theta_ort_right), sin(theta_ort_right)]
            RVO_BA = [transl_vB_vA, bound_left, bound_right, dist_BA, rad+ROB_RAD]
            RVO_BA_all.append(RVO_BA)
        vA_post = intersect(pA, V_des[i], RVO_BA_all)
        V_opt[i] = vA_post[:]
    return V_opt


def intersect(pA, vA, RVO_BA_all):
    norm_v = distance(vA, [0, 0])
    suitable_V = []
    unsuitable_V = []
    for theta in numpy.arange(0, 2*PI, 0.1):
        for rad in numpy.arange(0.02, norm_v+0.02, norm_v/5.0):
            new_v = [rad*cos(theta), rad*sin(theta)]
            suit = True
            for RVO_BA in RVO_BA_all:
                p_0 = RVO_BA[0]
                left = RVO_BA[1]
                right = RVO_BA[2]
                dif = [new_v[0]+pA[0]-p_0[0], new_v[1]+pA[1]-p_0[1]]
                theta_dif = atan2(dif[1], dif[0])
                theta_right = atan2(right[1], right[0])
                theta_left = atan2(left[1], left[0])
                if in_between(theta_right, theta_dif, theta_left):
                    suit = False
                    break
            if suit:
                suitable_V.append(new_v)
            else:
                unsuitable_V.append(new_v)                
    new_v = vA[:]
    suit = True
    for RVO_BA in RVO_BA_all:                
        p_0 = RVO_BA[0]
        left = RVO_BA[1]
        right = RVO_BA[2]
        dif = [new_v[0]+pA[0]-p_0[0], new_v[1]+pA[1]-p_0[1]]
        theta_dif = atan2(dif[1], dif[0])
        theta_right = atan2(right[1], right[0])
        theta_left = atan2(left[1], left[0])
        if in_between(theta_right, theta_dif, theta_left):
            suit = False
            break
    if suit:
        suitable_V.append(new_v)
    else:
        unsuitable_V.append(new_v)
    if suitable_V:
        vA_post = min(suitable_V, key = lambda v: distance(v, vA))
        new_v = vA_post[:]
        for RVO_BA in RVO_BA_all:
            p_0 = RVO_BA[0]
            left = RVO_BA[1]
            right = RVO_BA[2]
            dif = [new_v[0]+pA[0]-p_0[0], new_v[1]+pA[1]-p_0[1]]
            theta_dif = atan2(dif[1], dif[0])
            theta_right = atan2(right[1], right[0])
            theta_left = atan2(left[1], left[0])
    else:
        tc_V = dict()
        for unsuit_v in unsuitable_V:
            tc_V[tuple(unsuit_v)] = 0
            tc = []
            for RVO_BA in RVO_BA_all:
                p_0 = RVO_BA[0]
                left = RVO_BA[1]
                right = RVO_BA[2]
                dist = RVO_BA[3]
                rad = RVO_BA[4]
                dif = [unsuit_v[0]+pA[0]-p_0[0], unsuit_v[1]+pA[1]-p_0[1]]
                theta_dif = atan2(dif[1], dif[0])
                theta_right = atan2(right[1], right[0])
                theta_left = atan2(left[1], left[0])
                if in_between(theta_right, theta_dif, theta_left):
                    small_theta = abs(theta_dif-0.5*(theta_left+theta_right))
                    if abs(dist*sin(small_theta)) >= rad:
                        rad = abs(dist*sin(small_theta))
                    big_theta = asin(abs(dist*sin(small_theta))/rad)
                    dist_tg = abs(dist*cos(small_theta))-abs(rad*cos(big_theta))
                    if dist_tg < 0:
                        dist_tg = 0                    
                    tc_v = dist_tg/distance(dif, [0,0])
                    tc.append(tc_v)
            tc_V[tuple(unsuit_v)] = min(tc)+0.001
        WT = 0.2
        vA_post = min(unsuitable_V, key = lambda v: ((WT/tc_V[tuple(v)])+distance(v, vA)))
    return vA_post 

def in_between(theta_right, theta_dif, theta_left):
    if abs(theta_right - theta_left) <= PI:
        if theta_right <= theta_dif <= theta_left:
            return True
        else:
            return False
    else:
        if (theta_left <0) and (theta_right >0):
            theta_left += 2*PI
            if theta_dif < 0:
                theta_dif += 2*PI
            if theta_right <= theta_dif <= theta_left:
                return True
            else:
                return False
        if (theta_left >0) and (theta_right <0):
            theta_right += 2*PI
            if theta_dif < 0:
                theta_dif += 2*PI
            if theta_left <= theta_dif <= theta_right:
                return True
            else:
                return False

def compute_V_des(X, goal, V_max):
    V_des = []
    for i in range(len(X)):
        dif_x = [goal[i][k]-X[i][k] for k in range(2)]
        norm = distance(dif_x, [0, 0])
        norm_dif_x = [dif_x[k]*V_max[k]/norm for k in range(2)]
        V_des.append(norm_dif_x[:])
        if reach(X[i], goal[i], 0.1):
            V_des[i][0] = 0
            V_des[i][1] = 0
    return V_des
            
def reach(p1, p2, bound=0.5):
    if distance(p1,p2)< bound:
        return True
    else:
        return False

# Define workspace model
def get_ws_model(radius=0.2,w_obst=False):
    ws_model = dict()
    ws_model['robot_radius'] = radius
    #circular obstacles, format [x,y,rad]
    # no obstacles
    if w_obst:
        ws_model['circular_obstacles'] = [[-0.3, 2.5, 0.3], [1.5, 2.5, 0.3], [3.3, 2.5, 0.3], [5.1, 2.5, 0.3]]
    else:
        ws_model['circular_obstacles'] = []
    ws_model['boundary'] = [] 
    return ws_model

# Initialization for robot 
def init():
    # Position of [x,y]
    X = [[-0.5+1.0*i, 0.0] for i in range(7)] + [[-0.5+1.0*i, 5.0] for i in range(7)]
    # Velocity of [vx,vy]
    V = [[0,0] for i in range(len(X))]
    # Maximal velocity norm
    V_max = [1.0 for i in range(len(X))]
    # Goal of [x,y]
    goal = [[5.5-1.0*i, 5.0] for i in range(7)] + [[5.5-1.0*i, 0.0] for i in range(7)]
    return X,V,V_max,goal

#  Simulate: choose nav_mode from ["VO","RVO","HRVO"]
def simulate(nav_mode):
    ws_model = get_ws_model()   # Default: no obstacles
    X,V,V_max,goal = init()
    
    total_time = 15
    step = 0.01
    t_range = numpy.arange(0,total_time*100,1)
    print("Simulation in progress...")
    for t in t_range:
        # print(t)
        V_des = compute_V_des(X, goal, V_max)
        V = Vel_opt(X, V_des, V, ws_model,nav_mode)
        for i in range(len(X)):
            X[i][0] += V[i][0]*step
            X[i][1] += V[i][1]*step
        if t%10 == 0:
            visualize_traj_dynamic(ws_model, X, V, goal, time=t*step, name='snap%s.jpg'%str(t/10))
    print("Simulation completed.")
    # Convert to video:
    frameSize = (1280,960)
    # out = cv2.VideoWriter("output_video_%s.mp4"%(nav_mode),cv2.VideoWriter_fourcc(*'DIVX'), 15, frameSize)
    out = cv2.VideoWriter("output_video_%s.mp4"%(nav_mode),cv2.VideoWriter_fourcc(*'MP4V'), 15, frameSize)
    
    for i in range(total_time*10):
        filename = 'snap%s.jpg'%str(i*10/10)
        img = cv2.imread(filename)
        out.write(img)
        os.remove(filename)
    out.release()
    print("Video saved!")
    class TargetFormat(object):
        GIF = ".gif"
        MP4 = ".mp4"
        AVI = ".avi"
    
    def convertFile(inputpath, targetFormat):
        """Reference: http://imageio.readthedocs.io/en/latest/examples.html#convert-a-movie"""
        outputpath = os.path.splitext(inputpath)[0] + targetFormat
        print("Converting video to gif...".format(inputpath, outputpath))
    
        reader = imageio.get_reader(inputpath)
        fps = reader.get_meta_data()['fps']
    
        writer = imageio.get_writer(outputpath, fps=fps)
        for i,im in enumerate(reader):
            # sys.stdout.write("\rframe {0}".format(i))
            sys.stdout.flush()
            writer.append_data(im)
        # print("\r\nFinalizing...")
        writer.close()
        print("Gif saved!")
    convertFile("output_video_%s.mp4"%(nav_mode), TargetFormat.GIF) 
    return None

if __name__ == "__main__":
    mode = input('Please enter a number to choose navigation mode (1: "VO", 2: "RVO", 3:"HRVO"): ')
    nav_mode = {1: "VO", 2: "RVO", 3:"HRVO"}
    simulate(nav_mode[int(mode)])
       