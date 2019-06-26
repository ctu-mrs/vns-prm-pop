#!/usr/bin/env python3

import sys, os
import random
import numpy as np

import matplotlib as mpl
if os.environ.get('DISPLAY','') == '':
    print('no display found. Using non-interactive Agg backend')
    mpl.use('Agg')
import matplotlib.pyplot as plt

from mpl_toolkits.axes_grid1 import make_axes_locatable
import shapely.geometry as geometry
from shapely.ops import cascaded_union, polygonize
import math
from matplotlib.pyplot import arrow
import dubins
this_script_path = os.path.dirname(__file__)   
path_to_utils = os.path.join(this_script_path, "utils")  
sys.path.append(path_to_utils)
import figure_utils
import orienteering_utils
from orienteering_utils import PlanningState


legend_font_size = 20
tick_font_size = 16
NUM_POINTS_TO_GEN = 16
SCATTER_SIZE = 80
FIG_HEIGHT = 7.5
SHOW_FIGURE = True

RESULT_FILE = "../sources/results/results.log"
SAMPLES_FILE = '../sources/results/final_prm_points.txt'
PATH_FILE = '../sources/results/final_prm_point_path.txt'
SAVE_TO_FIGURE = "solution_pop.png"
RESULT_FILE = os.path.join(this_script_path, RESULT_FILE)
                                                                                                                 
#use nice latex fonts if latex is installed
#figure_utils.configure_latex_fonts_latex()

data_vns_pop = orienteering_utils.parse_op_log(RESULT_FILE)
prm_samples = orienteering_utils.load_csv_numbers(SAMPLES_FILE)
prm_path = orienteering_utils.load_csv_numbers(PATH_FILE)


print("using the last results")
record = data_vns_pop[-1]
print("record", record)

planning_state = PlanningState.UNKNOWN

PROBLEM_NAME = record['NAME']
PROBLEM_NAME_FILES = {"potholes":"potholes-cell.txt",
                      "dense":"dense-cell.txt",
                      "building":"building.txt"}

problem_file = os.path.join(this_script_path, "../datasets/",PROBLEM_NAME,PROBLEM_NAME_FILES[PROBLEM_NAME])
print("problem_file",problem_file)

city_points = []
obstacles = []

if "2d" == record["PLANNING_STATE_TYPE"]:
    print("using 2d planning type")
    planning_state = PlanningState.D2
    op = orienteering_utils.ObstacleProblem()
    op.load_obstacle_problem_definition(problem_file)
    city_points = op.city_points
    map_points = op.map_points
    obstacles = op.obstacles
    border = op.border_indices

elif "3d" == record["PLANNING_STATE_TYPE"]:
    print("using 3d planning type")
    planning_state = PlanningState.D3
    city_points , obstacles = orienteering_utils.load_3d_problem_definition(problem_file)
    print("city_points",city_points)

elif "dubins2d" == record["PLANNING_STATE_TYPE"]:
    print("using dubins2d planning type")
    planning_state = PlanningState.DUBINSD2
    op.load_obstacle_problem_definition(problem_file)
    city_points = op.nodes
    map_points = op.map_points
    obstacles = op.obstacles
    border = op.border_indices
    
else:
    error("can not decide planning type")
    planning_state = PlanningState.UNKNOWN
    quit(1)

result_target_ids = record['RESULT_TARGET_IDS']
result_rewards = record['REWARDS']
result_length = record['LENGTH']
print("problem loaded")
print("result_target_ids:", result_target_ids)
print("result_rewards", result_rewards)
print("result_length", result_length)


calc_reward = 0
for target_idx in range(len(result_target_ids)):
    node = result_target_ids[target_idx]
    #print("node",node,"len city nodes",len(city_points))
    if planning_state == PlanningState.D3:
        calc_reward += city_points[node][3]
    else:
        calc_reward += city_points[node][2]
    if node >= len(city_points) or node<0:
        print("what the hell, it is not good")

print("calc_reward", calc_reward)

calc_length = 0
for i in range(1,len(prm_path)):
    prew = prm_path[i-1]
    act = prm_path[i]
    if planning_state == PlanningState.D3:
        calc_length += math.sqrt((act[0]-prew[0])**2+(act[1]-prew[1])**2+(act[2]-prew[2])**2)
    else:
        calc_length += math.sqrt((act[0]-prew[0])**2+(act[1]-prew[1])**2)
        
print("calc_length",calc_length)

mycmap = plt.cm.get_cmap('RdYlBu_r')

maxx, maxy = -sys.float_info.max,-sys.float_info.max
minx, miny = sys.float_info.max,sys.float_info.max

circle_radiuses = np.ones([len(city_points), 1])
circle_radiuses1 = np.multiply(2.0, circle_radiuses)


xses = [city_points[i][0] for i in range(len(city_points))]
yses = [city_points[i][1] for i in range(len(city_points))]
zses = [city_points[i][2] for i in range(len(city_points))]
maxx = max(xses)
minx = min(xses)
maxy = max(yses)
miny = min(yses)
maxz = max(zses)
minz = min(zses)

nodes_w_rewards = np.zeros((len(city_points), 4))
if planning_state == PlanningState.D3:
    nodes_w_rewards = np.zeros((len(city_points), 4))

for nidx in range(len(city_points)):
    nodes_w_rewards[nidx, 0] = city_points[nidx][0]
    nodes_w_rewards[nidx, 1] = city_points[nidx][1]
    nodes_w_rewards[nidx, 2] = city_points[nidx][2]
    if planning_state == PlanningState.D3:
        print("D3")
        nodes_w_rewards[nidx, 3] = city_points[nidx][3]
    
print("nodes_w_rewards",nodes_w_rewards)
if planning_state == PlanningState.D3:
    minrew = min(nodes_w_rewards[:, 3])
    maxrew = max(nodes_w_rewards[:, 3])
else:
    minrew = min(nodes_w_rewards[:, 2])
    maxrew = max(nodes_w_rewards[:, 2])

cNorm = mpl.colors.Normalize(vmin=minrew, vmax=maxrew + 0.1 * (maxrew - minrew))       
mycmapScalarMap = mpl.cm.ScalarMappable(norm=cNorm, cmap=mycmap)

print("xy dist ",math.sqrt((maxx-minx)*(maxy-miny)))
print("maxz",maxz,"minz",minz)
fig_width = FIG_HEIGHT*(maxx-minx)/float(maxy-miny)
if planning_state == PlanningState.D3:
    fig_width = 14

figsize = (fig_width*0.9,FIG_HEIGHT)
print(figsize)

if planning_state == PlanningState.D3:
    fig = plt.figure(num=None, figsize=figsize, dpi=80, facecolor='w', edgecolor='k')
    ax = fig.add_subplot(111, projection='3d')
    
else:
    fig = plt.figure(num=None, figsize=figsize, dpi=80, facecolor='w', edgecolor='k')
    ax = plt.gca()



if planning_state == PlanningState.D3:
    color = [0.5,0.5,0.5,0.1]
    edgecolor = [0.5,0.5,0.5,0.1]
    for obst in obstacles:
        obst_plot = ax.plot_trisurf(obst[0] , obst[1], obst[2],triangles=obst[3], linewidth=0.2, antialiased=True,color=color,edgecolor=edgecolor,zorder=3,label='obstacles')
else:
    plt.plot([map_points[border[index_id]][0] for index_id in range(-1,len(border))],[map_points[border[index_id]][1] for index_id in range(-1,len(border))],'-k',lw=2,zorder=8)
    
    for obst in obstacles:
        obstacles_plot = plt.plot([map_points[obst[index_id]][0] for index_id in range(-1,len(obst))],[map_points[obst[index_id]][1] for index_id in range(-1,len(obst))],'-k')
    

#circles = figure_utils.circles(nodes_w_rewards[:, 0], nodes_w_rewards[:, 1], circle_radiuses1, c=nodes_w_rewards[:, 2] , alpha=0.05, edgecolor='black', linewidth=0.9, linestyle=':')
if planning_state == PlanningState.D3:
    print("rewards",nodes_w_rewards[:, 3])
    sc = ax.scatter(nodes_w_rewards[:, 0], nodes_w_rewards[:, 1],nodes_w_rewards[:, 2], c=nodes_w_rewards[:, 3], cmap=mycmap , alpha=1.0, s=55, linewidths= 1.2, facecolor='black',edgecolor='black')
    #plt.plot(nodes_w_rewards[:, 0], nodes_w_rewards[:, 1], nodes_w_rewards[:, 2], 'ok', ms=4.0)
    prm_samples = orienteering_utils.load_csv_numbers(SAMPLES_FILE)
else:
    sc = plt.scatter(nodes_w_rewards[:, 0], nodes_w_rewards[:, 1], c=nodes_w_rewards[:, 2], cmap=mycmap , alpha=1.0, s=55,linewidths= 1.2, facecolor='black',edgecolor='black',zorder=10)
    #plt.plot(nodes_w_rewards[:, 0], nodes_w_rewards[:, 1], 'ok', ms=4.0)

"""
for node_idx in range(1, len(result_target_ids)):

    node = result_target_ids[node_idx]
    node_prew = result_target_ids[node_idx - 1]
    node_pos = [city_points[node][0], city_points[node][1], city_points[node][2]]
    node_pos_prew = [city_points[node_prew][0], city_points[node_prew][1], city_points[node_prew][2]]
    print(node_prew, '->', node, ",", node_pos_prew, '->', node_pos)
    if planning_state == PlanningState.D3:
        plt.plot([node_pos_prew[0], node_pos[0] ], [node_pos_prew[1], node_pos[1] ], [node_pos_prew[2], node_pos[2] ], '-g', lw=1.6)
    else:
        plt.plot([node_pos_prew[0], node_pos[0] ], [node_pos_prew[1], node_pos[1] ], '-g', lw=1.6)
"""

for i in range(1,len(prm_path)):
#for i in range(1,len(prm_path)):
    prew = prm_path[i-1]
    act = prm_path[i]
    if planning_state == PlanningState.D3:
        plt.plot([prew[0], act[0] ], [prew[1], act[1] ], [prew[2], act[2] ], '-g', lw=2.3)
    else:
        plt.plot([prew[0], act[0] ], [prew[1], act[1] ], '-g', lw=2.3)
        
      


figure_utils.no_axis(ax)


cbar_position = [0.20, 0.04, 0.6, 0.03]
cbar_ax = fig.add_axes(cbar_position)
cb = plt.colorbar(sc, cax=cbar_ax, orientation='horizontal')
cb.ax.tick_params(labelsize=tick_font_size)
cb.set_label('target configuration reward', labelpad=-60.0, y=0.8, fontsize=legend_font_size)


# offset = 0.08


#fig.subplots_adjust(left=-0.035, right=1.035 , top=1.07 , bottom=0.0)
if planning_state == PlanningState.D3:
    ax.set_xlim([-20,20])
    ax.set_ylim([-20,20])
    ax.set_zlim([-0,8])
    ax.view_init(30,-34)
    figure_utils.aply_orthogonal_proj()
    fig.subplots_adjust(left=-0.35, right=1.35 , top=1.23 , bottom=-0.07)
    
else:
    ax.axis('equal')
    fig.subplots_adjust(left=-0.02, right=1.02 , top=1.06 , bottom=0.03)

plt.savefig(SAVE_TO_FIGURE, dpi=300)
if SHOW_FIGURE:
    plt.show()  

