from __future__ import division
import numpy as np
import sys
import csv
import os
from math import sqrt
import collada
import pywavefront
#from sets import Set

class OrienteeringProblemDefinition():
    def __init__(self):
        self.budget = 0
        self.nodes = np.array([])
        self.num_vehicles = 0
        self.start_index = 0
        self.end_index = -1
    
    def load_problem_file(self,problem_file):
        try:
            with open(problem_file,"r")  as file:
                first_line_readed = False
                for line in file:
                    if first_line_readed:
                        node_line = line.split()
                        point_to_add = np.array([float(node_line[0]),float(node_line[1]),float(node_line[2])])
                        if np.size(self.nodes,0)<1:  
                            self.nodes = np.hstack((self.nodes,point_to_add))
                        else:
                            self.nodes = np.vstack((self.nodes,point_to_add))
                        
                    else:
                        budget_line = line.split()
                        first_line_readed = True
                        self.budget = float(budget_line[0])
                        self.num_vehicles = int(budget_line[1])
                        
        except Exception as e:
            print("can not parse op problem file")
            raise


def parse_individual_value(value):
    converted = False
    if not converted:
        try: 
            value = int(value)
            converted = True
        except ValueError:
            pass
    if not converted:
        try:
            value = float(value)
            converted = True
        except ValueError:
            pass
    return value

def parse_op_log(log_file):
    log_lines = []
    delimiterVariable = ';';
    delimiterValue = ':';
    delimiterSubValue = ',';
    #not yet implemented
    delimiterSubSubValue = '|';
    if not os.path.exists(log_file):
        print("file does not exists "+log_file)
        return log_lines
    with open(log_file,"r")  as file:
        for line in file:
            var_values = line.split(delimiterVariable)
            variables = []
            values = []
            for single_var_value in var_values:
                if delimiterValue in single_var_value: 
                    splited_var_value = single_var_value.split(delimiterValue)
                    #print("splited_var_value",splited_var_value)
                    variable = splited_var_value[0]
                    value = splited_var_value[1]
                    
                   
                    if delimiterSubValue in value or delimiterSubSubValue in value:
                        #contains delimiterSubValue
                        splited_sub_value = value.split(delimiterSubValue)
                        subvalue_array = []
                        for subvalue in splited_sub_value:
                                
                            if delimiterSubSubValue in subvalue:
                                #contains delimiterSubSubValue
                                splited_sub_sub_value = subvalue.split(delimiterSubSubValue)
                                sub_subvalue_array = []
                                for sub_subvalue in splited_sub_sub_value:
                                    sub_subvalue = parse_individual_value(sub_subvalue)
                                    sub_subvalue_array.append(sub_subvalue)
                                subvalue_array.append(tuple(sub_subvalue_array))
                            else:
                                #does not contains delimiterSubSubValue
                                subvalue = parse_individual_value(subvalue)
                                subvalue_array.append(subvalue)
                        value = tuple(subvalue_array)
                    else:
                        #does not contains delimiterSubValue
                        value = parse_individual_value(value)    
                    #convert normal value ints
                        
                    
                    variables.append(variable)
                    values.append(value)
            #print("variables",variables)
            #print("values",values)
            single_log = dict(zip(variables, values))
            log_lines.append(single_log)
    #sys.exit()
    return log_lines

def load_sampled_path(sampled_path_file):
    sampled_path = None
    with open(sampled_path_file, 'rt') as csvfile:
        scv_reader = csv.reader(csvfile)
        for row in scv_reader:
            to_add = np.array([float(row[0]),float(row[1]),                   float(row[2])])
            if(sampled_path is None):
                sampled_path = to_add
            else:
                sampled_path = np.vstack((sampled_path,to_add))
    #print(sampled_path)
    return sampled_path

def getUniqueVeluesOfKey(data_to_process,key):
    unique_value_set = set([])
    for idx in range(len(data_to_process)):
        #print(data_to_process[idx][key])
        unique_value_set.add(data_to_process[idx][key])
    return list(unique_value_set)

def getDataWithKeyValue(data_to_process,key,value):
    data_subset = []
    for idx in range(len(data_to_process)):
        if data_to_process[idx][key] == value:
            data_subset.append(data_to_process[idx])
    return data_subset

def getMaxValueInData(data_to_process,key):
    max = data_to_process[0][key]
    for idx in range(1,len(data_to_process)):
        if data_to_process[idx][key] > max:
            max = data_to_process[idx][key]
    return max

def getMinValueInData(data_to_process,key):
    min = data_to_process[0][key]
    for idx in range(1,len(data_to_process)):
        if data_to_process[idx][key] < min:
            min = data_to_process[idx][key]
    return min

def getAverageValueInData(data_to_process,key):
    sum = 0
    num_tested = 0
    for idx in range(len(data_to_process)):
        num_tested +=1
        sum += data_to_process[idx][key]
    return sum / num_tested

def getStandardDeviationInData(data_to_process,key):
    average = getAverageValueInData(data_to_process,key)
    sum_squared_difference = 0
    num_tested = 0
    for idx in range(len(data_to_process)):
        num_tested +=1
        sum_squared_difference += (data_to_process[idx][key] - average)**2
    
    if num_tested <= 1:
        raise Exception('can not calc StandardDeviation for num tested '+str(num_tested))
    
    return sqrt(sum_squared_difference / (num_tested-1))

def load_csv_numbers(file):
    print("load_csv_numbers",file)
    samples = []
    with open(file, 'r') as csvfile:
        csvreader = csv.reader(csvfile, delimiter=',')
        for row in csvreader:
            #print(row)
            col = []
            for c in row:
                col.append(float(c))
            samples.append(col)
    return samples

def frange(x, y, jump):
  while x < y:
    yield x
    x += jump

class PlanningState():
    UNKNOWN = 0
    D2 = 1
    D3 = 2
    DUBINSD2 = 3  

class ObstacleProblem():
    LOADING_MAP_POINTS = 1
    LOADING_OBSTACLE = 2
    LOADING_BORDER = 3
    LOADING_CITY_POINTS = 4
    
    def __init__(self):
        self.map_points = {}
        self.city_points = {}
        self.obstacles = []
        self.border_indices = []
    
    def load_obstacle_problem_definition(self, problem_file):
        MAP_POINTS = "[MAP_POINTS]"
        MAP_OBSTACLE = "[MAP_OBSTACLE]"
        MAP_BORDER = "[MAP_BORDER]"
        CITY_POINTS = "[CITY_POINTS]"
        
        loading = None
        actual_obstacle = []
        
        with open(problem_file, 'r') as f:
            for line in f:
                
                if not line.strip():
                    if loading == ObstacleProblem.LOADING_OBSTACLE:
                        self.obstacles.append(actual_obstacle)
                        actual_obstacle = []
                    loading = None
                    continue
                
                if MAP_POINTS in line:
                    loading = ObstacleProblem.LOADING_MAP_POINTS
                    continue
                
                if CITY_POINTS in line:
                    loading = ObstacleProblem.LOADING_CITY_POINTS
                    continue
                
                if MAP_OBSTACLE in line:
                    loading = ObstacleProblem.LOADING_OBSTACLE
                    continue
                
                if MAP_BORDER in line:
                    loading = ObstacleProblem.LOADING_BORDER
                    continue
                
                if loading == ObstacleProblem.LOADING_MAP_POINTS:
                    dim_strs = line.split()
                    if len(dim_strs)==3:
                        self.map_points[int(dim_strs[0])] = [float(dim_strs[1]), float(dim_strs[2])]
                    else:
                        print("map points are not well", dim_strs,line)
                        sys.exit(1)
                if loading == ObstacleProblem.LOADING_CITY_POINTS:
                    dim_strs = line.split()
                    if len(dim_strs) == 4:
                        self.city_points[int(dim_strs[0])] = [float(dim_strs[1]), float(dim_strs[2]), float(dim_strs[3])]
                    else:
                        print("city points are not well", dim_strs)
                        sys.exit(1)
                
                if loading == ObstacleProblem.LOADING_OBSTACLE:
                    actual_obstacle.append(int(line))
                
                if loading == ObstacleProblem.LOADING_BORDER:
                    self.border_indices.append(int(line))



def load_3d_problem_definition(file):
    loading_rewards=False
    object_rewards = {}
    map_file = None
    with open(file, 'r') as f:
        for line in f:
            if '[TARGET_REWARDS]' in line:
                loading_rewards = True
                continue
            if 'MAP_FILE' in line:
                map_file_param = line.split("=")
                map_file = map_file_param[1].strip()
                print("map_file",map_file)
                problem_dir_path = os.path.dirname(file)
                map_file = os.path.join(problem_dir_path,map_file)
                print("map_file",map_file)
            if loading_rewards and not line.strip():
                loading_rewards = False
                continue
            if loading_rewards:
                parts = line.split()
                #print(parts)
                object_rewards[parts[0]] = float(parts[1])
    
    city_points = None
    obstacles = None
    if map_file is not None:
        city_points , obstacles  = load_obj_mesh_problem(map_file,object_rewards)
    else:
        print("can not load problem description",file,"no MAP_FILE")
        
    return city_points, obstacles

def get_avg_target_pos(xses,yses,zses,triangles):
    x=0
    y=0
    z=0
    num_vertices = 0
    for tri in triangles:
        for vert in tri:
            x += xses[vert]
            y += yses[vert]
            z += zses[vert]
            num_vertices += 1
    return x/float(num_vertices) , y/float(num_vertices) , z/float(num_vertices)

def load_obj_mesh_problem(filename,target_rewards):
    city_points = []
    obstacles = []
    
    print("loading colada file",filename)
    scene = pywavefront.Wavefront(filename,collect_faces=True)
    xses=[]
    yses=[]
    zses = []
    
    
    for vertex in scene.vertices:
        xses.append(vertex[0])
        yses.append(vertex[1])
        zses.append(vertex[2])
        
    
    for mesh in scene.mesh_list:
        triangles = []
        print("mesh ",mesh.name)
        
        for face in mesh.faces:
            #print("face",face)
            triangles.append(face)
    
        if "wall_" in mesh.name or "floor_" in mesh.name:
             obstacles.append([xses,yses,zses,triangles])
        
        if "target" in mesh.name:
            target_name = mesh.name.replace("target_","target.").replace("-mesh","")
            traget_reward = target_rewards[target_name]
            #print(target_rewards)
            x,y,z =  get_avg_target_pos(xses , yses , zses , triangles)
            city_points.append([x,y,z,traget_reward])
            print(target_name,"x",x,"y",y,"z",z,"reward",traget_reward)
    
        if "start" in mesh.name:
            x,y,z =  get_avg_target_pos(xses , yses , zses , triangles)
            traget_reward = 0
            city_points.append([x,y,z,traget_reward])
            print("start x",x,"y",y,"z",z,"reward",traget_reward)
        
        if "goal" in mesh.name:
            x,y,z =  get_avg_target_pos(xses , yses , zses , triangles)
            traget_reward = 0
            city_points.append([x,y,z,traget_reward])
            print("goal x",x,"y",y,"z",z,"reward",traget_reward)
    
    print("city_points len",len(city_points))
    return city_points , obstacles
    
    
def load_dae_mesh_problem(filename,target_rewards):
    city_points = []
    obstacles = []
    
    print("loading colada file",filename)
    mesh = collada.Collada(filename)
    boundgeoms = list(mesh.scene.objects('geometry'))
    
    #print(boundgeoms)
    for geomid in range(len(boundgeoms)):
        
        geom = boundgeoms[geomid]
        #print("geomid",geomid)
        #print("geom",geom.original)
        #print("geom.id",geom.original.id)
        transform = mesh.scene.nodes[geomid].transforms
        #print(geom)
        
        #print("transform",transform)
        xses = []
        yses = []
        zses = []
        triandgles = []
        
        for triset in geom.primitives():
            trilist = list(triset)
            
            for vertex in triset.vertex:
                xses.append(vertex[0])
                yses.append(vertex[1])
                zses.append(vertex[2])
                #print("v",vertex)
            for triangle in trilist:
                triangles.append(triangle.indices)
    
        #print(len(xses))
        #print(triandgles)
        if "wall_" in geom.original.id or "floor_" in geom.original.id:
            #color = [0.5,0.5,0.5,0.1]
            #edgecolor = [0.5,0.5,0.5,0.1]
            #obst_plot = ax.plot_trisurf(xses , yses, zses,triangles=triandgles, linewidth=0.2, antialiased=True,color=color,edgecolor=edgecolor,zorder=3,label='obstacles')
            obstacles.append([xses,yses,zses,triangles])
            """
            if not ploted_first_obstacle :
                ploted_first_obstacle = True
                fake2Dline = mpl.lines.Line2D([0],[0], linestyle="-",c=edgecolor, markerfacecolor=color, marker = 'o')
                legend_plots.append(fake2Dline)
                legend_text.append("\\textbf{obstacles}")
            """
            
        if "start-" in geom.original.id:
            x = mesh.scene.nodes[geomid].transforms[0].matrix[0,3]
            y = mesh.scene.nodes[geomid].transforms[0].matrix[1,3]
            z = mesh.scene.nodes[geomid].transforms[0].matrix[2,3]
            traget_reward = 0
            city_points.append([x,y,z,traget_reward])
            print("start x",x,"y",y,"z",z)
            #color = mycmap(cNorm(0))#
            #edgecolor = mycmap(cNorm(0))#
            #obst_plot = ax.plot_trisurf(xses , yses, zses,triangles=triandgles, linewidth=0.2, antialiased=True,color=color,edgecolor=edgecolor,zorder=3,label='obstacles')
            
            #plot([x],[y],[z],'ok',zorder=1000,label = 'start')
        
        if "goal-" in geom.original.id:
            x = mesh.scene.nodes[geomid].transforms[0].matrix[0,3]
            y = mesh.scene.nodes[geomid].transforms[0].matrix[1,3]
            z = mesh.scene.nodes[geomid].transforms[0].matrix[2,3]
            traget_reward = 0
            city_points.append([x,y,z,traget_reward])
            print("goal x",x,"y",y,"z",z)
            #color = mycmap(cNorm(0))#
            #edgecolor = mycmap(cNorm(0))#
            #obst_plot = ax.plot_trisurf(xses , yses, zses,triangles=triandgles, linewidth=0.2, antialiased=True,color=color,edgecolor=edgecolor,zorder=3,label='obstacles')
            
            #plot([x],[y],[z],'ok',zorder=1000,label = 'goal')
            
        if "target" in geom.original.id:
            #print(geom.original.id)
            target_name = geom.original.id.replace("target_","target.").replace("-mesh","")
            traget_reward = target_rewards[target_name]
            #print(target_rewards)
            x = mesh.scene.nodes[geomid].transforms[0].matrix[0,3]
            y = mesh.scene.nodes[geomid].transforms[0].matrix[1,3]
            z = mesh.scene.nodes[geomid].transforms[0].matrix[2,3]
            print(target_name,"x",x,"y",y,"z",z)
            city_points.append([x,y,z,traget_reward])
            #color = mycmap(cNorm(traget_reward))#
            #edgecolor = mycmap(cNorm(traget_reward))#
            #obst_plot = ax.plot_trisurf(xses , yses, zses,triangles=triandgles, linewidth=0.2, antialiased=True,color=color,edgecolor=edgecolor,zorder=3,label='obstacles')
            
            #z = 1.2
            #plot([x],[y],[z],'ok',zorder=1000,label = 'goal')
    
    return city_points , obstacles        
            
