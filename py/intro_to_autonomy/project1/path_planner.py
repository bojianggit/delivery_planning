import environment
import math
import shapely as s
import random

class PathPlanner():
    def __init__(self, bounds, env, radius):
        """Initialize the path planner.
	
	Vehicle is an object representing the vehicle that is being planned for. It 		is guaranteed to have a speed field that represents the speed at which it 		travels."""
        self.bounds = bounds
        self.env = env
        self.radius = radius

    def path(self, from_region, to_region):
        """Compute a path from from_region to to_region.
        
        Returns a LineString (from Shapely) that represents the
        path. The first coordinate in the path must be the centroid of
        from_region. The last coordinate must be in to_region.

        None is returned if a path does not exist between the regions."""
        raise NotImplementedError()

class StraightLinePathPlanner(PathPlanner):
    """Everything is done in 2 seconds."""

    def __init__(self, bounds, env, radius):
        PathPlanner.__init__(self, bounds, env, radius)
    
    def path(self, from_region, to_region):
        x0, y0 = from_region.centroid.coords[0]
        x1, y1 = to_region.centroid.coords[0]
        return s.geometry.LineString([(x0, y0), (x1, y1)])
    

    
class RRTStarPathPlanner(PathPlanner):
    def __init__(self, bounds, env, radius):
        PathPlanner.__init__(self, bounds, env, radius)
    
    def path(self, from_region, to_region):
        
        # function to generate a random point within bounds
        def generate_random_point(bounds):
            random_point = (random.random()*(bounds[2]-bounds[0])+bounds[0], random.random()*(bounds[3]-bounds[1])+bounds[1])
            return random_point
         
        # function to compute distance between two points
        def dist(point1,point2):
            return math.sqrt((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)
        
        # function to generate a new point
        def generate_new_point(current_point,new,dist_limit):
            l = dist(current_point,new)
            if l<dist_limit:
                return new
            else:
                x_dist = new[0]-current_point[0]
                y_dist = new[1]-current_point[1]
                if x_dist == 0:
                    return (current_point[0], current_point[1] + y_dist/abs(y_dist)*dist_limit)
                else:
                    return (current_point[0]+dist_limit*x_dist/l, current_point[1]+dist_limit*y_dist/l)
                    
            
        # function to check collision free
        def check_collision(current_node,new_node,radius,environment):
            check = []
            for obs in environment.obstacles: 
                check_radius = dist(current_node[0],new_node[0]) + radius
                ver_dists = [dist(new_node[0],ver) for ver in list(obs.exterior.coords)]
                if min(ver_dists) > check_radius + obs.length and obs.contains(s.geometry.Point(new_node[0])) == False:
                    check.append(0)
                else:
                    line = s.geometry.LineString([current_node[0], new_node[0]])
                    expanded_line = line.buffer(radius, resolution=3) 
                    if expanded_line.intersects(obs)==False:
                        check.append(0)
                    else:
                        break
            if len(check) == len(environment.obstacles):
                return True
            else:
                return False       
        
        def find_path(nodes_input,start,node):
            if start == node:
                return [start]
            
            path = [node[1],node[0]]
            seg_end = path[0]
            while seg_end != start[0]:
                # look forthe parent in nodes
                for seg_start_p in nodes_input:
                    if seg_end == seg_start_p[0]:
                        path.insert(0,seg_start_p[1])
                        break                      
                seg_end = path[0]
            return path
                        
        
        def pltp(path):
            path_length = 0.00
            path_step = len(path)
            for step_no in range(0,path_step-1):
                path_length = path_length + dist(path[step_no],path[step_no+1])
            return path_length
            
        def find_near(nodes,new_node,near_limit_n):
            X_near = []
            for x_near_p in nodes:
                #if dist(x_near_p[0], new_node[0]) < min(near_limit_n, step_limit):
                if dist(x_near_p[0], new_node[0]) < near_limit_n:
                    X_near.append(x_near_p) 
            return X_near     
        
        step_limit = 3*self.radius
        #step_limit = min( [obs.length for obs in self.env.obstacles] )*3
        iteration_limit = 100000
        start_node = (from_region.centroid.coords[0],None)
        nodes = [start_node]
        near_limit = 3*step_limit
        cost_list = dict()
        cost_list[start_node] = 0
        
        
        for ite in range(15,iteration_limit): 
            current_node = nodes[0]
           
            if ite%15 == 0:
                random_point = to_region.centroid.coords[0]
            else:
                random_point = generate_random_point(self.bounds)
            
            for each in nodes:
                if dist(each[0],random_point) < dist(current_node[0],random_point):
                    current_node = (each[0],each[1])
            new_point = generate_new_point(current_node[0],random_point,step_limit)
            new_node = (new_point,current_node[0])
               
            if check_collision(current_node,new_node,self.radius,self.env) == True:
                #near_limit_n = (near_limit*math.log10(len(nodes))/len(nodes))**(1/2)
                near_limit_n = near_limit
                X_near = find_near(nodes,new_node,near_limit_n)

                # rewire new node to a better one in the tree
                x_min = (current_node[0], current_node[1])  
                c_min = cost_list[x_min] + dist(x_min[0],new_node[0])
                
                '''for x_near in X_near:
                    if cost_list[x_near] + dist(x_near[0],new_node[0]) < c_min and check_collision(x_near,new_node,self.radius,self.env) == True:
                        x_min = x_near
                        c_min = cost_list[x_near] + dist(x_near[0],new_node[0])
                        
                new_node = (new_node[0],x_min[0])
                nodes.append(new_node)
                #path = find_path(nodes,start_node,new_node)
                #cost_list[new_node] = pltp(path)
                cost_list[new_node] = c_min'''
                
                rewire_cost = dict()
                for x_near in X_near:
                    rewire_cost[x_near] = cost_list[x_near] + dist(x_near[0],new_node[0])
                if rewire_cost == True:
                    if min(rewire_cost.values()) < c_min:
                        while rewire_cost == True:
                            x_low_cost = rewire_cost.pop('key', min(rewire_cost, key=rewire_cost.get))
                            if check_collision(x_low_cost,new_node,self.radius,self.env):
                                x_min = (x_low_cost[0],x_low_cost[1])
                                c_min = x_lost
                                break    
                new_node = (new_node[0],x_min[0])
                nodes.append(new_node)
                cost_list[new_node] = c_min
                
                # rewire nearby node to new node if better
                for x_near in X_near:   
                    if check_collision(x_near,new_node,self.radius,self.env) and cost_list[new_node]+dist(new_node[0],x_near[0]) < cost_list[x_near]:
                        nodes.remove(x_near)
                        x_add = (x_near[0],new_node[0])
                        nodes.append(x_add)
                        del cost_list[x_near]
                        cost_list[x_add] = cost_list[new_node]+dist(new_node[0],x_add[0])
                        
    
                if to_region.contains(s.geometry.Point(new_node[0])):   
                    print('finished')
                    break                          
            
        solution_path = find_path(nodes,start_node,new_node)
        LineSolution = s.geometry.LineString(solution_path)
        print(len(nodes),len(solution_path),pltp(solution_path))
     
    
        return LineSolution
