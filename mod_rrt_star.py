import copy
import numpy as np
import cv2
import time
import random
import argparse

def draw_obstacles(canvas,clr=5,unknown=False, map_flag=1):
    """
    @brief: This function goes through each node in the canvas image and checks for the
    obstacle space using the half plane equations. 
    If the node is in obstacle space, the color is changed to blue.
    :param canvas: Canvas Image
    :param unknown: Unknown Map flag
    """
    # Uncomment to use the cv2 functions to create the obstacle space
    # cv2.circle(canvas, (300,65),45,(255,0,0),-1)
    # cv2.fillPoly(canvas, pts = [np.array([[115,40],[36,65],[105,150],[80,70]])], color=(255,0,0)) #Arrow
    # cv2.fillPoly(canvas, pts = [np.array([[200,110],[235,130],[235,170],[200,190],[165,170],[165,130]])], color=(255,0,0)) #Hexagon
    
    height,width,_ = canvas.shape
    
    
    for i in range(width):
        for j in range(height):
            if map_flag == 1:
                # Map 01
                if(i<=clr) or (i>=(400-clr)) or (j<=clr) or (j>=(250-clr)):
                    canvas[j][i] = [255,0,0]

                if ((i-300)**2+(j-65)**2-((40+clr)**2))<=0:
                    canvas[j][i] = [255,0,0]
                
                if (j+(0.57*i)-213.53)>=clr and (j-(0.57*i)+5.04+clr)>=0 and (i-235-clr)<=0 and (j+(0.57*i)-305.04-clr)<=0 and (j-(0.57*i)-76.465-clr)<=0 and (i-155-clr)>=0:
                    canvas[j][i] = [255,0,0]

                if ((j+(0.316*i)-66.1483-clr)>=0 and (j+(0.857*i)-140.156-clr)<=0 and (j-(0.114*i)-55.909-clr)<=0) or ((j-(1.23*i)-23.576-clr)<=0 and (j-(3.2*i)+197.763+clr)>=0 and (j-(0.114*i)-55.909-clr)>=0):
                    canvas[j][i] = [255,0,0]

            elif map_flag == 2:
                # Map 02
                if(i<=clr) or (i>=(400-clr)) or (j<=clr) or (j>=(250-clr)):
                    canvas[j][i] = [255,0,0]

                if ((i>=118-clr) and (i<=148+clr) and (j>=clr) and (j<=63)):
                    canvas[j][i] = [255,0,0]

                if ((i>=118-clr) and (i<=148+clr) and (j>=103-clr) and (j<=147+clr)):
                    canvas[j][i] = [255,0,0]

                if ((i>=118-clr) and (i<=148+clr) and (j>=187-clr) and (j<=(250-clr))):
                    canvas[j][i] = [255,0,0]

                if ((i>=251-clr) and (i<=281+clr) and (j>=42-clr) and (j<=105+clr)):
                    canvas[j][i] = [255,0,0]
                
                if ((i>=251-clr) and (i<=281+clr) and (j>=145-clr) and (j<=208+clr)):
                    canvas[j][i] = [255,0,0]
            
    if(unknown):
        for i in range(width):
            for j in range(height):
                
                if map_flag == 1:
                    if ((i-110)**2+(j-210)**2-((35)**2))<=0:
                        canvas[j][i] = [0,255,0]
                elif map_flag == 2:
                    if ((i-70)**2+(j-190)**2-((35)**2))<=0:
                        canvas[j][i] = [0,255,0]

                    if ((i-200)**2+(j-140)**2-((20)**2))<=0:
                        canvas[j][i] = [0,255,0]
            
    return canvas

def is_obstacle(next_width,next_height,canvas,unknown=False):
    """
    @brief: This function checks if the node is present in the obstacle. 
    If the node is in obstacle space, the function returns true.
    :param canvas: Canvas Image
    :param unknown: Unknown Map flag
    """
    if(unknown):
        if canvas[int(round(next_height))][int(round(next_width))][1]==255 or canvas[int(round(next_height))][int(round(next_width))][0]==255:
            return True
        else:
            return False    
    else:
        if canvas[int(round(next_height))][int(round(next_width))][0]==255:
            # print("In obstacle")
            return True
        else:
            return False

def cost_to_goal(node,final):
    """
    @brief: This function computes the euclidean distance between the current node and the final goal node. 
    :param node: present node
    :param final: final node (Goal Node)
    """
    return np.sqrt(np.power(node[0]-final[0],2)+np.power(node[1]-final[1],2))

def compute_distance(node1,node2):
    """
    @brief: This function computes the euclidean distance between the two given nodes. Mainly used to compute
    the edge length.
    :param node1: First Node
    :param node2: Second Node
    """
    return np.sqrt(np.power(node1[0]-node2[0],2)+np.power(node1[1]-node2[1],2))

def nearest(sample, TV):
    """
    @brief: This function returns the nearest node from the previously generated vertices. 
    :param sample: sampled node
    :param TV: Tree Vertices
    """
    dist = []
    for vertex in TV:
        dist.append(compute_distance(sample,vertex))
    
    nearest_vertex = TV[dist.index(min(dist))]
    return nearest_vertex

def neighbor_nodes(sample, TV, d=10):
    """
    @brief: This function computes the nearby neighbouring nodes at a particular distance threshold from
    the sample and the tree vertices.
    :param sample: Node from which the neighbouring nodes are to be returned
    :param TV: Tree vertices
    """
    neighbors = []
    for vertex in TV:
        if compute_distance(sample,vertex) < d: 
            neighbors.append(vertex)

    return neighbors
    
def collision_free(X_nearest,X_rand,canvas,unknown=False): #Replace X_rand with X_new for steer function
    """
    @brief: This function samples the edge and checks for the validity of the edge by checking the obstacle space
    :param X_nearest: Nearest Node
    :param X_rand: Random node
    :param canvas: Map
    :unknown: Flag for unknown map
    """
    if X_rand[0] != X_nearest[0]:
        x1 = min(X_nearest[0],X_rand[0])
        if(X_nearest[0] == x1):
            for w in range(X_nearest[0],X_rand[0]+1):
                h = ((X_rand[1] - X_nearest[1])/(X_rand[0] - X_nearest[0]))*(w - X_nearest[0]) + X_nearest[1]
                if(is_obstacle(int(w),int(h),canvas,unknown)):
                    # print("Collision!")
                    return False
        else:
            for w in range(X_rand[0],X_nearest[0]+1):
                h = ((X_nearest[1] - X_rand[1])/(X_nearest[0] - X_rand[0]))*(w - X_rand[0]) + X_rand[1] 
                if(is_obstacle(int(w),int(h),canvas,unknown)):
                    # print("Collision!")
                    return False
        
    else:
        y1 = min(X_nearest[1],X_rand[1])
        if(y1 == X_nearest[1]):
            for h in range(X_nearest[1],X_rand[1]+1):
                if(is_obstacle(int(X_nearest[0]),int(h),canvas,unknown)):
                    # print("Collision!")
                    return False
        else:
            for h in range(X_rand[1],X_nearest[1]+1):
                if(is_obstacle(int(X_rand[0]),int(h),canvas,unknown)):
                    # print("Collision!")
                    return False
            
    return True

def rewire(node1,node2,node_dict,canvas,final_state):
    """
    @brief: This function rewires the edge between the nodes by checking the edge length and the cost to goal. 
    :param node1: Node 1
    :param node2: Node 2
    :param node_dict: Dictionary containing the Parent nodes and costs
    :param canvas: Map
    :param final_state: Goal Node
    """
    # print("In rewire")
    parent = []
    if collision_free(node1,node2, canvas) is True:
        if (compute_distance(node1, node2) + cost_to_goal(node1, final_state)) < cost_to_goal(node2, final_state):
            node_dict[tuple(node2)] = [node1, compute_distance(node1, node2) + cost_to_goal(node1, final_state)]
            parent = node1.copy()
    if len(parent) != 0:
        return parent
    else: 
        return node_dict[tuple(node2)][0]

def mod_rrt_star(initial_state,final_state,canvas):
    """
    @brief: This function generates the random tree for the given obstacle map.
    :param initial_state: Start Node
    :param final_state: final node (Goal Node)
    :param canvas: Map
    """
    TV = []
    TE = {}
    TV.append(final_state)
    node_dict = {}
    node_dict[tuple(final_state)] = final_state
    while True:
        width_rand = random.randint(0,canvas.shape[1]-1)
        height_rand = random.randint(0,canvas.shape[0]-1)

        X_rand = [width_rand,height_rand]
        # print("Random sample: ", X_rand)
        X_nearest = nearest(X_rand,TV)
        
        #Steer function to be implemented later for non-holonomic constraints. 
        #X_new <- Steer(X_rand, X_nearest)
        #Here X_rand is X_new
        if(collision_free(X_nearest, X_rand, canvas) is False):
            continue
        
        X_parent = X_nearest.copy()
        node_dict[tuple(X_rand)] = [X_parent, cost_to_goal(X_nearest,final_state) + compute_distance(X_nearest,X_rand)]

        X_neighbors = neighbor_nodes(X_rand, TV, 10)

        for n in X_neighbors:
            X_parent = rewire(n, X_rand, node_dict, canvas, final_state)

        TV.append(X_rand)
        TE[tuple(X_rand)] = X_parent.copy()
        # print("X_parent", X_parent)

        for n in X_neighbors:
            X_parent_temp = rewire(X_rand, n, node_dict, canvas, final_state)

            if X_parent_temp == X_rand:
                # print("Before Pop", n)
                TE.pop(tuple(n))
                TE[tuple(n)] = X_rand.copy()

        if compute_distance(X_rand,initial_state) < 5:
            print("RRT* Converged!")
            return TE, TV, X_rand

def backtrack(initial_state, final_state, edges, canvas):
    """
    @brief: This function backtracks the path from the goal node to the start node.
    :param initial_state: start node
    :param final_state: final node (Goal Node)\
    :param edges: Edges between the nodes
    """
    state = initial_state.copy()
    path = []
    while True:
        node = edges[tuple(state)]
        path.append(state)
        # cv2.line(canvas, tuple(state), tuple(node), (0, 255, 230), 3)
        if(tuple(node) == tuple(final_state)):
            path.append(final_state)
            print("Back Tracking Done!")
            break
        state = node.copy()
    return path

def path_sampling(path):
    """
    @brief: This function samples the generated path
    :param path: path from start node to the goal node
    """
    sampled_path = []
    for i in range(0,len(path)-1):
        X_rand = path[i]
        X_nearest = path[i+1]
        if X_rand[0] != X_nearest[0]:
            x1 = min(X_nearest[0],X_rand[0])
            if(X_nearest[0] == x1):
                for w in range(X_nearest[0],X_rand[0]+1):
                    h = ((X_rand[1] - X_nearest[1])/(X_rand[0] - X_nearest[0]))*(w - X_nearest[0]) + X_nearest[1]
                    sampled_path.append([int(w),int(h)])
            else:
                for w in range(X_rand[0],X_nearest[0]+1):
                    h = ((X_nearest[1] - X_rand[1])/(X_nearest[0] - X_rand[0]))*(w - X_rand[0]) + X_rand[1] 
                    sampled_path.append([int(w),int(h)])
            
        else:
            print("vertical line", X_nearest[1], X_rand[1])
            y1 = min(X_nearest[1],X_rand[1])
            print("y1 ", y1)
            if(y1 == X_nearest[1]):
                for h in range(X_nearest[1],X_rand[1]+1):
                    sampled_path.append([int(X_nearest[0]),int(h)])
            else:
                for h in range(X_rand[1],X_nearest[1]+1):
                    sampled_path.append([int(X_rand[0]),int(h)])
    return sampled_path

def path_smoothening(sampled_path, final_state, canvas, unknown = False):
    """
    @brief: This function smoothenes the path by connecting the start nodes with the most feasible node starting
    from the goal node 
    :param sampled path: Sampled Path
    :param final_state: final node (Goal Node)
    :param canvas: Map
    :param unknown: Flag for dynamic map ( unknown obstacles )
    """
    shortest_path = []
    # if len(sampled_path) > 0:
    shortest_path.append(sampled_path[0])
    print("Length of Sampled Path: ",len(sampled_path))
    while (tuple(shortest_path[-1]) != tuple(sampled_path[-1])):
        # print(sampled_path.index(shortest_path[-1]))
        for i in range(sampled_path.index(shortest_path[-1]),len(sampled_path)):
            if collision_free(shortest_path[-1],sampled_path[len(sampled_path)-1-i+sampled_path.index(shortest_path[-1])], canvas, unknown):
                shortest_path.append(sampled_path[len(sampled_path)-1-i+sampled_path.index(shortest_path[-1])])
                break
    # print(shortest_path)
    return shortest_path
    

def path_replanning(path, dynamic_map, edges, vertices, initial, final):
    """
    @brief: This function replans the path based on the dynamic obstacles present in the map.
    :param path: actual path to be followed
    :param dynamic_map: Dynamic map
    :param edges: Edges
    :param vertices: Vertices of the tree
    :param initial: starting node
    :param final: Goal Node
    """
    replanned_path = path.copy()
    print("in path replanning")
    for i in range(1,len(path)):
        node = replanned_path[i]
        X_next = node.copy()
        if is_obstacle(node[0],node[1],dynamic_map,True) or (not collision_free(replanned_path[i-1],X_next,dynamic_map, unknown=True)):
            X_curr = replanned_path[i-1].copy()
            X_candi = []
            X_near = neighbor_nodes(X_curr, vertices, d=50)
            for near in X_near:
                if collision_free(X_curr,near,dynamic_map, unknown=True):
                    if near not in replanned_path:
                        X_candi.append(near)
            
            X_next = pareto(X_candi,X_curr,final)
            # X_next = X_candi[0]
        if(X_next is not None):
            # print("Nearby node found!")
            # X_curr = X_next.copy()
            # print("Previous: ",replanned_path[i])
            replanned_path[i] = X_next.copy()
            # print("Updated: ",replanned_path[i])

        else:
            print("Not Enough Samples found nearby and hence the path goes through the obstacle")
            
    # print("Replanned Path: ", replanned_path)
    return replanned_path

def pareto(X_candi,initial,final):
    """
    @brief: This function returns the most dominant node by using the pareto dominance theory
    :param X_candi: Candidate Nodes
    :param initial: Initial Node 
    :paran final: Final Node (Goal Node)
    """
    paretos = []
    for candidates in X_candi:
        paretos.append([compute_distance(candidates,initial), cost_to_goal(candidates,final)])
    
    if(len(paretos) != 0):
        pareto_dict = {}
        for i in range(0,len(paretos)):
            dominant_node = paretos[i].copy()
            ID = 0
            OD = 0
            for j in range(0,len(paretos)):
                if(tuple(paretos[i]) == tuple(paretos[j])):
                    continue
                elif ((paretos[j][0]<=dominant_node[0] and paretos[j][1]<=dominant_node[1]) and (paretos[j][0]<dominant_node[0] or paretos[j][1]<dominant_node[1])):
                    ID += 1
                elif (((paretos[j][0]>=dominant_node[0] and paretos[j][1]>=dominant_node[1]) and (paretos[j][0]>dominant_node[0] or paretos[j][1]>dominant_node[1]))):
                    OD += 1
            pareto_dict[tuple(dominant_node)] = [ID, OD]
        
        pareto_keys = list(pareto_dict.keys())
        pareto_IDs = []
        pareto_ODs = []
        for p_key in pareto_keys:
            pareto_IDs.append(pareto_dict[tuple(p_key)][0])
            pareto_ODs.append(pareto_dict[tuple(p_key)][1])
        zero_ID_index = list(np.where(np.array(pareto_IDs)==0))[0]
        # print("Zero ID Index Type: ",type(zero_ID_index), zero_ID_index)
        if(len(zero_ID_index)>1):
            zero_ID_keys = []
            for i in zero_ID_index:
                zero_ID_keys.append(pareto_keys[i])
            
            zero_ID_max_OD = []
            for key in zero_ID_keys:
                zero_ID_max_OD.append(pareto_dict[tuple(key)][1])
            max_OD = np.max(zero_ID_max_OD)
            max_OD_key = zero_ID_keys[zero_ID_max_OD.index(max_OD)]

            # print(max_OD_key)
            return X_candi[paretos.index(list(max_OD_key))]
        elif(len(zero_ID_index)==1):
            return X_candi[paretos.index(list(pareto_keys[zero_ID_index[0]]))]
        else:
            print("NO PARETO!")   
            
    else:
        print("No Candidate Nodes")

if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument('--map1', action='store_true',
                        help="Loads Map 01")
    parser.add_argument('--map2', action='store_true',
                        help="Loads Map 02")
    args = parser.parse_args()
    
    #Gives the time at which the program has started
    canvas = np.ones((250,400,3),dtype="uint8") #Creating a blank canvas
    
    if args.map1:
        flag = 1
    elif args.map2:
        flag = 2
    
    canvas = draw_obstacles(canvas,clr=5,unknown=False,map_flag=flag) #Draw the obstacles in the canvas, default point robot with 5 units of clearance
    
    initial_state,final_state = [10,10], [350,150] #Take the start and goal node from the user

    #Changing the cartesian coordinates to image coordinates:
    initial_state[1] = canvas.shape[0]-1 - initial_state[1]
    final_state[1] = canvas.shape[0]-1 - final_state[1]
    
    #Write a condition to check if the initial state and final state are in the obstacle space and exit from program and ask to rerun with valid start and goal positions
    if(canvas[initial_state[1]][initial_state[0]][0]==255 or canvas[final_state[1]][final_state[0]][0]==255):
        print("Given Start or Goal Node is in the Obstacle Region. Please re-run with Valid Coordinates")
        exit()

    start_time = time.time()
    #Start Node and Goal Node
    cv2.circle(canvas,(int(initial_state[0]),int(initial_state[1])),5,(0,255,0),-1)
    cv2.circle(canvas,(int(final_state[0]),int(final_state[1])),5,(0,0,255),-1)
    
    #Generate the graph from the Modified RRT* Algorithm
    edges, vertices, s_node = mod_rrt_star(initial_state,final_state,canvas) #Compute the path using A Star Algorithm
    
    tree_canvas = canvas.copy()
    #Draw the edges
    for key in edges:
        # cv2.line(canvas, tuple(key), tuple(edges[key]), (255, 128, 223), 2)
        cv2.line(tree_canvas, tuple(key), tuple(edges[key]), (0, 0, 255), 3)
        cv2.circle(tree_canvas,(key[0], key[1]), 1, (0,255,255), -1)
        cv2.circle(tree_canvas,(edges[key][0],edges[key][1]), 1, (0,255,0), -1)
    # cv2.imshow("Modified RRT* Tree Expansion", canvas)
    
    #Generate a dynamic map
    dynamic_map = np.ones((250,400,3),dtype="uint8") #Creating a blank canvas
    dynamic_map = draw_obstacles(dynamic_map,clr=5,unknown=True, map_flag=flag) #Draw the obstacles in the canvas, default point robot with 5 units of clearance
    cv2.imshow("Known Map with Unknown Obstacles", dynamic_map)

    #Backtack the path to reach from start node to goal node
    path = backtrack(s_node, final_state, edges, canvas)
    rrt_path_canvas = tree_canvas.copy()
    for i in range(1,len(path)):
        cv2.line(rrt_path_canvas, tuple(path[i-1]), tuple(path[i]), (0, 255, 0), 3)
    # cv2.imshow("Modified RRT* Path", rrt_path_canvas)
    
    #Sample and Smoothen the path from the list returned from the backtracking function.
    sampled_path = path_sampling(path)

    smoothened_path = path_smoothening(sampled_path.copy(),final_state,canvas,unknown = False)
    
    smooth_rrt_path_canvas = rrt_path_canvas.copy()
    for i in range(1,len(smoothened_path)):
        cv2.line(smooth_rrt_path_canvas, tuple(smoothened_path[i-1]), tuple(smoothened_path[i]), (255, 255, 255), 3)
    # cv2.imshow("Smoothened Modified RRT* Path", smooth_rrt_path_canvas)

    #Resample the smoothened path
    sampled_path = path_sampling(smoothened_path)

    #Replan the path from the dynamic obstacles
    replanned_path = path_replanning(sampled_path, dynamic_map, edges, vertices,s_node,final_state)

    replanned_path_canvas = dynamic_map.copy()
    for i in range(0,len(replanned_path)):
        cv2.circle(replanned_path_canvas,tuple(replanned_path[i]), 2, (0,145,145), -1)

    n_path  = []
    prev_path = []
    for i in range(0,len(replanned_path)):
        if(tuple(sampled_path[i]) == tuple(replanned_path[i])):
            
            prev_path.append(sampled_path[i])
            continue
        else:
            # print("N Path Append")
            n_path.append(sampled_path[i-2])
            n_path.append(sampled_path[i-1])
            for j in range(i,len(replanned_path)):
                n_path.append(replanned_path[j])
            break

    # cv2.imshow("Replanned Modified RRT* Path", replanned_path_canvas)

    # replanned_sampled = path_sampling(replanned_path)
    # print("New path ",n_path)
    new_replanned_path = path_smoothening(n_path.copy(), final_state, dynamic_map, unknown=True)

    smooth_replanned_path_canvas = replanned_path_canvas.copy()
    for i in range(1,len(sampled_path)):
        cv2.line(dynamic_map, tuple(sampled_path[i-1]), tuple(sampled_path[i]), (0, 137, 255), 3)

    # print(replanned_path)
    for i in range(1,len(prev_path)):
        cv2.line(smooth_replanned_path_canvas, tuple(prev_path[i-1]), tuple(prev_path[i]), (255, 128, 223), 2)
    for i in range(1,len(new_replanned_path)):
        cv2.line(smooth_replanned_path_canvas, tuple(new_replanned_path[i-1]), tuple(new_replanned_path[i]), (255, 128, 223), 2)
        # cv2.circle(dynamic_map,tuple(replanned_path[i]), 3, (255,255,255), -1)
    # cv2.imshow("Smoothened Replanned Modified RRT* Path",dynamic_map)

    end_time = time.time() #Time taken to run the whole algorithm to find the optimal path

    cv2.imshow("Known Map with Initial & Final Nodes", canvas)
    cv2.imshow("Modified RRT* Tree Expansion", tree_canvas)
    # cv2.imshow("Known Map with Unknown Obstacles", dynamic_map)
    cv2.imshow("Modified RRT* Path", rrt_path_canvas)
    cv2.imshow("Smoothened Modified RRT* Path", smooth_rrt_path_canvas)
    # cv2.imshow("Replanned Pareto Dominant Nodes", replanned_path_canvas)
    cv2.imshow("Smoothened Replanned Modified RRT* Path", smooth_replanned_path_canvas)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
    # print("Code Execution Time: ",end_time-start_time) #Prints the total execution time