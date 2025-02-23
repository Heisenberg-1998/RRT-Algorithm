import csv
import os
import random
import numpy as np
import math
import matplotlib.pyplot as plt 

class Obstacles:
    def __init__(self, x, y, diameter):
        self.x = x
        self.y = y
        self.diameter = diameter

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.id = None

class Edge:
    def __init__(self, id1, id2, cost):
        self.id1 = id1
        self.id2 = id2
        self.cost = cost
            

def sampling_method(x_range, y_range):
    x = round(random.uniform(x_range[0], x_range[1]), 4)
    y = round(random.uniform(y_range[0], y_range[1]), 4)
    sample = Node(x, y)
    return sample

def cal_distance(Node1:Node, Node2:Node):
    return (math.sqrt((Node1.x - Node2.x)**2 + (Node1.y - Node2.y) **2))


def find_nearest_node(search_tree, sample:Node) -> Node:
    # Initialized minimum distance to infinite large
    min_distance = float('inf')

    # Find the nearest node
    for node in search_tree:
        distance = cal_distance(node, sample)
        if distance < min_distance:
            min_distance = distance
            nearest_node = node
    return nearest_node

def generate_new_node(sample:Node, nearest_node:Node, step) -> Node:
    distance = cal_distance(sample, nearest_node)

    if distance > step:
        delta_x = sample.x - nearest_node.x
        delta_y = sample.y - nearest_node.y
        x_new_node = nearest_node.x + ((delta_x / distance) * step)
        y_new_node = nearest_node.y + ((delta_y / distance) * step)
        new_node = Node(x_new_node, y_new_node)
    else:
        return sample
    return new_node


def check_repeat_node(new_node:Node, search_tree:list) -> bool:
    for node in search_tree:
        if cal_distance(new_node, node) < step:
            return False
    return True

# Read obstacles data from csv
def read_csv_obstacles(filename):
    obstacle_list = []
    with open(filename, 'r') as csvfile:
        csvreader = csv.reader(csvfile)

        # skip the row begin with '#'
        # row[0] represent the first character of ever row
        for row in csvreader:
            if row[0].startswith('#'):
                continue
            obstacle_x = float(row[0])
            obstacle_y = float(row[1])
            obstacle_diameter = float(row[2])
            obstacle_list.append(Obstacles(obstacle_x, obstacle_y, obstacle_diameter))
        
    return obstacle_list

def check_collision(node1:Node, node2:Node, obstacles:list):
    for obs in obstacles:
      a_x = node1.x - obs.x
      a_y = node1.y - obs.y
      b_x = node2.x - obs.x 
      b_y = node2.y - obs.y
      radius = ((obs.diameter)/2)+0.02
      a = (b_x - a_x)** 2 + (b_y - a_y)**2
      b = 2*((a_x*(b_x - a_x)) + a_y * (b_y - a_y))
      c = a_x ** 2 + a_y ** 2 - radius ** 2
      
      # discriminant = b^2 - 4ac
      discriminant = b ** 2 - 4 * a * c
      
      # If no collision , return true
      if discriminant < 0:
          continue
      # If in collision, return false
      sqrtDist = math.sqrt(discriminant)
     
      # t = [-b ± sqrt(b^2 - 4ac)] / 2a
      t_1 = (-b + sqrtDist) / (2 * a)
      t_2 = (-b - sqrtDist) / (2 * a)
      if (t_1 >= 0 and t_1 <= 1) or (t_2 >= 0 and t_2 <= 1):
        return False
    return True

def check_done(new_node: Node, goal:Node, step:float) -> bool:
    if cal_distance(new_node, goal) < step:
        return True
    else:
        return False
    

def write_path(path, filename = 'path.csv'):
    current_dir = os.path.dirname(os.path.abspath(__file__))
    result_dir = os.path.join(current_dir, 'result')

    if not os.path.exists(result_dir):
        os.makedirs(result_dir)
    
    file_path = os.path.join(result_dir, filename)

    with open(file_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(path)


def write_node(search_tree, filename = 'node.csv'):
    current_dir = os.path.dirname(os.path.abspath(__file__))
    result_dir = os.path.join(current_dir, 'result')
    if not os.path.exists(result_dir):
        os.makedirs(result_dir)
    
    file_path = os.path.join(result_dir, filename)
    
    with open(file_path, mode = 'w', newline='') as file:
        writer = csv.writer(file)
        
        writer.writerow(['#id', 'x', 'y'])

        for node in search_tree:
            writer.writerow([node.id, node.x, node.y])


def write_edges(edge_list:list[Edge], filename = 'edges.csv'):
    current_dir = os.path.dirname(os.path.abspath(__file__))
    result_dir = os.path.join(current_dir, 'result')
    if not os.path.exists(result_dir):
        os.makedirs(result_dir)
    
    file_path = os.path.join(result_dir, filename)
    with open(file_path, mode = 'w', newline='') as file:
        writer = csv.writer(file)
        
        writer.writerow(['#id1', 'id2', 'edges'])

        for edge in edge_list:
            writer.writerow([edge.id1, edge.id2, edge.cost])

if __name__ == '__main__':
    print("process begin...")
    goal = Node(0.5, 0.5)
    step = 0.1
    node_id = 1
    
    fig = plt.figure()
    ax = fig.subplots()
    ax.set_xlim(-0.6, 0.6)
    ax.set_ylim(-0.6, 0.6)    

    # Start point initialization
    start = Node((-0.5), (-0.5))
    start.id = 1
    start.parent = None

    x_range = [-0.5, 0.5]
    y_range = [-0.5, 0.5]
    search_tree: list[Node] = []
    maximum_iteration = 200

    # search_tree initialization
    search_tree.append(start)

    # Initialized a edge_list
    edge_list: list[Edge] = []

    # get obstacle list
    file_path = 'result/obstacles.csv'
    obstacles_list: list[Obstacles] = []
    obstacles_list = read_csv_obstacles(file_path)

    while len(search_tree) < maximum_iteration:
        # Create Node object name sample
        sample = sampling_method(x_range, y_range)
        #print("sample = ", sample.x, sample.y)

        # Find the nearest node
        nearest_node = find_nearest_node(search_tree, sample)
        #print("nearest node = ", nearest_node.x, nearest_node.y)

        # Find new node according to the step
        new_node = generate_new_node(sample, nearest_node, step)
        #print("new_node = ", new_node.x, new_node.y)

        # Check if collision occur
        free_motion = check_collision(new_node, nearest_node, obstacles_list)
        #print("free motion = ", free_motion)

        # Check if the generated new_node is already in the search tree
        repeat_node = check_repeat_node(new_node, search_tree)
        #print("repeat node = ", repeat_node)

        # If collision not occur, and the generated new_node is not repeat
        if free_motion and repeat_node:
            if new_node not in search_tree:
                # Add new node to the node list
                node_id += 1
                new_node.id = node_id
                new_node.parent = nearest_node
                print("new node = id", new_node.id, ",", new_node.x, new_node.y)
                search_tree.append(new_node)

                # Initialized a edge object
                edge = None
                edge = Edge(node_id, nearest_node.id, cal_distance(new_node, nearest_node))
                # Add edge to the edge_list
                edge_list.append(edge)
            done = check_done(new_node, goal, step)
            #print("done = ", done)
            if done:
                # Find the path
                print("Path found!")

                # Path generation
                path: list[int] = []    # Initialize a path contains node id
                node = new_node
                #print("new node = ",new_node.x, new_node.y, "parent", new_node.parent)
                while node.parent != None:
                    path.insert(0, node.id)
                    node = node.parent
                path.insert(0, 1)
                print(path)
                break
# File generation
write_path(path, filename = 'path.csv')
write_node(search_tree, filename = 'nodes.csv')
write_edges(edge_list, filename = 'edges.csv')

if not done:
    print('No path found')




            

            



