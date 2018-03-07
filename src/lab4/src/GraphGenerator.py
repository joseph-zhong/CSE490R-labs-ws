#!/usr/bin/env python

# Import standard python libraries
import rospy
from nav_msgs.srv import GetMap
import networkx as nx
import math
import numpy
import Dubins
import KinematicModel as model
import IPython
from ObstacleManager import ObstacleManager

# Halton Sequence Generator
def halton_sequence_value(index, base):
    
    result = 0
    f = 1

    while index > 0:
        f = f*1.0/base
        result = result + f*(index % base)
        index = index/base
    
    return result

# Wrap the values around 0 and 1
def wrap_around(coordinate):

    for i in range(numpy.size(coordinate)):
        if coordinate[i] > 1.0:
            coordinate[i] = coordinate[i] - 1.0
        if coordinate[i] < 0:
            coordinate[i] = 1.0 + coordinate[i]

    return coordinate

# Halton Graph Generator
def euclidean_halton_graph(n, radius, bases, lower, upper, source, target, mapFile):

    manager = ObstacleManager(mapFile)

    G = nx.DiGraph()
    upper = numpy.array(upper)
    lower = numpy.array(lower)
    scale = upper-lower
    offset = lower

    position = []

    numVertices = 0
    haltonIndex = 1

    if source is not None:
      position.append(source)
      num_vertices += 1
    if target is not None:
      position.append(target)
      num_vertices += 1

    while numVertices < n:
        p = wrap_around(numpy.array([halton_sequence_value(haltonIndex,base) for base in bases]))
        p = p * scale + offset

        if manager.get_state_validity(p):
            position.append(p)
            numVertices += 1

        haltonIndex += 1

    state = [" ".join(str(x) for x in p) for p in position]

    for i in range(n):
        node_id = i
        G.add_node(str(node_id), state = state[i])

    for i in range(n-1):     
        print i
        for j in range(i+1,n):
            edgeLength = Dubins.path_length(position[i], position[j], 1.0/model.TURNING_RADIUS)
            euclideanLength = numpy.linalg.norm(position[i][0:2] - position[j][0:2])
            if edgeLength < radius:
                G.add_edge(str(i), str(j), length = str(edgeLength)) 
            edgeLength = Dubins.path_length(position[j], position[i], 1.0/model.TURNING_RADIUS)
            if edgeLength < radius:
                G.add_edge(str(j), str(i), length = str(edgeLength)) 
    return G

def insert_vertices(G, configs, radius):

    numVertices = G.number_of_nodes()
    for config in configs:
        state = " ".join(str(x) for x in config)
        G.add_node(str(numVertices), state = state)
        for i in range(numVertices):
            position = [float(a) for a in G.node[str(i)]["state"].split()]
            
            edgeLength = Dubins.path_length(config, position, 1.0/model.TURNING_RADIUS)
            if edgeLength < radius:
                G.add_edge(str(numVertices), str(i), length = str(edgeLength))
            
            edgeLength = Dubins.path_length(position, config, 1.0/model.TURNING_RADIUS)
            if edgeLength < radius:
                G.add_edge(str(i), str(numVertices), length = str(edgeLength))
        numVertices += 1

    #nx.write_graphml(G, "currentHalton.graphml")

# Main Function
if __name__ == "__main__":
    rospy.init_node("generate_graph")
    map_service_name = rospy.get_param("~static_map", "static_map")
    print("Getting map from service: ", map_service_name)
    rospy.wait_for_service(map_service_name)
    
    graph_file = rospy.get_param("~graph_file", None)
    map_msg = rospy.ServiceProxy(map_service_name, GetMap)().map
    map_info = map_msg.info
    
    spaceDimension = 3

    if spaceDimension == 3:
        bases = [2,3,5]

    lower = numpy.array([map_info.origin.position.x, map_info.origin.position.y,0.0])
    upper = numpy.array([map_info.origin.position.x+map_info.resolution*map_info.width, map_info.origin.position.y+map_info.resolution*map_info.height, 2*numpy.pi])

    # Settings
    halton_points = 2 # TODO: Set this appropriately
    disc_radius = 1 # TODO: Set this appropriately
    print(disc_radius)

    for i in range(1):
        print i
        numpy.random.seed()
        offset = numpy.random.random_sample(spaceDimension,)
        riskmapFile = 'haltonGraph.graphml'

        # Generate the graph
        print 'Generating the graph'
        G = euclidean_halton_graph(halton_points, disc_radius, bases, lower, upper, None, None, map_msg)
        nx.write_graphml(G, riskmapFile)
