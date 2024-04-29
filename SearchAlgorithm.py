# This file contains all the required routines to make an A* search algorithm.
#
__authors__ = '1636581'
__group__ = 'DJ.12'

import sys

# _________________________________________________________________________________________
# Intel.ligencia Artificial
# Curs 2022 - 2023
# Universitat Autonoma de Barcelona
# _______________________________________________________________________________________

from SubwayMap import *
from utils import *
import os
import math
import copy


def expand(path, map):
    """
     It expands a SINGLE station and returns the list of class Path.
     Format of the parameter is:
        Args:
            path (object of Path class): Specific path to be expanded
            map (object of Map class):: All the information needed to expand the node
        Returns:
            path_list (list): List of paths that are connected to the given path.
    """
    path_list = []
    for i in map.connections[path.route[-1]].keys():
        path2 = copy.deepcopy(path)
        path2.add_route(i)
        path2.g = path.g
        path2.h = path.h
        path_list.append(path2)
    return path_list


def remove_cycles(path_list):
    """
     It removes from path_list the set of paths that include some cycles in their path.
     Format of the parameter is:
        Args:
            path_list (LIST of Path Class): Expanded paths
        Returns:
            path_list (list): Expanded paths without cycles.
    """

    new_pathlist = []

    for i in range(len(path_list)):
        if len(path_list[i].route) == len(set(path_list[i].route)):
            new_pathlist.append(path_list[i])
    return new_pathlist


def insert_depth_first_search(expand_paths, list_of_path):
    """
     expand_paths is inserted to the list_of_path according to DEPTH FIRST SEARCH algorithm
     Format of the parameter is:
        Args:
            expand_paths (LIST of Path Class): Expanded paths
            list_of_path (LIST of Path Class): The paths to be visited
        Returns:
            list_of_path (LIST of Path Class): List of Paths where Expanded Path is inserted
    """
    for i in reversed(range(len(expand_paths))):
        list_of_path.insert(0, expand_paths[i])
    return list_of_path


def depth_first_search(origin_id, destination_id, map):
    """
     Depth First Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
        Returns:
            list_of_path[0] (Path Class): the route that goes from origin_id to destination_id
    """
    # creo la llista de path amb el node inicial

    list_of_path = [Path(origin_id)]
    while len(list_of_path) != 0 and list_of_path[0].route[-1] != destination_id:
        exp = expand(list_of_path[0], map)  # busca caminos a partir de mi origen
        list_of_path.pop(0)
        rem = remove_cycles(exp)
        list_of_path = insert_depth_first_search(rem, list_of_path)
    return list_of_path[0]


def insert_breadth_first_search(expand_paths, list_of_path):
    """
        expand_paths is inserted to the list_of_path according to BREADTH FIRST SEARCH algorithm
        Format of the parameter is:
           Args:
               expand_paths (LIST of Path Class): Expanded paths
               list_of_path (LIST of Path Class): The paths to be visited
           Returns:
               list_of_path (LIST of Path Class): List of Paths where Expanded Path is inserted
    """
    for i in range(len(expand_paths)):
        list_of_path.append(expand_paths[i])
    return list_of_path


def breadth_first_search(origin_id, destination_id, map):
    """
     Breadth First Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """
    list_of_path = [Path(origin_id)]
    while len(list_of_path) != 0 and list_of_path[0].route[-1] != destination_id:
        exp = expand(list_of_path[0], map)  # busca caminos a partir de mi origen
        list_of_path.pop(0)
        rem = remove_cycles(exp)
        list_of_path = insert_breadth_first_search(rem, list_of_path)
    return list_of_path[0]


def calculate_cost(expand_paths, map, type_preference=0):
    """
         Calculate the cost according to type preference
         Format of the parameter is:
            Args:
                expand_paths (LIST of Paths Class): Expanded paths
                map (object of Map class): All the map information
                type_preference: INTEGER Value to indicate the preference selected:
                                0 - Adjacency
                                1 - minimum Time
                                2 - minimum Distance
                                3 - minimum Transfers
            Returns:
                expand_paths (LIST of Paths): Expanded path with updated cost
"""
    if type_preference == 0:
        for p in expand_paths:
                p.g = (len(p.route)-1)
        return expand_paths
    else:
        if type_preference == 1:
            for p in expand_paths:
                anterior = p.route[-2]
                seguent = p.route[-1]
                p.update_g(map.connections[anterior][seguent])
            return expand_paths
        else:
            if type_preference == 2:
                for p in expand_paths:
                    anterior = p.route[-2]
                    seguent = p.route[-1]
                    if map.stations[seguent]['line']== map.stations[anterior]['line']:
                        distancia = map.stations[p.route[-1]]['velocity'] * map.connections[anterior][seguent]
                        p.update_g(distancia)
                    else:
                        p.update_g(0)
                return expand_paths
            else:
                if type_preference == 3:
                    for p in expand_paths:
                        anterior = p.route[-2]
                        seguent = p.route[-1]
                        if map.stations[anterior]['line'] != map.stations[seguent]['line']:
                            p.update_g(1)
                    return expand_paths

def insert_cost(expand_paths, list_of_path):
    """
        expand_paths is inserted to the list_of_path according to COST VALUE
        Format of the parameter is:
           Args:
               expand_paths (LIST of Path Class): Expanded paths
               list_of_path (LIST of Path Class): The paths to be visited
           Returns:
               list_of_path (LIST of Path Class): List of Paths where expanded_path is inserted according to cost
    """
    aux = list_of_path + expand_paths
    aux = sorted(aux, key=lambda cost: cost.g)
    return aux


def uniform_cost_search(origin_id, destination_id, map, type_preference=0):
    """
     Uniform Cost Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
            type_preference: INTEGER Value to indicate the preference selected:
                            0 - Adjacency
                            1 - minimum Time
                            2 - minimum Distance
                            3 - minimum Transfers
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """


    list_of_path = [Path(origin_id)]
    while len(list_of_path) != 0 and list_of_path[0].route[-1] != destination_id:
        exp = expand(list_of_path[0], map)  # busca caminos a partir de mi origen
        list_of_path.pop(0)
        rem = remove_cycles(exp)
        calcul = calculate_cost(rem, map, type_preference)
        list_of_path = insert_cost(calcul, list_of_path)

    return list_of_path[0]


def calculate_heuristics(expand_paths, map, destination_id, type_preference=0):
    """
     Calculate and UPDATE the heuristics of a path according to type preference
     WARNING: In calculate_cost, we didn't update the cost of the path inside the function
              for the reasons which will be clear when you code Astar (HINT: check remove_redundant_paths() function).
     Format of the parameter is:
        Args:
            expand_paths (LIST of Path Class): Expanded paths
            map (object of Map class): All the map information
            destination_id (int): Final station id
            type_preference: INTEGER Value to indicate the preference selected:
                            0 - Adjacency
                            1 - minimum Time
                            2 - minimum Distance
                            3 - minimum Transfers
        Returns:
            expand_paths (LIST of Path Class): Expanded paths with updated heuristics
    """
    if type_preference == 0:
        for p in expand_paths:
            anterior = p.route[-1]
            if anterior != destination_id:
                p.h = 1
            else:
                p.h = 0
        return expand_paths
    else:
        if type_preference == 1:
            for p in expand_paths:
                anterior = p.route[-1]
                if anterior != destination_id:
                    v = max(map.stations[i]['velocity'] for i in map.stations)
                    d = euclidean_dist((map.stations[anterior]['x'], map.stations[anterior]['y']),(map.stations[destination_id]['x'], map.stations[destination_id]['y']))
                    p.h=(d / v)
                else:
                    p.h = 0
            return expand_paths
        else:
            if type_preference == 2:
                for p in expand_paths:
                    anterior = p.route[-1]
                    if anterior != destination_id:
                        d = euclidean_dist((map.stations[anterior]['x'], map.stations[anterior]['y']),(map.stations[destination_id]['x'], map.stations[destination_id]['y']))
                        p.h = d
                    else:
                        p.h = 0
                return expand_paths
            else:
                if type_preference == 3:
                    for p in expand_paths:
                        anterior = p.route[-1]
                        if anterior != destination_id:
                            if map.stations[anterior]['line'] != map.stations[destination_id]['line']:
                                p.h = 1
                            else:
                                p.h=0
                    return expand_paths


def update_f(expand_paths):
    """
      Update the f of a path
      Format of the parameter is:
         Args:
             expand_paths (LIST of Path Class): Expanded paths
         Returns:
             expand_paths (LIST of Path Class): Expanded paths with updated costs
    """
    for p in expand_paths:
        p.update_f()
    return expand_paths


def remove_redundant_paths(expand_paths, list_of_path, visited_stations_cost):
    """
      It removes the Redundant Paths. They are not optimal solution!
      If a station is visited and have a lower g-cost at this moment, we should remove this path.
      Format of the parameter is:
         Args:
             expand_paths (LIST of Path Class): Expanded paths
             list_of_path (LIST of Path Class): All the paths to be expanded
             visited_stations_cost (dict): All visited stations cost
         Returns:
             new_paths (LIST of Path Class): Expanded paths without redundant paths
             list_of_path (LIST of Path Class): list_of_path without redundant paths
             visited_stations_cost (dict): Updated visited stations cost
    """

    sin_reducir = []
    for path in expand_paths:
        if path.last not in visited_stations_cost:
            visited_stations_cost[path.last] = path.g
        else:
            if visited_stations_cost[path.last] <= path.g:
                continue
            else:
                visited_stations_cost[path.last] = path.g
                lista = []
                for i in list_of_path:
                    if path.last not in i.route:
                        lista.append(i)
                list_of_path = lista
        sin_reducir.append(path)

    return sin_reducir, list_of_path, visited_stations_cost

def insert_cost_f(expand_paths, list_of_path):
    """
        expand_paths is inserted to the list_of_path according to f VALUE
        Format of the parameter is:
           Args:
               expand_paths (LIST of Path Class): Expanded paths
               list_of_path (LIST of Path Class): The paths to be visited
           Returns:
               list_of_path (LIST of Path Class): List of Paths where expanded_path is inserted according to f
    """
    aux_paths = expand_paths + list_of_path
    aux_paths = sorted(aux_paths, key=lambda x: x.f)
    return aux_paths


def coord2station(coord, map):
    """
        From coordinates, it searches the closest stations.
        Format of the parameter is:
        Args:
            coord (list):  Two REAL values, which refer to the coordinates of a point in the city.
            map (object of Map class): All the map information
        Returns:
            possible_origins (list): List of the Indexes of stations, which corresponds to the closest station
    """

    dist_min = sys.float_info.max
    distancies = []
    estaciones = []
    for i in map.connections:
        station_coord = (map.stations[i]['x'], map.stations[i]['y'])
        dist = euclidean_dist(coord, station_coord)
        distancies.append(dist)
        if dist <= dist_min:
            dist_min = dist

    for j in range(len(distancies)):
        if distancies[j] == dist_min:
            estaciones.append(j + 1)
    return estaciones


def Astar(origin_id, destination_id, map, type_preference=0):
    """
     A* Search algorithm
     Format of the parameter is:
        Args:
            origin_id (int): Starting station id
            destination_id (int): Final station id
            map (object of Map class): All the map information
            type_preference: INTEGER Value to indicate the preference selected:
                            0 - Adjacency
                            1 - minimum Time
                            2 - minimum Distance
                            3 - minimum Transfers
        Returns:
            list_of_path[0] (Path Class): The route that goes from origin_id to destination_id
    """
    list_of_path = [Path(origin_id)]
    diccionari={}
    while len(list_of_path) != 0 and list_of_path[0].route[-1] != destination_id:
        exp = expand(list_of_path[0], map)  # busca caminos a partir de mi origen
        list_of_path.pop(0)
        rem = remove_cycles(exp)
        calcul = calculate_cost(rem, map, type_preference)
        heuristica = calculate_heuristics(calcul, map, destination_id, type_preference)
        actualizado = update_f(heuristica)
        actualizado,list_of_path,diccionari = remove_redundant_paths(actualizado,list_of_path,diccionari)
        list_of_path = insert_cost_f(actualizado, list_of_path)
    return list_of_path[0]
