#CS 411 - Assignment 4 
# Starter Code by Sarit Adhikari
# Iterative Deepening Search on 15 Puzzle
#Spring 2024
# Completed by Rovia Simmons
# This Python file contains functions for solving the 15 tile puzzle at any given board position
# Use the Search object to run Iterative Deepening Search: agent.solve(input)
# Solve returns solution path, number of expansions, time taken, maximum memory usage in bytes

from logging import root
from operator import truediv
import random
import math
from shutil import move
import time
import psutil
import os
from collections import deque
import sys
from heapq import *


# This class defines the state of the problem in terms of board configuration
class Board:
    def __init__(self, tiles):
        self.tiles = tiles

    # This function returns the resulting state from taking particular action from current state
    def execute_action(self, action):
        new_tiles = self.tiles[:]
        empty_index = self.tiles.index("0")

        actions = {'U': -4, 'D': 4, 'L': -1, 'R': 1}
        new_index = empty_index + actions[action]

        # bounds check
        if (0 <= new_index < len(self.tiles)) and not \
           ((action == 'R' and empty_index % 4 == 3) or \
            (action == 'L' and empty_index % 4 == 0)):
            new_tiles[empty_index], new_tiles[new_index] = new_tiles[new_index], new_tiles[empty_index] 
        return Board(new_tiles)



# This class defines the node on the search tree, consisting of state, parent and previous action
class Node:
    def __init__(self, state, parent, action):
        self.state = state
        self.parent = parent
        self.action = action
        if parent is None:
            self.cost=0
        else:
            self.cost = parent.cost+1

    # Returns string representation of the state
    def __repr__(self):
        return str(self.state.tiles)

    # Comparing current node with other node. They are equal if states are equal
    def __eq__(self, other):
        return self.state.tiles == other.state.tiles
    
    def __lt__(self, other):
        return id(self) <= id(other)

    def __hash__(self):
        return hash(tuple(self.state.tiles))

class Astar:
    # globals
    def __init__(self, root, heuristic):
        self.goal = ["1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15", "0"]
        self.root = root
        self.num_expanded = 0
        self.max_memory = 0
        if heuristic == "manhattan":
            self.h_value = self.h_manhattan
        else:
            self.h_value = self.h_misplaced

    # This function runs iterative deepening search from the given root node and returns path, number of nodes expanded, total time taken, and memory
    def run(self):
        frontier = []
        visited = {}
        max_memory = 0
        heappush(frontier, (self.f_value(self.root), self.root))
        expanded_node_count=0
        while(frontier):
            max_memory = max(max_memory, sys.getsizeof(frontier)+sys.getsizeof(visited))   
            cur_node = heappop(frontier)[1] # get node not cost
            expanded_node_count+=1
            visited[cur_node] = True
            if(self.goal_test(cur_node.state.tiles)):
                return self.find_path(cur_node),len(visited), max_memory
            for child in self.get_children(cur_node):
                if child not in visited:
                    heappush(frontier, (self.f_value(child), child))
        print("frontier empty") 
        return False
    
    def f_value(self, node):
        return node.cost + self.h_value(node)

    # This function returns the list of children obtained after simulating the actions on current node
    def get_children(self, parent_node):
        children = []
        for action in ['L', 'R', 'U', 'D']:
            new_state = parent_node.state.execute_action(action)
            child_node = Node(new_state, parent_node, action)
            children.append(child_node)
        return children

    # This function backtracks from current node to reach initial configuration. The list of actions would constitute a solution path
    def find_path(self, node):
        actions = []
        while node.parent is not None:
            actions.insert(0, node.action)
            node = node.parent
        return actions
    
    def goal_test(self, cur_tiles):
        return cur_tiles == self.goal
    
    # heuristic functions

    #This function calculates sum of manhattan distances of each tile from goal position
    def h_manhattan(self,node):
        tiles = node.state.tiles
        distance = 0
        for i in range(0, len(tiles)):
            tile = int(tiles[i])
            if tile == 0: continue
            distance += abs(i / 4 - (tile - 1) / 4) + abs(i % 4 - (tile - 1) % 4)
        return distance
    
    #This function calculates number of misplaced tiles from goal position
    def h_misplaced(self, state):
        distance = 0
        for i in range(len(state.tiles)):
            if state.tiles[i] != self.goal[i]:
                distance += 1
        return distance
    
    pass # end of method class

class Search:

    def solve(self, input):
        initial_time = time.time()
        initial_list = input.split(" ")
        root = Node(Board(initial_list), None, None)

        method = Astar(root, "manhattan")
        path, expanded_nodes, memory_consumed = method.run()
        time_taken = time.time() - initial_time

        print("Moves: " + " ".join(path))
        print("Number of expanded Nodes: " + str(expanded_nodes))
        print("Time Taken: " + str(time_taken))
        print("Max Memory (Bytes): " + str(memory_consumed))
        return "".join(path)

# Testing the algorithm locally
if __name__ == '__main__':
    agent = Search()
    agent.solve("1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 0") # 0 moves
    agent.solve("1 2 3 4 5 6 7 8 9 10 11 0 13 14 15 12") # 1 move
    agent.solve("1 2 3 4 5 6 7 8 9 10 0 11 13 14 15 12") # 2 moves
    agent.solve("1 2 3 4 5 6 7 8 9 10 15 11 13 14 0 12") # 3 moves
    agent.solve("1 2 3 4 5 6 7 8 9 10 15 11 13 14 12 0") # 4 moves
    agent.solve("0 2 3 4 1 6 7 8 5 10 11 12 9 13 14 15") # 6 moves
    agent.solve("1 0 2 4 5 7 3 8 9 6 11 12 13 10 14 15") # 7 moves
    agent.solve("2 3 4 0 1 6 7 8 5 10 11 12 9 13 14 15") # 9 moves
    agent.solve("5 1 2 3 9 10 6 4 13 15 7 8 14 0 11 12") # 16 moves
    agent.solve("5 2 4 8 10 3 11 14 6 0 9 12 13 1 15 7")
    agent.solve("5 2 4 8 10 0 3 14 13 6 11 12 1 15 9 7")
    