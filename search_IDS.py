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
        self.depth = 0 #TODO: added .depth

    # Returns string representation of the state
    def __repr__(self):
        return str(self.state.tiles)

    # Comparing current node with other node. They are equal if states are equal
    def __eq__(self, other):
        return self.state.tiles == other.state.tiles

    def __hash__(self):
        return hash(tuple(self.state.tiles))

class IDDFS:
    # globals
    def __init__(self, root):
        self.root = root
        self.num_expanded = 0
        self.max_memory = 0

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
        return cur_tiles == ["1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15", "0"]

    # This function runs iterative deepening search from the given root node and returns path, number of nodes expanded, total time taken, and memory
    def run(self):
        self.root.depth = 0  # TODO: ADDED DEPTH TO NODES
        for depth in range(sys.maxsize):
            result = self.run_dls(self.root, depth)

            if isinstance(result,Node):
                return self.find_path(result), self.num_expanded, self.max_memory    
        return False
    
    # This function runs depth-limited search on the given root node and limit and returns path/cutoff/failure, number of nodes expanded, and memory 
    def run_dls(self, node, limit):
        frontier = [node]
        result = "failure"
        visited = {}
        
        while frontier:
            self.max_memory = max(self.max_memory, sys.getsizeof(frontier) + sys.getsizeof(visited))
            node = frontier.pop()
            visited[node] = True
            self.num_expanded += 1

            if self.goal_test(node.state.tiles):
                return node
            elif node.depth > limit: #TODO: added .depth
                result = "cutoff"
            else:
                children = self.get_children(node)
                for child in children:
                    if child not in visited:
                        child.depth = node.depth + 1 #TODO: added .depth
                        frontier.append(child)
        return result
    pass

class Search:

    def solve(self, input):
        initial_time = time.time()
        initial_list = input.split(" ")
        root = Node(Board(initial_list), None, None)

        ids = IDDFS(root)
        path, expanded_nodes, memory_consumed = ids.run()
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
    