#CS 411 - Assignment 3 Starter Code by Sarit Adhikari
#Breadth First Search on 15 Puzzle
#Spring 2024
# Completed by Rovia Simmons
# This Python file contains functions for solving the 15 tile puzzle at any given board position
# Use the Search object to run Breadth-First Search: agent.solve(input)
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
import queue


# This class defines the state of the problem in terms of board configuration
class Board:
    def __init__(self, tiles):
        self.tiles = tiles


    # This function returns the resulting state from taking particular action from current state
    def execute_action(self, action):
        empty_index = self.tiles.index("0")

        actions = {'U': -4, 'D': 4, 'L': -1, 'R': 1}
        new_index = empty_index + actions[action]

        # bounds check
        if not (0 <= new_index < len(self.tiles)) or \
           action == 'R' and empty_index % 4 == 3 or \
           action == 'L' and empty_index % 4 == 0:
            return self
        
        # swap
        new_tiles = list(self.tiles)
        new_tiles[empty_index], new_tiles[new_index] = new_tiles[new_index], new_tiles[empty_index] 
        return Board(new_tiles)



# This class defines the node on the search tree, consisting of state, parent and previous action
class Node:
    def __init__(self, state, parent, action):
        self.state = state
        self.parent = parent
        self.action = action

    # Returns string representation of the state
    def __repr__(self):
        return str(self.state.tiles)

    # Comparing current node with other node. They are equal if states are equal
    def __eq__(self, other):
        return self.state.tiles == other.state.tiles
    
    def __lt__(self, other):
        return self.fscore  <= other.fscore 

    def __hash__(self):
        return hash(tuple(self.state.tiles))


class Search:
    def __init__(self):
        moves = []
        nodes_expanded = 0
        self.goal = ["1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15", "0"]
        
    # This function returns the list of children obtained after simulating the actions on current node
    def get_children(self, parent_node):
        for action in ['U', 'D', 'L', 'R']:
            new_state = parent_node.state.execute_action(action)
            if new_state and new_state != parent_node.state:
                yield Node(new_state, parent_node, action)

    # This function backtracks from current node to reach initial configuration. The list of actions would constitute a solution path
    def find_path(self, node):
        actions = []
        while node.parent is not None:
            actions.insert(0, node.action)
            node = node.parent
        return actions

    # Manhattan distance heuristic from state to goal
    def h_manhattan(self, state):
        tiles= state.tiles
        distance = 0
        for i in range(0, len(tiles)):
            tile = int(tiles[i])
            if tile == 0: continue
            distance += abs(i // 4 - (tile - 1) // 4) + abs(i % 4 - (tile - 1) % 4)
        return distance
    
    # number of misplaced tiles
    def h_misplaced(self, state):
        distance = 0
        for i in range(len(state.tiles)):
            if state.tiles[i] != self.goal[i]:
                distance += 1
        return distance

    def run_ida_star(self, root_node):
        max_memory = 0
        expanded_nodes = 0
        bound = self.heuristic(root_node.state)
        while True:
            result = self.search(root_node, 0, bound)
            # max_memory = max(max_memory, psutil.Process().memory_info().rss)
            # expanded_nodes += expanded
            if isinstance(result, Node):
                return self.find_path(result), expanded_nodes, max_memory
            if result == float('inf'): # failure
                return False
            bound = result

    def search(self, node, g, bound):
        fscore = g + self.heuristic(node.state)
        if fscore > bound:
            return fscore
        if self.goal_test(node.state):
            return node
        min_bound = float('inf')
        for child in self.get_children(node):
            result = self.search(child, g + 1, bound)
            if isinstance(result, Node):
                return result
            min_bound = min(min_bound, result)
        return min_bound
            

    def goal_test(self, state):
        return state.tiles == self.goal

    def solve(self, input):
        # set heuristic here
        self.heuristic = self.h_manhattan

        initial_time = time.time()
        initial_list = input.split(" ")
        root = Node(Board(initial_list), None, None)
        path, expanded_nodes, memory_consumed = self.run_ida_star(root)
        print("Moves: " + " ".join(path))
        print("Number of expanded Nodes: " + str(expanded_nodes))
        print("Time Taken: " + str(time.time() - initial_time))
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
    