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
        self.cost = 0 # to be modified

    # Returns string representation of the state
    def __repr__(self):
        return str(self.state.tiles)

    # Comparing current node with other node. They are equal if states are equal
    def __eq__(self, other):
        return self.state.tiles == other.state.tiles
    
    def __lt__(self, other):
        return self.cost < other.cost

    def __hash__(self):
        return hash(tuple(self.state.tiles))



class Search:
    # This function returns the list of children obtained after simulating the actions on current node
    def get_children(self, parent_node):
        children = []
        for action in ['U', 'D', 'L', 'R']:
            new_state = parent_node.state.execute_action(action)
            if new_state and new_state != parent_node.state:
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

    def heuristic(self, state):
        # Manhattan distance heuristic
        distance = 0
        for i, tile in enumerate(state):
            tile = int(tile)
            if tile != 0:
                distance += abs(i // 4 - (tile - 1) // 4) + abs(i % 4 - (tile - 1) % 4)
        return distance
    
    def heuristic2(self, state):
        distance = 0
        goal = ["1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15", "0"]
        for i in range(16):
            if state[i] != goal[i]:
                distance += 1
        return distance


    # This function runs breadth first search from the given root node and returns path, number of nodes expanded and total time taken
    def run_bfs(self, root_node):
        start_time = time.time()
        curr_node = root_node
        max_memory = 0
        frontier = queue.PriorityQueue()
        frontier.put(root_node)
        visited = {}

        while not frontier.empty():
            curr_node = frontier.get()
            
            # solution found
            if self.goal_test(curr_node.state):
                return self.find_path(curr_node), len(visited), max_memory

            visited[curr_node.state] = True
            children = self.get_children(curr_node)
            for child in children:
                if child.state not in visited:
                    child.cost = self.heuristic(child.state.tiles)
                    frontier.put(child)

            max_memory = max(max_memory, sys.getsizeof(frontier) + sys.getsizeof(visited))

        # failure   
        return False
            

    def goal_test(self, cur_tiles):
        goal = ["1", "2", "3", "4", "5", "6", "7", "8", "9", "10", "11", "12", "13", "14", "15", "0"]
        return cur_tiles.tiles == goal

    def solve(self, input):
        initial_time = time.time()
        initial_list = input.split(" ")
        root = Node(Board(initial_list), None, None)
        path, expanded_nodes, memory_consumed = self.run_bfs(root)
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
    agent.solve("5 1 2 3 9 10 6 4 13 15 7 8 14 0 11 12") # 16 moves LIMIT
    agent.solve("14 9 6 5 15 13 7 1 12 10 11 2 8 4 3 0") # 37 moves LIMIT
