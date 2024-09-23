# Created by Kartavya Singh, from team 

import heapq
import time
from collections import deque
from tabulate import tabulate
import random

# Romaina's Enntire Road Map Defined
romania_map = {
    'Arad': [('Zerind', 75), ('Sibiu', 140), ('Timisoara', 118)],
    'Zerind': [('Oradea', 71), ('Arad', 75)],
    'Oradea': [('Sibiu', 151), ('Zerind', 71)],
    'Timisoara': [('Lugoj', 111), ('Arad', 118)],
    'Lugoj': [('Mehadia', 70), ('Timisoara', 111)],
    'Mehadia': [('Drobeta', 75), ('Lugoj', 70)],
    'Drobeta': [('Craiova', 120), ('Mehadia', 75)],
    'Craiova': [('Pitesti', 138), ('Rimnicu Vilcea', 146), ('Drobeta', 120)],
    'Sibiu': [('Fagaras', 99), ('Rimnicu Vilcea', 80), ('Oradea', 151), ('Arad', 140)],
    'Rimnicu Vilcea': [('Pitesti', 97), ('Craiova', 146), ('Sibiu', 80)],
    'Pitesti': [('Bucharest', 101), ('Rimnicu Vilcea', 97), ('Craiova', 138)],
    'Fagaras': [('Bucharest', 211), ('Sibiu', 99)],
    'Bucharest': [('Giurgiu', 90), ('Urziceni', 85), ('Pitesti', 101), ('Fagaras', 211)],
    'Giurgiu': [('Bucharest', 90)],
    'Urziceni': [('Bucharest', 85), ('Hirsova', 98), ('Vaslui', 142)],
    'Hirsova': [('Eforie', 86), ('Urziceni', 98)],
    'Eforie': [('Hirsova', 86)],
    'Vaslui': [('Iasi', 92), ('Urziceni', 142)],
    'Iasi': [('Neamt', 87), ('Vaslui', 92)],
    'Neamt': [('Iasi', 87)]
}

# Straight Line Distances To Bucharest (for Best-First and A* Search Algorithms)
straight_line_distance_to_bucharest = {
    'Arad': 366, 'Zerind': 374, 'Oradea': 380, 'Timisoara': 329, 'Lugoj': 244,
    'Mehadia': 241, 'Drobeta': 242, 'Craiova': 160, 'Sibiu': 253, 'Rimnicu Vilcea': 193,
    'Pitesti': 100, 'Fagaras': 176, 'Bucharest': 0, 'Giurgiu': 77, 'Urziceni': 80,
    'Hirsova': 151, 'Eforie': 161, 'Vaslui': 199, 'Iasi': 226, 'Neamt': 234
}

# Define the Breadth First Search Algorithm:
def bfs(start, goal):
    queue = deque([(start, [start])])
    visited = set()

    while queue:
        city, path = queue.popleft()
        if city in visited:
            continue

        visited.add(city)
        if city == goal:
            return path
        
        for neighbor, _ in romania_map[city]:
            if neighbor not in visited:
                queue.append((neighbor, path + [neighbor]))
    return []

# Define the Depth First Search Algorithm:
def dfs(start, goal):
    stack = [(start, [start])]
    visited = set()

    while stack:
        city, path = stack.pop()
        if city in visited:
            continue

        visited.add(city)
        if city == goal:
            return path
        
        for neighbor, _ in romania_map[city]:
            if neighbor not in visited:
                stack.append((neighbor, path + [neighbor]))
    return []

# Define the Best-First Search Algorithm:\
# Using the Straight Line Distance to Bucharest as heuristic
def best_first_search(start, goal):
    priority_queue = [(straight_line_distance_to_bucharest[start], start, [start])]
    visited = set()
    
    while priority_queue:
        _, city, path = heapq.heappop(priority_queue)
        if city in visited:
            continue

        visited.add(city)
        if city == goal:
            return path

        for neighbor, _ in romania_map[city]:
            if neighbor not in visited:
                heapq.heappush(priority_queue, (straight_line_distance_to_bucharest[neighbor], neighbor, path + [neighbor]))
    return []

# Define the A* Search Algorithm:
# Using the Straight Line Distance to Bucharest as heuristic
def a_star_search(start, goal):
    priority_queue = [(straight_line_distance_to_bucharest[start], 0, start, [start])]
    visited = set()
    
    while priority_queue:
        _, cost, city, path = heapq.heappop(priority_queue)
        if city in visited:
            continue
        
        visited.add(city)
        if city == goal:
            return path
        
        for neighbor, distance in romania_map[city]:
            if neighbor not in visited:
                g_cost = cost + distance
                f_cost = g_cost + straight_line_distance_to_bucharest[neighbor]
                heapq.heappush(priority_queue, (f_cost, g_cost, neighbor, path + [neighbor]))
    return []

# To Display our path taken results in a neat manner
def format_path(path):
    return ' -> '.join(path)

# Now we need to compare the time each Search Algorithm takes to find the path from Arad to Buchacrest
def compare_algorithms(start, goal):
    algorithms = [bfs, dfs, best_first_search, a_star_search]
    names = ["BFS", "DFS", "Best-First", "A*"]
    results = []
    for name, algorithm in zip(names, algorithms):
        start_time = time.perf_counter_ns()
        path = algorithm(start, goal)
        end_time = time.perf_counter_ns()
        elapsed_time_ns = end_time - start_time
        results.append([name, elapsed_time_ns, format_path(path)])
    
    # Print the results in table format using tabulate
    print(tabulate(results, headers=["Algorithm", "Time (nanoseconds)", "Path"], tablefmt="grid"))

# Run the comparison
# List of available cities (excluding Bucharest)
available_cities = list(romania_map.keys())
available_cities.remove('Bucharest')

# Select 4 random cities
random_cities = random.sample(available_cities, 4)

# Run the comparison for each city to Bucharest
compare_algorithms('Arad', 'Bucharest')
for city in random_cities:
    compare_algorithms(city, 'Bucharest')