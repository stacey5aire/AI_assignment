#AMPAIRE STACEY NUWAGABA 22/U/2947/PS

#implement the search strategies using for tree search

#Depth First Search
def dfs_tree_search(graph, start, goal):
    stack = [(start, [start], 0)]  # Stack contains (current state, path, path cost)
    visited = set()  # Keep track of visited states
    
    while stack:
        current_state, path, path_cost = stack.pop()
        visited.add(current_state)
        
        if current_state == goal:
            return path
        
        successors = graph[current_state]
        for successor, edge_cost in successors.items():
            if successor not in visited:
                new_path = path + [successor]
                new_path_cost = path_cost + edge_cost
                stack.append((successor, new_path, new_path_cost))
    
    return None  # Goal not found

# Define the state space graph
graph = {
    'S': {'A': 3, 'B': 1},
    'A': {'B': 2, 'C': 2},
    'B': {'C': 3},
    'C': {'D': 4, 'G': 4},
    'D': {'G': 1},
    'G': {}
}

start_state = 'S'
goal_state = 'G'

path = dfs_tree_search(graph, start_state, goal_state)

if path:
    print("DFS Path from", start_state, "to", goal_state, ":", path)
else:
    print("No path found from", start_state, "to", goal_state)

#Breadth First Search
from collections import deque

def bfs_tree_search(graph, start, goal):
    queue = deque([(start, [start])])  # Queue contains (current state, path)
    visited = set()  # Keep track of visited states
    
    while queue:
        current_state, path = queue.popleft()
        visited.add(current_state)
        
        if current_state == goal:
            return path
        
        successors = graph[current_state]
        for successor, _ in successors.items():
            if successor not in visited:
                new_path = path + [successor]
                queue.append((successor, new_path))
    
    return None  # Goal not found

# Define the state space graph
graph = {
    'S': {'A': 3, 'B': 1},
    'A': {'B': 2, 'C': 2},
    'B': {'C': 3},
    'C': {'D': 4, 'G': 4},
    'D': {'G': 1},
    'G': {}
}

start_state = 'S'
goal_state = 'G'

path = bfs_tree_search(graph, start_state, goal_state)

if path:
    print("BFS Path from", start_state, "to", goal_state, ":", path)
else:
    print("No path found from", start_state, "to", goal_state)
#Uniform Cost Search 
import heapq

def ucs_tree_search(graph, start, goal):
    priority_queue = [(0, start, [start])]  # Priority queue contains (priority, current state, path)
    visited = set()  # Keep track of visited states
    
    while priority_queue:
        priority, current_state, path = heapq.heappop(priority_queue)
        
        if current_state == goal:
            return path
        
        if current_state in visited:
            continue
        
        visited.add(current_state)
        
        successors = graph[current_state]
        for successor, edge_cost in successors.items():
            if successor not in visited:
                new_path = path + [successor]
                new_priority = priority + edge_cost
                heapq.heappush(priority_queue, (new_priority, successor, new_path))
    
    return None  # Goal not found

# Define the state space graph
graph = {
    'S': {'A': 3, 'B': 1},
    'A': {'B': 2, 'C': 2},
    'B': {'C': 3},
    'C': {'D': 4, 'G': 4},
    'D': {'G': 1},
    'G': {}
}

start_state = 'S'
goal_state = 'G'

path = ucs_tree_search(graph, start_state, goal_state)

if path:
    print("UCS Tree Search Path from", start_state, "to", goal_state, ":", path)
else:
    print("No path found from", start_state, "to", goal_state)
#Greedy Search
#Greedy Search selects the node that appears closest to the goal based on a heuristic.
def greedy_tree_search(graph, start, goal, heuristic):
    priority_queue = [(heuristic[start], start, [start])]  # Priority queue contains (heuristic, current state, path)
    visited = set()  # Keep track of visited states
    
    while priority_queue:
        _, current_state, path = priority_queue.pop(0)
        
        if current_state == goal:
            return path
        
        if current_state in visited:
            continue
        
        visited.add(current_state)
        
        successors = graph[current_state]
        for successor, _ in successors.items():
            if successor not in visited:
                new_path = path + [successor]
                priority = heuristic[successor]
                priority_queue.append((priority, successor, new_path))
        
        # Sort the priority queue based on heuristic value
        priority_queue.sort(key=lambda x: x[0])
    
    return None  # Goal not found

# Define the state space graph
graph = {
    'S': {'A': 3, 'B': 1},
    'A': {'B': 2, 'C': 2},
    'B': {'C': 3},
    'C': {'D': 4, 'G': 4},
    'D': {'G': 1},
    'G': {}
}

start_state = 'S'
goal_state = 'G'

# Define heuristic values for each state
heuristic_values = {
    'S': 7,
    'A': 5,
    'B': 7,
    'C': 4,
    'D': 1,
    'G': 0
}

path = greedy_tree_search(graph, start_state, goal_state, heuristic_values)

if path:
    print("Greedy Tree Search Path from", start_state, "to", goal_state, ":", path)
else:
    print("No path found from", start_state, "to", goal_state)
    
#A* search
#A* Search combines the cost-so-far (g-value) and a heuristic estimate of the remaining cost (h-value) to determine which node to explore next.
import heapq

def astar_tree_search(graph, start, goal, heuristic):
    priority_queue = [(heuristic[start], start, [start], 0)]  # Priority queue contains (f-value, current state, path, g-value)
    visited = set()  # Keep track of visited states
    
    while priority_queue:
        _, current_state, path, g_value = heapq.heappop(priority_queue)
        
        if current_state == goal:
            return path
        
        if current_state in visited:
            continue
        
        visited.add(current_state)
        
        successors = graph[current_state]
        for successor, edge_cost in successors.items():
            if successor not in visited:
                new_path = path + [successor]
                new_g_value = g_value + edge_cost
                f_value = new_g_value + heuristic[successor]
                heapq.heappush(priority_queue, (f_value, successor, new_path, new_g_value))
    
    return None  # Goal not found

# Define the state space graph
graph = {
    'S': {'A': 3, 'B': 1},
    'A': {'B': 2, 'C': 2},
    'B': {'C': 3},
    'C': {'D': 4, 'G': 4},
    'D': {'G': 1},
    'G': {}
}

start_state = 'S'
goal_state = 'G'

# Define heuristic values for each state
heuristic_values = {
    'S': 7,
    'A': 5,
    'B': 7,
    'C': 4,
    'D': 1,
    'G': 0
}

path = astar_tree_search(graph, start_state, goal_state, heuristic_values)

if path:
    print("A* Tree Search Path from", start_state, "to", goal_state, ":", path)
else:
    print("No path found from", start_state, "to", goal_state)
