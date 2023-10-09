#AMPAIRE STACEY NUWAGABA 22/U/2947/PS
#implement the search strategies using for graph search
#Depth First Search
def dfs_graph_search(graph, start, goal):
    stack = [(start, [start])]  # Stack contains (current state, path)
    visited = set()  # Keep track of visited states
    
    while stack:
        current_state, path = stack.pop()
        
        if current_state == goal:
            return path
        
        if current_state in visited:
            continue
        
        visited.add(current_state)
        
        successors = graph[current_state]
        for successor, _ in successors.items():
            if successor not in visited:
                new_path = path + [successor]
                stack.append((successor, new_path))
    
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

path = dfs_graph_search(graph, start_state, goal_state)

if path:
    print("DFS Graph Search Path from", start_state, "to", goal_state, ":", path)
else:
    print("No path found from", start_state, "to", goal_state)
    
#Breadth First Search
from collections import deque

def bfs_graph_search(graph, start, goal):
    queue = deque([(start, [start])])  # Queue contains (current state, path)
    visited = set()  # Keep track of visited states
    
    while queue:
        current_state, path = queue.popleft()
        
        if current_state == goal:
            return path
        
        if current_state in visited:
            continue
        
        visited.add(current_state)
        
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

path = bfs_graph_search(graph, start_state, goal_state)

if path:
    print("BFS Graph Search Path from", start_state, "to", goal_state, ":", path)
else:
    print("No path found from", start_state, "to", goal_state)
    
#Uniform Cost Search
def ucs_graph_search(graph, start, goal):
    priority_queue = [(0, start, [start])]  # Priority queue contains (cumulative cost, current state, path)
    visited = set()  # Keep track of visited states
    
    while priority_queue:
        cumulative_cost, current_state, path = heapq.heappop(priority_queue)
        
        if current_state == goal:
            return path
        
        if current_state in visited:
            continue
        
        visited.add(current_state)
        
        successors = graph[current_state]
        for successor, edge_cost in successors.items():
            if successor not in visited:
                new_path = path + [successor]
                new_cumulative_cost = cumulative_cost + edge_cost
                heapq.heappush(priority_queue, (new_cumulative_cost, successor, new_path))
    
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

path = ucs_graph_search(graph, start_state, goal_state)

if path:
    print("UCS Graph Search Path from", start_state, "to", goal_state, ":", path)
else:
    print("No path found from", start_state, "to", goal_state)
    
    #Greedy Search 
import heapq

def greedy_graph_search(graph, start, goal, heuristic):
    priority_queue = [(heuristic[start], start, [start])]  # Priority queue contains (heuristic, current state, path)
    visited = set()  # Keep track of visited states
    
    while priority_queue:
        _, current_state, path = heapq.heappop(priority_queue)
        
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
                heapq.heappush(priority_queue, (priority, successor, new_path))
    
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

path = greedy_graph_search(graph, start_state, goal_state, heuristic_values)

if path:
    print("Greedy Graph Search Path from", start_state, "to", goal_state, ":", path)
else:
    print("No path found from", start_state, "to", goal_state)
    
    
#A* Search 
#astar_graph_search performs A* Search starting from the start_state and searching for the goal_state in the state space graph graph. It uses a priority queue (implemented using a min-heap) to explore the graph structure while considering both the cost-so-far (g-value) and the heuristic estimate of the remaining cost (h-value). A* Search selects the node with the lowest f-value (f = g + h) as it aims to find the optimal path. When the goal state is found, it returns the path from the start state to the goal state, which represents the optimal path in terms of both cost and heuristic estimation.
import heapq

def astar_graph_search(graph, start, goal, heuristic):
    priority_queue = [(heuristic[start], 0, start, [start])]  # Priority queue contains (heuristic, g-value, current state, path)
    visited = set()  # Keep track of visited states
    
    while priority_queue:
        _, g_value, current_state, path = heapq.heappop(priority_queue)
        
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
                heapq.heappush(priority_queue, (f_value, new_g_value, successor, new_path))
    
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

path = astar_graph_search(graph, start_state, goal_state, heuristic_values)

if path:
    print("A* Graph Search Path from", start_state, "to", goal_state, ":", path)
else:
    print("No path found from", start_state, "to", goal_state)


