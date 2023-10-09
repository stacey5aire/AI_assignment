#AMPAIRE STACEY NUWAGABA 22/U/2947/PS
# Define the state space graph as a nested dictionary
state_space_graph = {
    'S': {'h': 7, 'neighbors': {'A': 3, 'B': 1}},
    'A': {'h': 5, 'neighbors': {'B': 2, 'C': 2}},
    'B': {'h': 7, 'neighbors': {'C': 3}},
    'C': {'h': 4, 'neighbors': {'D': 4, 'G': 4}},
    'D': {'h': 1, 'neighbors': {'G': 1}},
    'G': {'h': 0, 'neighbors': {}}
}

# Example path costs:
# S to A: 3
# S to B: 1
# A to B: 2
# A to C: 2
# B to C: 3
# C to D: 4
# C to G: 4
# D to G: 1

# Access heuristic value of a state 'A':
heuristic_A = state_space_graph['A']['h']
print(f"Heuristic value of A: {heuristic_A}")

# Access neighbors and path costs of a state 'S':
neighbors_S = state_space_graph['S']['neighbors']
print(f"Neighbors of S and their path costs: {neighbors_S}")


#2nd opt
# Initialize an empty dictionary for the graph
graph = {}

# Add the nodes to the graph
graph["S"] = {"A", "B"}
graph["A"] = {"B", "C"}
graph["B"] = {"C"}
graph["C"] = {"D", "G"}
graph["D"] = {"G"}

# Initialize an empty set for edges and add the edges to it
edges = set()
edges.add(("S", "A", 3))
edges.add(("S", "B", 1))
edges.add(("A", "B", 2))
edges.add(("A", "C", 2))
edges.add(("B", "C", 3))
edges.add(("C", "D", 4))
edges.add(("C", "G", 4))
edges.add(("D", "G", 1))

# Define heuristic values for each node
heuristic_values = {
    "S": 7,
    "A": 5,
    "B": 7,
    "C": 4,
    "D": 1,
    "G": 0
}

# Print the graph, edges, and heuristic values
print("Graph:", graph)
print("Edges:", edges)
print("Heuristic Values:", heuristic_values)
