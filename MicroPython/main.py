import network
import socket
from time import sleep
import ujson

# --- Constants ---
SSID = "EBAV & Kwesties 2.4"
PASSWORD = "BestuurMiddelhuis55"
HOST = ''
PORT = 8888
BUFFER_SIZE = 1024

# --- Graph for Dijkstra ---
graph = {
    'A': {'E': 1},
    'B': {'F': 1},
    'C': {'G': 1},
    'D': {'H': 1},
    'E': {'A': 1, 'J': 2, 'F': 1},
    'F': {'E': 1, 'B': 1, 'G': 1},
    'G': {'F': 1, 'C': 1, 'H': 1},
    'H': {'G': 1, 'D': 1, 'O': 2},
    'I': {'K': 2, 'J': 5},
    'J': {'I': 5, 'L': 1, 'E': 2},
    'K': {'M': 1, 'L': 5, 'I': 2},
    'L': {'N': 1, 'J': 1, 'K': 5},
    'M': {'W': 2, 'N': 5, 'K': 1},
    'N': {'P': 1, 'M': 5, 'L': 1, 'O': 5},
    'O': {'Q': 1, 'N': 5, 'H': 2},
    'P': {'S': 1, 'Q': 5, 'N': 1},
    'Q': {'R': 1, 'O': 1, 'P': 5},
    'R': {'S': 5, 'Q': 1},
    'S': {'T': 2, 'P': 1, 'R': 5},
    'T': {'U': 1, 'AA': 1, 'S': 2},
    'U': {'V': 1, 'Z': 1, 'T': 1},
    'V': {'W': 1, 'Y': 1, 'U': 1},
    'W': {'X': 1, 'V': 1, 'M': 2},
    'X': {'W': 1},
    'Y': {'V': 1},
    'Z': {'U': 1},
    'AA': {'T': 1},
}

def dijkstra(graph, start, goal):
    """
    Compute the shortest path using Dijkstra's algorithm.

    Args:
        graph (dict): A dictionary representing the graph where keys are node names
                      and values are dictionaries of neighboring nodes with edge weights.
        start (str): The starting node.
        goal (str): The goal node.

    Returns:
        list: A list of nodes representing the shortest path from start to goal, or [] when unreachable.
    """
    # Initialize all distances as infinity
    shortest_distance = {node: float('inf') for node in graph}
    shortest_distance[start] = 0    # Distance to start node is 0
    predecessor = {}                # To reconstruct the path
    unseen_nodes = dict(graph)      # Nodes yet to be visited

    # Main loop: visit the closest unseen node each time
    while unseen_nodes:
        # Select the node with the smallest known distance
        min_node = min(unseen_nodes, key=shortest_distance.get)
        if shortest_distance[min_node] == float('inf'):
            break  # Remaining nodes are unreachable

        # Update distances to neighbors
        for neighbour, weight in graph[min_node].items():
            new_distance = weight + shortest_distance[min_node]
            if new_distance < shortest_distance[neighbour]:
                shortest_distance[neighbour] = new_distance
                predecessor[neighbour] = min_node  # Track the best path
        unseen_nodes.pop(min_node)  # Mark node as visited

    # Reconstruct the shortest path from start to goal
    path = []
    node = goal
    while node != start:
        if node not in predecessor:
            return []  # No path found
        path.insert(0, node)  # Insert node at the beginning
        node = predecessor[node]
    path.insert(0, start)  # Add the start node
    return path

def handle_client(client):
    """
    Handle incoming requests from the client socket.
    """
    buffer = b""  # Buffer to accumulate incoming bytes
    while True:
        try:
            data = client.recv(BUFFER_SIZE)  # Receive data from the client
            if not data:
                break  # Connection closed by client
            
            # Add received data to buffer
            buffer += data  

            # Process complete lines in the buffer
            while b'\n' in buffer:  
                line, buffer = buffer.split(b'\n', 1)  # Split at newline
                msg_str = line.decode('utf-8').strip()  # Decode and strip whitespace
                if msg_str.startswith('{') and msg_str.endswith('}'):  # Check for JSON object
                    try:
                        command = ujson.loads(msg_str)  # Parse JSON command
                        start = command['start']  
                        goal = command['goal']   
                        
                        path = dijkstra(graph, start, goal) 
                        
                        print("Dijkstra from", start, "to", goal, ":", path)
                        
                        client.send((ujson.dumps({'path': path}) + '\n').encode())
                    
                    except Exception as e:
                        client.send((ujson.dumps({'error': str(e)}) + '\n').encode()) 

        except Exception as e:
            print("Socket error:", e) 
            break

if __name__ == "__main__":
    print("Connecting to Wi-Fi...")
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(SSID, PASSWORD)
    while not wlan.isconnected():
        sleep(0.5)
    print("Connected to Wi-Fi. IP:", wlan.ifconfig()[0])

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind((HOST, PORT))
    server.listen(1)
    print("Waiting for PC connection...")
    client, addr = server.accept()
    print("Connected by", addr)
    client.settimeout(0.1)

    try:
        handle_client(client)
    finally:
        client.close()
        server.close()
        print("Server closed.")