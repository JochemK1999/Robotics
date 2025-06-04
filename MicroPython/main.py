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
    'A': {'E': 21},
    'B': {'F': 21},
    'C': {'G': 21},
    'D': {'H': 21},
    'E': {'A': 21, 'J': 13, 'F': 12},
    'F': {'E': 12, 'B': 21, 'G': 12},
    'G': {'F': 12, 'C': 21, 'H': 12},
    'H': {'G': 12, 'D': 21, 'O': 34},
    'I': {'K': 19, 'J': 49},
    'J': {'I': 49, 'L': 19, 'E': 130},
    'K': {'M': 13, 'L': 72, 'I': 19},
    'L': {'N': 13, 'J': 19, 'K': 72},
    'M': {'W': 34, 'N': 72, 'K': 13},
    'N': {'P': 12, 'M': 72, 'L': 13, 'O': 49},
    'O': {'Q': 13, 'N': 49, 'H': 34},
    'P': {'S': 13, 'Q': 72, 'N': 12},
    'Q': {'R': 19, 'O': 8, 'P': 72},
    'R': {'S': 49, 'Q': 19},
    'S': {'T': 13, 'P': 13, 'R': 49},
    'T': {'U': 12, 'AA': 12, 'S': 12},
    'U': {'V': 12, 'Z': 21, 'T': 12},
    'V': {'W': 12, 'Y': 12, 'U': 12},
    'W': {'X': 12, 'V': 12, 'M': 34},
    'X': {'W': 12},
    'Y': {'V': 12},
    'Z': {'U': 21},
    'AA': {'T': 12},
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