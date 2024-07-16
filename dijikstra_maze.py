import heapq
from pyamaze import maze, agent, COLOR

def dijikstra_maze(m, start, end):
    class Edge:
        def __init__(self, weight, start_vertex, target_vertex):
            self.weight = weight
            self.start_vertex = start_vertex
            self.target_vertex = target_vertex

    class Node: 
        def __init__(self, name, x, y):
            self.name = name
            self.x = x
            self.y = y
            self.visited = False
            self.predecessor = None
            self.neighbors = []
            self.min_distance = float("inf")
            
        def __lt__(self, other_node):
            return self.min_distance < other_node.min_distance
            
        def add_edge(self, weight, destination_vertex):
            edge = Edge(weight, self, destination_vertex)
            self.neighbors.append(edge)

    class Dijkstra:
        def __init__(self):
            self.heap = []
            
        def calculate(self, start_vertex):
            start_vertex.min_distance = 0
            heapq.heappush(self.heap, start_vertex)
                
            while self.heap:
                actual_vertex = heapq.heappop(self.heap)
                if actual_vertex.visited:
                    continue
                for edge in actual_vertex.neighbors:
                    start = edge.start_vertex
                    target = edge.target_vertex
                    new_distance = start.min_distance + edge.weight
                    if new_distance < target.min_distance:
                        target.min_distance = new_distance
                        target.predecessor = start
                        heapq.heappush(self.heap, target)
                actual_vertex.visited = True        
            
        def get_shortest_path(self, target_vertex):
            path = []
            actual_vertex = target_vertex
            while actual_vertex is not None:        
                path.append((actual_vertex.x, actual_vertex.y))
                actual_vertex = actual_vertex.predecessor
            path.reverse()
            return path

    
    nodes = {}
    for x in range(1, m.rows + 1):
        for y in range(1, m.cols + 1):
            nodes[(x, y)] = Node(f"({x},{y})", x, y)
            
    for cell in m.maze_map:
        for direction in m.maze_map[cell]:
            if m.maze_map[cell][direction]:
                if direction == 'E':
                    neighbor = (cell[0], cell[1] + 1)
                elif direction == 'W':
                    neighbor = (cell[0], cell[1] - 1)
                elif direction == 'N':
                    neighbor = (cell[0] - 1, cell[1])
                elif direction == 'S':
                    neighbor = (cell[0] + 1, cell[1])
                nodes[cell].add_edge(1, nodes[neighbor])
    
    
    algo = Dijkstra()
    algo.calculate(nodes[start])
    path = algo.get_shortest_path(nodes[end])

    return path

if __name__ == '__main__':
    myMaze = maze(6, 6)
    myMaze.CreateMaze(loopPercent=100)

    start = (myMaze.rows, myMaze.cols)
    end = (1, 1)

    path = dijikstra_maze(myMaze, start, end)
    
    a = agent(myMaze, *start, footprints=True, color=COLOR.red)
    myMaze.tracePath({a: path}, delay=100)

    myMaze.run()
