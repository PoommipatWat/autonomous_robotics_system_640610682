import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle
import time

class RRTStar:
    class Node:
        def __init__(self, position):
            self.position = position
            self.parent = None
            self.cost = 0

    def __init__(self, x_range, y_range, start, goal, obstacles, max_iterations=1000, max_travel_distance=1.0, search_radius=2.0, goal_radius=0.5):
        self.x_range = x_range
        self.y_range = y_range
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.obstacles = np.array(obstacles)
        self.max_iterations = max_iterations
        self.max_travel_distance = max_travel_distance
        self.search_radius = search_radius
        self.goal_radius = goal_radius
        
        self.nodes = []
        self.node_positions = np.array([self.start])
        self.best_path = None
        self.best_cost = float('inf')

    def is_collision(self, p1, p2):
        def ccw(A, B, C):
            return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

        def intersect(A, B, C, D):
            return ccw(A,C,D) != ccw(B,C,D) and ccw(A,B,C) != ccw(A,B,D)

        for obs in self.obstacles:
            left, top, width, height = obs
            right, bottom = left + width, top + height
            if (intersect(p1, p2, (left, top), (right, top)) or
                intersect(p1, p2, (right, top), (right, bottom)) or
                intersect(p1, p2, (right, bottom), (left, bottom)) or
                intersect(p1, p2, (left, bottom), (left, top))):
                return True
        return False

    def find_nearest_node(self, point):
        distances = np.sum((self.node_positions - point)**2, axis=1)
        return self.nodes[np.argmin(distances)]

    def steer(self, from_pos, to_pos):
        direction = to_pos - from_pos
        distance = np.linalg.norm(direction)
        if distance > self.max_travel_distance:
            return from_pos + direction / distance * self.max_travel_distance
        return to_pos

    def find_nearby_nodes(self, point):
        distances = np.sum((self.node_positions - point)**2, axis=1)
        return [self.nodes[i] for i in np.where(distances <= self.search_radius**2)[0]]

    def choose_parent(self, point, nearby_nodes):
        best_parent = None
        best_cost = float('inf')
        for node in nearby_nodes:
            potential_cost = node.cost + np.linalg.norm(node.position - point)
            if potential_cost < best_cost and not self.is_collision(node.position, point):
                best_cost = potential_cost
                best_parent = node
        return best_parent

    def rewire(self, new_node, nearby_nodes):
        for node in nearby_nodes:
            new_cost = new_node.cost + np.linalg.norm(new_node.position - node.position)
            if new_cost < node.cost and not self.is_collision(new_node.position, node.position):
                node.parent = new_node
                node.cost = new_cost

    def build(self):
        self.nodes = [self.Node(self.start)]
        self.node_positions = np.array([self.start])

        for _ in range(self.max_iterations):
            random_point = np.random.uniform(self.x_range[0], self.x_range[1], 2)
            nearest_node = self.find_nearest_node(random_point)
            new_point = self.steer(nearest_node.position, random_point)
            
            if not self.is_collision(nearest_node.position, new_point):
                nearby_nodes = self.find_nearby_nodes(new_point)
                new_node = self.Node(new_point)
                parent = self.choose_parent(new_point, nearby_nodes)
                
                if parent:
                    new_node.parent = parent
                    new_node.cost = parent.cost + np.linalg.norm(parent.position - new_point)
                    self.nodes.append(new_node)
                    self.node_positions = np.vstack((self.node_positions, new_point))
                    self.rewire(new_node, nearby_nodes)
                    
                    if np.linalg.norm(new_point - self.goal) <= self.goal_radius:
                        path_cost = new_node.cost + np.linalg.norm(new_point - self.goal)
                        if path_cost < self.best_cost:
                            self.best_cost = path_cost
                            self.best_path = self.get_path(new_node)

        print(f"Finished. Total nodes: {len(self.nodes)}")
        if self.best_path is not None:
            print(f"Best path cost: {self.best_cost}")
        else:
            print("No path found to the goal.")

    def get_path(self, end_node):
        path = []
        current = end_node
        while current is not None:
            path.append(current.position)
            current = current.parent
        return np.array(path[::-1])

    def plot_results(self):
        fig, ax = plt.subplots(figsize=(10, 10))
        ax.set_xlim(self.x_range)
        ax.set_ylim(self.y_range)
        
        for obs in self.obstacles:
            ax.add_patch(Rectangle(obs[:2], obs[2], obs[3], fill=True, color='gray'))
        
        ax.add_artist(Circle(self.goal, self.goal_radius, color='g', fill=False))
        ax.plot(self.start[0], self.start[1], 'go', markersize=10, label='Start')
        ax.plot(self.goal[0], self.goal[1], 'ro', markersize=10, label='Goal')
        
        for node in self.nodes:
            if node.parent:
                ax.plot([node.position[0], node.parent.position[0]], 
                        [node.position[1], node.parent.position[1]], 'k-', linewidth=0.5, alpha=0.5)
        
        if self.best_path is not None:
            ax.plot(self.best_path[:, 0], self.best_path[:, 1], 'b-', linewidth=2, label='Best Path')
        
        ax.legend()
        plt.show()

def create_valid_rrt_star(x_range, y_range, obstacles, max_iterations=2000, max_travel_distance=1.0, search_radius=2.0, goal_radius=0.5):
    while True:
        start = np.random.uniform(x_range[0], x_range[1], 2)
        goal = np.random.uniform(x_range[0], x_range[1], 2)
        
        if not any(obs[0] <= p[0] <= obs[0]+obs[2] and obs[1] <= p[1] <= obs[1]+obs[3] for p in [start, goal] for obs in obstacles):
            print(f"Generated start: {start}, goal: {goal}")
            return RRTStar(x_range, y_range, start, goal, obstacles, max_iterations, max_travel_distance, search_radius, goal_radius)

if __name__ == "__main__":
    x_range = (0, 20)
    y_range = (0, 20)
    obstacles = [
        (5, 5, 3, 10),
        (12, 7, 3, 8),
        (8, 12, 8, 3)
    ]
    max_travel_distance = 2.0
    search_radius = 2.0
    goal_radius = 0.5

    rrt_star = create_valid_rrt_star(x_range, y_range, obstacles, max_iterations=5000, 
                                     max_travel_distance=max_travel_distance, search_radius=search_radius, 
                                     goal_radius=goal_radius)
    
    start_time = time.time()
    rrt_star.build()
    end_time = time.time()

    print(len([i.position for i in rrt_star.nodes]))
    
    execution_time = end_time - start_time
    print(f"Execution time: {execution_time:.2f} seconds")
    
    rrt_star.plot_results()