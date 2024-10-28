import numpy as np
import time
import matplotlib.pyplot as plt
from matplotlib.patches import Circle

class RRTStar:
    class Node:
        def __init__(self, position):
            self.position = position
            self.parent = None
            self.cost = 0

    def __init__(self, map_data, start, goal, max_iterations=5000, max_travel_distance=0.1, search_radius=0.1, goal_radius=10.0):
        self.map = map_data
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.x_range = (0, self.map.shape[1])
        self.y_range = (0, self.map.shape[0])
        self.max_iterations = max_iterations
        self.max_travel_distance = max_travel_distance
        self.search_radius = search_radius
        self.goal_radius = goal_radius
        
        self.nodes = []
        self.node_positions = np.array([self.start])
        self.best_path = None
        self.best_cost = float('inf')

    def is_collision(self, p1, p2):
        x1, y1 = p1.astype(int)
        x2, y2 = p2.astype(int)
        
        if not self.is_point_valid(x1, y1) or not self.is_point_valid(x2, y2):
            return True

        dx = x2 - x1
        dy = y2 - y1
        distance = np.sqrt(dx**2 + dy**2)
        
        if distance == 0:
            return False

        steps = max(abs(dx), abs(dy))
        x_step = dx / steps
        y_step = dy / steps

        for i in range(1, int(steps)):
            x = int(x1 + i * x_step)
            y = int(y1 + i * y_step)
            if not self.is_point_valid(x, y):
                return True
        return False

    def is_point_valid(self, x, y):
        if x < 0 or y < 0 or x >= self.map.shape[1] or y >= self.map.shape[0]:
            return False
        return self.map[y, x] == 0

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

        if self.best_path is not None:
            return self.best_path
        else:
            return None

    def get_path(self, end_node):
        path = []
        current = end_node
        while current is not None:
            path.append(current.position)
            current = current.parent
        return path[::-1]