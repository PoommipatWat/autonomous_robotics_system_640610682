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

    def __init__(self, map_data, start, goal, robot_radius, max_iterations=5000, max_travel_distance=0.1, search_radius=0.1, goal_radius=10.0):
        self.map = map_data
        self.start = np.array(start)
        self.goal = np.array(goal)
        self.robot_radius = robot_radius
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
        if x < self.robot_radius or y < self.robot_radius or x >= self.map.shape[1] - self.robot_radius or y >= self.map.shape[0] - self.robot_radius:
            return False
        return np.all(self.map[y-self.robot_radius:y+self.robot_radius+1, x-self.robot_radius:x+self.robot_radius+1] == 0)

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

# def plan_path(map_data, start, goal, robot_radius, max_iterations=5000, max_travel_distance=5.0, search_radius=10.0, goal_radius=2.0):
#     rrt_star = RRTStar(map_data, start, goal, robot_radius, max_iterations, max_travel_distance, search_radius, goal_radius)
#     path = rrt_star.build()
#     return path, rrt_star

# def plot_results(map_data, start, goal, path, rrt_star):
#     fig, ax = plt.subplots(figsize=(8, 8))
    
#     ax.imshow(map_data, cmap='binary')
    
#     start_circle = Circle(start, rrt_star.robot_radius, color='g', fill=False)
#     goal_circle = Circle(goal, rrt_star.robot_radius, color='r', fill=False)
#     ax.add_artist(start_circle)
#     ax.add_artist(goal_circle)
#     ax.plot(start[0], start[1], 'go', markersize=8, label='Start')
#     ax.plot(goal[0], goal[1], 'ro', markersize=8, label='Goal')
    
#     for node in rrt_star.nodes:
#         if node.parent:
#             ax.plot([node.position[0], node.parent.position[0]], 
#                     [node.position[1], node.parent.position[1]], 'c-', alpha=0.3)
    
#     if path is not None:
#         path = np.array(path)
#         ax.plot(path[:, 0], path[:, 1], 'b-', linewidth=2, label='Path')
        
#         for point in path[::5]:  # Plot every 5th point to reduce clutter
#             robot_circle = Circle(point, rrt_star.robot_radius, color='b', alpha=0.2)
#             ax.add_artist(robot_circle)
    
#     ax.set_xlim(0, map_data.shape[1])
#     ax.set_ylim(map_data.shape[0], 0)
#     ax.set_title("RRT* Path Planning with Robot Size")
#     ax.legend()
#     plt.show()

# # Example usage:
# if __name__ == "__main__":
#     map_size = 100  # Reduced map size
#     map_data = np.zeros((map_size, map_size), dtype=int)
    
#     # Add obstacles
#     map_data[20:30, 20:80] = 1  # Horizontal obstacle
#     map_data[40:90, 40:50] = 1  # Vertical obstacle
#     map_data[70:80, 70:90] = 1  # Small obstacle

#     start = (10, 10)
#     goal = (90, 90)
#     robot_radius = 5  # Reduced robot radius

#     start_time = time.time()
#     path, rrt_star = plan_path(map_data, start, goal, robot_radius)
#     end_time = time.time()

#     if path:
#         print(f"Path found with {len(path)} waypoints")
#     else:
#         print("No path found")

#     print(f"Execution time: {end_time - start_time:.2f} seconds")

#     plot_results(map_data, start, goal, path, rrt_star)