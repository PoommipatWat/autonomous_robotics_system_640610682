import numpy as np
import time
from .Bresenham import Bresenham

class OccupancyGridMap:
    def __init__(self, x_min, x_max, y_min, y_max, cell_size=1.0):
        self.cell_size = cell_size
        self.x_min, self.x_max = x_min, x_max
        self.y_min, self.y_max = y_min, y_max
        self.grid_width = int(np.ceil((x_max - x_min) / cell_size))
        self.grid_height = int(np.ceil((y_max - y_min) / cell_size))
        self.grid = np.full((self.grid_height, self.grid_width), 0.5)  # Initialize with unknown
        self.bresenham = Bresenham()
        self.update_factor = 0.1  # Factor for updating probabilities

    def world_to_grid(self, x, y):
        gx = int((x - self.x_min) / self.cell_size)
        gy = int((y - self.y_min) / self.cell_size)
        return min(max(gx, 0), self.grid_width - 1), min(max(gy, 0), self.grid_height - 1)

    def update_line(self, x0, y0, x1, y1):
        gx0, gy0 = self.world_to_grid(x0, y0)
        gx1, gy1 = self.world_to_grid(x1, y1)
        cells = self.bresenham.bresenham(gx0, gy0, gx1, gy1)
        
        for x, y in cells:
            gx, gy = int(x), int(y)  # Ensure integer indices
            if 0 <= gx < self.grid_width and 0 <= gy < self.grid_height:
                if gx == gx1 and gy == gy1:  # Last cell (potential obstacle)
                    self.update_cell(gy, gx, True)
                else:  # Free space
                    self.update_cell(gy, gx, False)

    def update_cell(self, y, x, is_obstacle):
        current_prob = self.grid[y, x]
        if is_obstacle:
            new_prob = current_prob + self.update_factor * (1 - current_prob)
        else:
            new_prob = current_prob - self.update_factor * current_prob
        self.grid[y, x] = np.clip(new_prob, 0.01, 0.99)  # Avoid 0 and 1 for continued updates

    def get_map(self):
        return self.grid