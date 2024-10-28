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
        
        # แยก update factor สำหรับสิ่งกีดขวางและพื้นที่ว่าง
        self.obstacle_factor = 0.8  # ปรับให้เพิ่มช้าลงเมื่อเจอสิ่งกีดขวาง
        self.free_factor = 0.3      # ปรับให้ลดช้าลงสำหรับพื้นที่ว่าง
        
        # เพิ่มตัวนับจำนวนครั้งที่อัพเดท
        self.update_counts = np.zeros((self.grid_height, self.grid_width))
        self.max_confidence = 0.99
        self.min_confidence = 0.01

    def world_to_grid(self, x, y):
        gx = int((x - self.x_min) / self.cell_size)
        gy = int((y - self.y_min) / self.cell_size)
        return min(max(gx, 0), self.grid_width - 1), min(max(gy, 0), self.grid_height - 1)

    def update_line(self, x0, y0, x1, y1):
        gx0, gy0 = self.world_to_grid(x0, y0)
        gx1, gy1 = self.world_to_grid(x1, y1)
        cells = self.bresenham.bresenham(gx0, gy0, gx1, gy1)
        
        # ตรวจสอบระยะทางระหว่างจุดเริ่มต้นและจุดสิ้นสุด
        dist = np.sqrt((x1-x0)**2 + (y1-y0)**2)
        confidence = 1.0 / (1.0 + dist/5.0)  # ลดความน่าเชื่อถือตามระยะทาง
        
        for x, y in cells:
            gx, gy = int(x), int(y)
            if 0 <= gx < self.grid_width and 0 <= gy < self.grid_height:
                if gx == gx1 and gy == gy1:  # Last cell (potential obstacle)
                    self.update_cell(gy, gx, True, confidence)
                else:  # Free space
                    self.update_cell(gy, gx, False, confidence)

    def update_cell(self, y, x, is_obstacle, confidence):
        current_prob = self.grid[y, x]
        update_count = self.update_counts[y, x]
        
        # ปรับ update factor ตามจำนวนครั้งที่อัพเดท
        decay = 1.0 / (1.0 + 0.1 * update_count)
        
        if is_obstacle:
            factor = self.obstacle_factor * confidence * decay
            new_prob = current_prob + factor * (1 - current_prob)
        else:
            factor = self.free_factor * confidence * decay
            new_prob = current_prob - factor * current_prob
        
        # เพิ่มจำนวนครั้งที่อัพเดท
        self.update_counts[y, x] += 1
        
        # ป้องกันค่าสุดขีด
        if is_obstacle:
            new_prob = min(max(new_prob, current_prob), self.max_confidence)
        else:
            new_prob = max(min(new_prob, current_prob), self.min_confidence)
            
        self.grid[y, x] = new_prob

    def get_map(self):
        return self.grid