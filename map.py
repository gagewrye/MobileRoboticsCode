import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
import matplotlib.colors as mcolors
import numpy as np
from typing import Tuple

class OccupancyMap():
    def __init__(self, width=10, height=10, location=(0, 0)):
        self.occupied = dict()
        self.traveled = dict()
        self.location = location
        self.width = width
        self.height = height
    
    def add_occupancy(self, x, y):
        self.occupied[(x, y)] = True

    def check_occupancy(self, x, y):
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            return True # Occupied if outside the map
        return (x, y) in self.occupied
    
    def get_location(self) -> Tuple:
        return self.location
    def set_location(self, x, y):
        self.location = (x, y)
    def travel_to(self, x, y):
        self.traveled[self.location] = (x, y)
        self.location = (x, y)

    def bfs(self, start, goal) -> list[Tuple]:
        queue = [start]
        visited = set()
        parent = dict()
        found = False

        # Cardinal directions (up, down, left, right)
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]

        while queue:
            current = queue.pop(0)
            visited.add(current)
            if current == goal:
                found = True
                break
            x, y = current
            for dx, dy in directions:
                new_x, new_y = x + dx, y + dy
                if not self.check_occupancy(new_x, new_y) and (new_x, new_y) not in visited:
                    queue.append((new_x, new_y))
                    visited.add((new_x, new_y))  # Mark as visited when queued
                    parent[(new_x, new_y)] = current

        if not found:
            return None

        # Reconstruct the path
        path = [goal]
        while path[-1] != start:
            path.append(parent[path[-1]])
        path.reverse()
        return path

    def show(self):
        # Create a grid
        occupied = np.zeros((self.height, self.width))
        
        # Mark occupied cells
        for (x, y) in self.occupied.keys():
            occupied[self.height - 1 - y, x] = 1  # Flip y-axis

        # Create a custom colormap
        colors = [(1, 1, 1), (0.7, 0.7, 0.7)]  # White for unoccupied, Gray for occupied
        cmap = mcolors.LinearSegmentedColormap.from_list("custom_gray", colors, N=2)

        # Plot the grid
        plt.figure(figsize=(8, 8))
        plt.imshow(occupied, cmap=cmap, origin='upper', extent=(0, self.width, 0, self.height))
        
        # Draw traveled path
        ax = plt.gca()
        
        # Fill the starting square with red
        (x,y), _ = next(iter(self.traveled.items()))
        rect = plt.Rectangle((x, y), 1, 1, color='red', alpha=0.6)
        ax.add_patch(rect)
        # Start with a red dot
        ax.plot(x + 0.5, y + 0.5, 'ro', markersize=5)
        # Fill the final destination square with green
        (x,y), (dest_x, dest_y) = next(iter(reversed(self.traveled.items())))
        rect = plt.Rectangle((dest_x, dest_y), 1, 1, color='green', alpha=0.6)
        ax.add_patch(rect)
        # Add arrows to indicate direction
        ax.arrow(x + 0.5, y + 0.5, dest_x - x, dest_y - y, head_width=0.2, head_length=0.2, fc='red', ec='red')

        # Draw lines from (x, y) to (dest_x, dest_y)
        for (x, y), (dest_x, dest_y) in self.traveled.items():
            ax.plot(
                [x + 0.5, dest_x + 0.5],
                [y + 0.5, dest_y + 0.5],
                color='red', linewidth=2, linestyle='-', zorder=2
        )

        # Configure gridlines
        ax = plt.gca()
        ax.set_xticks(range(self.width + 1), minor=True)  # Minor ticks align with all gridlines
        ax.set_yticks(range(self.height + 1), minor=True)
        ax.set_xticks(range(0, self.width + 1, max(1, self.width // 10)))  # Major ticks for labels
        ax.set_yticks(range(0, self.height + 1, max(1, self.height // 10)))

        # Enable gridlines
        ax.grid(which='minor', color='black', linestyle='-', linewidth=0.5)  # Minor gridlines
        ax.grid(which='major', color='black', linestyle='-', linewidth=0.8)  # Major gridlines
        ax.set_aspect('equal', adjustable='box')

        # Set integer-based mouse-over coordinate display
        ax.xaxis.set_major_formatter(ticker.FuncFormatter(lambda val, pos: f'{int(val)}'))
        ax.yaxis.set_major_formatter(ticker.FuncFormatter(lambda val, pos: f'{int(val)}'))

        # Label axes
        plt.xlabel('X')
        plt.ylabel('Y')
        plt.title('Occupancy GridMap')
        plt.show()

    def build_edmunds105(self, location=(15, 2)):
        """
        Builds the occupancy map for Edmunds 105 and places the robot at (15, 2).
        """
        self.height = 11
        self.width = 21
        self.location = location
        self.occupied = dict()
        self.traveled = dict()
        
        # Add occupied points
        # Perimeter
        for i in range(0, 21):
            self.add_occupancy(i, 0)
            self.add_occupancy(i, 10)
        # Long Desk
        for i in range(3, 14):
            self.add_occupancy(i, 9)
            self.add_occupancy(i, 8)
        # Wall
        for i in range(0, 7):
            self.add_occupancy(20, i)
            self.add_occupancy(19, i)
            self.add_occupancy(18, i)
        for i in range(1, 10):
            self.add_occupancy(0, i)
            self.add_occupancy(20, i)

        # Desks
        for i in range(1, 6):
            # Desk 1
            self.add_occupancy(1, i)
            self.add_occupancy(2, i)
            # Desk 2
            self.add_occupancy(4, i)
            self.add_occupancy(5, i)
            # Desk 3
            self.add_occupancy(7, i)
            self.add_occupancy(8, i)
            # Desk 4
            self.add_occupancy(10, i)
            self.add_occupancy(11, i)
            self.add_occupancy(12, i)

# Example usage
if __name__ == "__main__":
    m = OccupancyMap()
    m.build_edmunds105()
    current_location = m.get_location()
    path = m.bfs(current_location, (3, 3))
    for x, y in path:
        m.travel_to(x, y)
    # Show the map
    m.show()
