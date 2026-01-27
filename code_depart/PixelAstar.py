import pygame
import math
import heapq
from Constants import PERCEPTION_RADIUS


class PixelAstar:
    """Pixel-level A* pathfinding for local obstacle avoidance.
    
    This class handles fine-grained navigation around obstacles using
    a pixel-level grid built from perception data.
    """
    
    def __init__(self, maze, player):
        self.maze = maze
        self.player = player
        self.include_full_blocker_tiles = True  # Expand grid to full tiles containing blockers
    
    def make_perception_grid(self, path, path_index):
        """Build perception grid and find pixel-level path.
        
        Args:
            path: Current A* tile path
            path_index: Current index in the path
        
        Returns:
            tuple: (grid, start_pos, target_pos, perception_left, perception_top, CELL_SIZE, target_tile)
            or None if grid cannot be built
        """
        CELL_SIZE = 1  # 1 pixel per grid cell

        # IMPORTANT: match Maze.make_perception_list exactly for the perception rect.
        # pygame.Rect truncates its args to ints.
        perception_distance = PERCEPTION_RADIUS * max(self.maze.tile_size_x, self.maze.tile_size_y)
        perception_left = self.player.x + 0.5 * (self.player.size_x - perception_distance)
        perception_top = self.player.y + 0.5 * (self.player.size_y - perception_distance)
        perception_rect = pygame.Rect(perception_left, perception_top, perception_distance, perception_distance)

        # Blockers are still taken ONLY from perception.
        perception = self.maze.make_perception_list(self.player, None)
        all_blockers = perception[0] + perception[1]  # walls + obstacles

        expanded_rect = perception_rect
        if self.include_full_blocker_tiles and all_blockers:
            tile_rects = []
            for blocker in all_blockers:
                # Walls are aligned to tiles already; obstacles are random-positioned inside a tile.
                cx, cy = blocker.center
                tile_row = int(cy / self.maze.tile_size_y)
                tile_col = int(cx / self.maze.tile_size_x)
                tile_left = tile_col * self.maze.tile_size_x
                tile_top = tile_row * self.maze.tile_size_y
                tile_rects.append(pygame.Rect(tile_left, tile_top, self.maze.tile_size_x, self.maze.tile_size_y))

            if tile_rects:
                min_left = min([expanded_rect.left] + [r.left for r in tile_rects])
                min_top = min([expanded_rect.top] + [r.top for r in tile_rects])
                max_right = max([expanded_rect.right] + [r.right for r in tile_rects])
                max_bottom = max([expanded_rect.bottom] + [r.bottom for r in tile_rects])
                expanded_rect = pygame.Rect(min_left, min_top, max_right - min_left, max_bottom - min_top)

        grid_width = max(1, int(expanded_rect.width // CELL_SIZE))
        grid_height = max(1, int(expanded_rect.height // CELL_SIZE))

        # Create empty grid (all walkable initially)
        grid = [[True for _ in range(grid_width)] for _ in range(grid_height)]

        player_rect = self.player.get_rect()

        def pix_to_col(pixel_x):
            return int(math.floor((pixel_x - expanded_rect.left) / CELL_SIZE))

        def pix_to_row(pixel_y):
            return int(math.floor((pixel_y - expanded_rect.top) / CELL_SIZE))

        def nearest_walkable(start, max_radius=12):
            sr, sc = start
            if 0 <= sr < grid_height and 0 <= sc < grid_width and grid[sr][sc]:
                return start
            for r in range(1, max_radius + 1):
                for dr in range(-r, r + 1):
                    dc = r - abs(dr)
                    for dc_signed in (-dc, dc):
                        rr = sr + dr
                        cc = sc + dc_signed
                        if 0 <= rr < grid_height and 0 <= cc < grid_width and grid[rr][cc]:
                            return (rr, cc)
            return None

        for blocker in all_blockers:
            # Inflate the blocker by the player's collision rect size.
            # This turns the "player rect" problem into a "point" problem.
            inflated_rect = blocker.inflate(player_rect.width, player_rect.height)
            
            # Grid cell range that overlaps with inflated_rect
            start_col = max(0, pix_to_col(inflated_rect.left))
            end_col = min(grid_width - 1, pix_to_col(inflated_rect.right - 1))
            start_row = max(0, pix_to_row(inflated_rect.top))
            end_row = min(grid_height - 1, pix_to_row(inflated_rect.bottom - 1))
            
            # Mark these cells as blocked
            for row in range(start_row, end_row + 1):
                for col in range(start_col, end_col + 1):
                    grid[row][col] = False  # blocked
        
        # Player center in PIXELS (AFTER the blocker loop)
        player_center_x, player_center_y = player_rect.center

        # Convert to GRID indices
        start_col = pix_to_col(player_center_x)
        start_row = pix_to_row(player_center_y)
        start_pos = (start_row, start_col)

        # Find the next tile to target - should be adjacent to current tile
        # Use center-based tile for consistency with pixel grid.
        current_tile = (int(player_center_y / self.maze.tile_size_y), int(player_center_x / self.maze.tile_size_x))
        
        # Look through path from current index to find the next adjacent tile
        target_tile = None
        for i in range(path_index, len(path)):
            candidate = path[i]
            # Check if this tile is adjacent (Manhattan distance = 1)
            manhattan_dist = abs(candidate[0] - current_tile[0]) + abs(candidate[1] - current_tile[1])
            if manhattan_dist == 1:
                target_tile = candidate
                break
            elif manhattan_dist == 0:
                # We're already on this tile, skip to next
                continue
        
        # If no adjacent tile found, use whatever is at path_index
        if target_tile is None:
            if path_index < len(path):
                target_tile = path[path_index]
            else:
                print("ERROR: No valid target tile in path!")
                return None

        # Target selection for pixel A*:
        # We already know the next tile in the global (tile) path. For the local pixel planner,
        # choose the FURTHEST walkable cell in that direction inside the perception square.
        # This gives A* a clear "make progress forward" goal while still allowing it to route
        # around local obstacles.
        drow = target_tile[0] - current_tile[0]
        dcol = target_tile[1] - current_tile[1]

        step_row, step_col = 0, 0
        if drow < 0:
            step_row = -1
        elif drow > 0:
            step_row = 1
        elif dcol < 0:
            step_col = -1
        else:
            step_col = 1

        # Prefer: farthest walkable cell in the intended direction (anywhere on the forward edge).
        # Fallback: if the forward edge is fully blocked, use the farthest walkable cell on the
        # player's aligned ray (same row/col) in that direction.
        start_r, start_c = start_pos

        target_pos = None

        if step_row == 1:  # down
            edge_row = grid_height - 1
            edge_walkables = [(edge_row, c) for c in range(grid_width) if grid[edge_row][c]]
            if edge_walkables:
                target_pos = min(edge_walkables, key=lambda rc: abs(rc[1] - start_c))

            edge_candidate = (edge_row, start_c)
            scan_positions = ((r, start_c) for r in range(grid_height - 1, -1, -1))
        elif step_row == -1:  # up
            edge_row = 0
            edge_walkables = [(edge_row, c) for c in range(grid_width) if grid[edge_row][c]]
            if edge_walkables:
                target_pos = min(edge_walkables, key=lambda rc: abs(rc[1] - start_c))

            edge_candidate = (edge_row, start_c)
            scan_positions = ((r, start_c) for r in range(0, grid_height))
        elif step_col == 1:  # right
            edge_col = grid_width - 1
            edge_walkables = [(r, edge_col) for r in range(grid_height) if grid[r][edge_col]]
            if edge_walkables:
                target_pos = min(edge_walkables, key=lambda rc: abs(rc[0] - start_r))

            edge_candidate = (start_r, edge_col)
            scan_positions = ((start_r, c) for c in range(grid_width - 1, -1, -1))
        else:  # left
            edge_col = 0
            edge_walkables = [(r, edge_col) for r in range(grid_height) if grid[r][edge_col]]
            if edge_walkables:
                target_pos = min(edge_walkables, key=lambda rc: abs(rc[0] - start_r))

            edge_candidate = (start_r, edge_col)
            scan_positions = ((start_r, c) for c in range(0, grid_width))

        # Fallback requested: if nothing walkable exists on the forward edge, use the farthest
        # walkable cell on the aligned ray.
        if target_pos is None:
            for rr, cc in scan_positions:
                if grid[rr][cc]:
                    target_pos = (rr, cc)
                    break

        # Final fallback: if even the whole ray is blocked, find the closest walkable near the edge.
        if target_pos is None:
            target_pos = nearest_walkable(edge_candidate, max_radius=64)
            if target_pos is None:
                print(f"Pixel A*: No walkable target cell near forward edge {edge_candidate} in local perception grid")
                return None

        # Convert the chosen grid target back to a representative pixel (for debug only).
        target_pixel_x = expanded_rect.left + target_pos[1] * CELL_SIZE
        target_pixel_y = expanded_rect.top + target_pos[0] * CELL_SIZE
        
        # DEBUG: Print positions
        player_tile = (int(self.player.y / self.maze.tile_size_y), int(self.player.x / self.maze.tile_size_x))
        print(f"DEBUG Grid Build:")
        print(f"  Current player TILE (game): {player_tile}")
        print(f"  Target TILE (game): {target_tile}")
        print(f"  Player center pixels: ({player_center_x:.1f}, {player_center_y:.1f})")
        print(f"  Target step pixels: ({target_pixel_x:.1f}, {target_pixel_y:.1f})")
        print(f"  Start grid pos: {start_pos}")
        print(f"  Target grid pos: {target_pos}")
        print(f"  Grid walkable at start: {grid[start_pos[0]][start_pos[1]]}")
        print(f"  Grid walkable at target: {grid[target_pos[0]][target_pos[1]]}")
        
        return (grid, start_pos, target_pos, expanded_rect.left, expanded_rect.top, CELL_SIZE, target_tile)

    def find_pixel_path(self, grid, start_pos, target_pos):
        """Run A* on a boolean grid to find path from start to target.
        
        Args:
            grid: 2D list where True = walkable, False = blocked
            start_pos: (row, col) tuple for start position
            target_pos: (row, col) tuple for target position
        
        Returns:
            List of (row, col) tuples representing the path, or [] if no path exists
        """
        rows = len(grid)
        cols = len(grid[0]) if rows > 0 else 0
        
        def is_walkable(row, col):
            if row < 0 or col < 0 or row >= rows or col >= cols:
                return False
            return grid[row][col]
        
        def manhattan(a, b):
            return abs(a[0] - b[0]) + abs(a[1] - b[1])
        
        def neighbors(pos):
            row, col = pos
            for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
                new_row, new_col = row + dr, col + dc
                if is_walkable(new_row, new_col):
                    yield (new_row, new_col)
        
        # Check if start and target are walkable
        if not is_walkable(start_pos[0], start_pos[1]):
            print(f"Pixel A*: Start position {start_pos} is blocked!")
            return []
        if not is_walkable(target_pos[0], target_pos[1]):
            print(f"Pixel A*: Target position {target_pos} is blocked!")
            return []
        
        # A* algorithm
        open_set = []
        heapq.heappush(open_set, (0, start_pos))
        came_from = {}
        g_score = {start_pos: 0}
        f_score = {start_pos: manhattan(start_pos, target_pos)}
        
        while open_set:
            _, current = heapq.heappop(open_set)
            
            if current == target_pos:
                # Reconstruct path
                path = [current]
                while current in came_from:
                    current = came_from[current]
                    path.append(current)
                path.reverse()
                return path
            
            for neighbor in neighbors(current):
                tentative_g = g_score[current] + 1
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + manhattan(neighbor, target_pos)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))
        
        print(f"Pixel A*: No path found from {start_pos} to {target_pos}")
        return []  # No path found

    def pixel_path_to_instructions(self, pixel_path):
        """Convert entire pixel path to list of angle instructions.
        
        Args:
            pixel_path: List of (row, col) tuples from find_pixel_path
                       Path includes start position as first element
        
        Returns:
            List of angles (0=RIGHT, 90=UP, 180=LEFT, 270=DOWN) for each step
        """
        if not pixel_path or len(pixel_path) < 2:
            return []
        
        instructions = []
        
        # Iterate through consecutive pairs in the path
        for i in range(len(pixel_path) - 1):
            current_pos = pixel_path[i]
            next_pos = pixel_path[i + 1]
            
            row_diff = next_pos[0] - current_pos[0]
            col_diff = next_pos[1] - current_pos[1]
            
            # Determine direction based on grid movement
            if row_diff < 0:  # row decreased = move up
                instructions.append(90)
            elif row_diff > 0:  # row increased = move down
                instructions.append(270)
            elif col_diff < 0:  # col decreased = move left
                instructions.append(180)
            elif col_diff > 0:  # col increased = move right
                instructions.append(0)
            # If no movement (shouldn't happen in A* path), skip
        
        return instructions

    def compute_pixel_path(self, path, path_index):
        """Build perception grid, find pixel path, convert to angle instructions.
        
        Args:
            path: Current A* tile path
            path_index: Current index in the path
        
        Returns:
            tuple: (instructions, target_tile) where instructions is list of angles,
                   or (None, target_tile) if path not found
        """
        result = self.make_perception_grid(path, path_index)
        if result is None:
            return None, None
            
        grid, start_pos, target_pos, perception_left, perception_top, CELL_SIZE, target_tile = result
        pixel_path = self.find_pixel_path(grid, start_pos, target_pos)
        
        if pixel_path:
            instructions = self.pixel_path_to_instructions(pixel_path)
            return instructions, target_tile
        else:
            return None, target_tile
