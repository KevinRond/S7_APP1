import pygame
from A_star import A_star
from FuzzyObstacleController import LogiqueFlou
from Constants import PERCEPTION_RADIUS
import numpy as np



# Test-only option: when enabled, the AI will chain through
# all reachable targets one by one. Set to False or comment out
# the related code when you no longer need this behaviour.
ENABLE_CHAIN_TARGETS = True

# HOMING mode: fine-tuned navigation to collect items (ported from PlayerAI_OLD)
HOMING_ACTIVATION_DISTANCE = 60  # pixels (kept for parity; perception-gated in this AI)
PRECISION_THRESHOLD = 8  # pixels - precision to consider we're "arrived" to the item

# Pixel A* grid expansion: when an obstacle/wall is perceived, we can safely infer
# the *rest of that tile* (outside the obstacle rect) is empty (since there is at most
# one obstacle per tile). So we expand the grid to include whole tiles for perceived
# blockers, even if parts of those tiles lie outside the perception square.
PIXEL_ASTAR_INCLUDE_FULL_BLOCKER_TILES = True


class PlayerAI:
    """Simple AI controller that uses A* to move the player automatically.

    Usage idea (in App):
      ai = PlayerAI(self.maze, self.player)
      ai.recompute_path()  # from current player position
      instr = ai.get_next_instruction()  # 'UP', 'DOWN', 'LEFT', 'RIGHT'
      if instr:
          self.on_AI_input(instr)
    """

    def __init__(self, maze, player):
        self.maze = maze          # Maze instance
        self.player = player      # Player instance
        self.path = []            # list of (row, col)
        self.path_index = 0       # current waypoint index in path
        # Positions (row, col) of targets already visited when chaining
        # multiple goals (used only if ENABLE_CHAIN_TARGETS is True).
        self.completed_targets = set()
        
        # Fuzzy logic controller
        self.logique_floue = LogiqueFlou()
        self.last_direction = 270  # Initial direction (DOWN in degrees)
        self.direction_a_star = 0  # A* target direction in degrees
        
        # Oscillation detection
        self.fuzzy_use_counter = 0  # Count frames using fuzzy direction
        self.max_fuzzy_frames = 20  # Max frames before forcing A* to push through
        
        # Stuck detection (position-based)
        self.position_history = []  # Recent positions (pixel coords)
        self.stuck_check_interval = 10  # Check every N frames
        self.stuck_distance_threshold = 5.0  # pixels - if moved less than this, considered stuck
        
        # Pixel A* avoidance mode
        self.pixel_astar_mode = False  # Whether we're in pixel A* mode
        self.pixel_instructions = []  # List of angles to execute
        self.pixel_instruction_index = 0  # Current instruction index

        # HOMING mode (ported from PlayerAI_OLD)
        self.mode = 'PATH'  # 'PATH' or 'HOMING'
        self.current_target = None  # pygame.Rect of the targeted item

        # Dynamic tile blocking: if pixel A* cannot find a local path toward the next
        # tile waypoint, temporarily mark that tile as blocked and recompute tile A*.
        self.temp_blocked_tiles = {}  # (row, col) -> expires_at_ms

    def _active_blocked_tiles(self):
        """Return the set of currently-blocked tiles (prunes expired entries)."""
        now = pygame.time.get_ticks()
        expired = [tile for tile, expires_at in self.temp_blocked_tiles.items() if expires_at <= now]
        for tile in expired:
            self.temp_blocked_tiles.pop(tile, None)
        return set(self.temp_blocked_tiles.keys())

    def temporarily_block_tile(self, tile, duration_ms=2000, reason=None):
        """Temporarily block a tile for the global tile A* planner."""
        if tile is None:
            return
        if tile == self.player_tile():
            return
        now = pygame.time.get_ticks()
        expires_at = now + int(duration_ms)
        current = self.temp_blocked_tiles.get(tile)
        if current is None or expires_at > current:
            self.temp_blocked_tiles[tile] = expires_at
        if reason:
            print(f"Tile temporarily blocked {tile} ({reason})")
        else:
            print(f"Tile temporarily blocked {tile}")

    # ---------------- Pixel <-> tile conversions ----------------

    def player_tile(self):
        """Return the tile indices (row, col) where the player currently is."""
        row = int(self.player.y / self.maze.tile_size_y)
        col = int(self.player.x / self.maze.tile_size_x)
        return row, col
    
    def is_player_stuck(self):
        """Check if player is stationary (not making progress).
        
        Returns:
            True if player hasn't moved significantly in recent frames, False otherwise.
        """
        # Get current position
        current_pos = (self.player.x, self.player.y)
        
        # Add to history
        self.position_history.append(current_pos)
        
        # Keep only recent positions (last stuck_check_interval frames)
        if len(self.position_history) > self.stuck_check_interval:
            self.position_history.pop(0)
        
        # Need enough history to check
        if len(self.position_history) < self.stuck_check_interval:
            return False
        
        # Calculate distance moved from oldest to newest position
        old_pos = self.position_history[0]
        dx = current_pos[0] - old_pos[0]
        dy = current_pos[1] - old_pos[1]
        distance_moved = (dx * dx + dy * dy) ** 0.5
        
        # If moved less than threshold, consider stuck
        return distance_moved < self.stuck_distance_threshold

    # ---------------- Item detection (HOMING) ----------------

    def get_nearest_item_in_perception(self):
        """Find nearest item (coin/treasure) inside the perception list.

        Returns the pygame.Rect of the nearest item, or None.
        """
        perception = self.maze.make_perception_list(self.player, None)
        item_list = perception[2]  # coins + treasures

        if not item_list:
            return None

        player_cx, player_cy = self.player.get_rect().center

        def distance_sq_to_item(item):
            dx = item.centerx - player_cx
            dy = item.centery - player_cy
            return dx * dx + dy * dy

        return min(item_list, key=distance_sq_to_item)

    def compute_direction_to_target(self, target_rect):
        """Compute a simple cardinal direction toward an item.

        Returns:
            'UP'|'DOWN'|'LEFT'|'RIGHT' or None if within PRECISION_THRESHOLD.
        """
        player_cx, player_cy = self.player.get_rect().center

        dx = target_rect.centerx - player_cx
        dy = target_rect.centery - player_cy

        # Prioritize the dominant axis (Manhattan-ish) like PlayerAI_OLD.
        if abs(dx) > PRECISION_THRESHOLD and abs(dx) >= abs(dy):
            return 'RIGHT' if dx > 0 else 'LEFT'
        elif abs(dy) > PRECISION_THRESHOLD:
            return 'DOWN' if dy > 0 else 'UP'
        else:
            return None

    def mark_tile_current_tile_completed(self):
        """Mark the player's current tile as completed (HOMING parity helper)."""
        row, col = self.player_tile()
        print("current_tile marked as completed:", (row, col))
        self.completed_targets.add((row, col))

    # ---------------- Path and instructions ----------------

    def get_direction_to_tile(self, target_row, target_col):
        """Calculate direction from current player position to target tile.
        
        Returns:
            Direction string ('UP', 'DOWN', 'LEFT', 'RIGHT') or None if at target.
        """
        current_row, current_col = self.player_tile()
        
        dr = target_row - current_row
        dc = target_col - current_col
        
        # Prioritize vertical movement if both needed (Manhattan style)
        if dr < 0:
            return 'UP'
        elif dr > 0:
            return 'DOWN'
        elif dc < 0:
            return 'LEFT'
        elif dc > 0:
            return 'RIGHT'
        else:
            return None  # Already at target tile

    def recompute_path(self):
        """Recompute a normal A* path from the player's current tile.

        If ENABLE_CHAIN_TARGETS is True, already-completed targets are
        ignored so the player will move on to the next closest one.
        """
        print("Recompute was called")
        start_tile = self.player_tile()
        astar = A_star(self.maze.maze, blocked_tiles=self._active_blocked_tiles())

        if ENABLE_CHAIN_TARGETS and self.completed_targets:
            self.path = astar.find_path_from(start_tile, excluded_targets=self.completed_targets)
            print(f"self.path: {self.path}")
        else:
            self.path = astar.find_path_from(start_tile)
            print(f"self.path: {self.path}")
        
        # Position-based tracking: start at first waypoint
        # Path index 0 is current position, so we target index 1 first
        self.path_index = 1 if len(self.path) > 1 else 0

    def mark_tile_completed(self, row, col):
        """Marque une tuile comme cible complétée."""
        print("tile marked as completed:", (row, col))
        self.completed_targets.add((row, col))

    def direction_to_angle(self, direction):
        """Convert cardinal direction to angle in degrees."""
        direction_angles = {
            'UP': 90,
            'DOWN': 270,
            'LEFT': 180,
            'RIGHT': 0
        }
        return direction_angles.get(direction, 0)

    def angle_to_direction(self, angle):
        """Convert angle in degrees to closest cardinal direction."""
        # Normalize angle to 0-360
        angle = angle % 360
        if angle < 0:
            angle += 360
        
        # Find closest cardinal direction
        if 315 <= angle or angle < 45:
            return 'RIGHT'
        elif 45 <= angle < 135:
            return 'UP'
        elif 135 <= angle < 225:
            return 'LEFT'
        else:  # 225 <= angle < 315
            return 'DOWN'
        
# ---------------- A star obstacle avoidance ----------------

    def make_perception_grid(self):
        """Build perception grid and find pixel-level path.
        
        Returns:
            tuple: (grid, start_pos, target_pos, perception_left, perception_top, CELL_SIZE, target_tile)
        """
        import math
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

        # Optionally expand the pixel grid to include full tiles that contain perceived blockers.
        # This adds extra (known-empty) space in those tiles beyond the perception square.
        expanded_rect = perception_rect
        if PIXEL_ASTAR_INCLUDE_FULL_BLOCKER_TILES and all_blockers:
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

        def clamp_pixel_into_perception(pixel_x, pixel_y):
            # Kept name for historical reasons; clamp into the expanded grid rect.
            clamped_x = max(expanded_rect.left, min(expanded_rect.right - 1, int(pixel_x)))
            clamped_y = max(expanded_rect.top, min(expanded_rect.bottom - 1, int(pixel_y)))
            return clamped_x, clamped_y

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
            # IMPORTANT: pygame.Rect.right / .bottom are EXCLUSIVE bounds.
            # So we use (right - 1) and (bottom - 1) when mapping to cell indices,
            # otherwise we block one extra row/col.
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

        # If we're touching an inflated blocker, the exact center cell can be blocked.
        # Don't blindly force it open; instead, snap to the nearest walkable cell.
        snapped_start = nearest_walkable(start_pos)
        if snapped_start is None:
            print("Pixel A*: No walkable start cell in local perception grid")
            return None
        start_pos = snapped_start

        # Find the next tile to target - should be adjacent to current tile
        # Use center-based tile for consistency with pixel grid.
        current_tile = (int(player_center_y / self.maze.tile_size_y), int(player_center_x / self.maze.tile_size_x))
        
        # Look through path from current index to find the next adjacent tile
        target_tile = None
        for i in range(self.path_index, len(self.path)):
            candidate = self.path[i]
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
            if self.path_index < len(self.path):
                target_tile = self.path[self.path_index]
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
        current_tile = self.player_tile()
        print(f"DEBUG Grid Build:")
        print(f"  Current player TILE (game): {current_tile}")
        print(f"  Target TILE (game): {target_tile}")
        print(f"  Player center pixels: ({player_center_x:.1f}, {player_center_y:.1f})")
        print(f"  Target step pixels: ({target_pixel_x:.1f}, {target_pixel_y:.1f})")
        print(f"  Start grid pos: {start_pos}")
        print(f"  Target grid pos: {target_pos}")
        print(f"  Grid walkable at start: {grid[start_pos[0]][start_pos[1]]}")
        print(f"  Grid walkable at target: {grid[target_pos[0]][target_pos[1]]}")
        
        return (grid, start_pos, target_pos, expanded_rect.left, expanded_rect.top, CELL_SIZE, target_tile)

    def clamp_to_grid_edge(self, target_row, target_col, grid_height, grid_width):
        """Clamp target position to nearest grid edge if outside bounds.
        
        Args:
            target_row: Target row (may be outside grid)
            target_col: Target column (may be outside grid)
            grid_height: Grid height
            grid_width: Grid width
        
        Returns:
            (clamped_row, clamped_col) tuple within grid bounds
        """
        clamped_row = max(0, min(grid_height - 1, target_row))
        clamped_col = max(0, min(grid_width - 1, target_col))
        
        return (clamped_row, clamped_col)

    def find_pixel_path(self, grid, start_pos, target_pos):
        """Run A* on a boolean grid to find path from start to target.
        
        Args:
            grid: 2D list where True = walkable, False = blocked
            start_pos: (row, col) tuple for start position
            target_pos: (row, col) tuple for target position
        
        Returns:
            List of (row, col) tuples representing the path, or [] if no path exists
        """
        import heapq
        
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

    def activate_pixel_astar_mode(self):
        """Activate pixel A* avoidance mode when obstacle collision detected.
        
        Builds perception grid, finds pixel path, converts to instructions,
        and switches to pixel A* mode for execution.
        """
        try:
            # Build grid and find path
            result = self.make_perception_grid()
            if result is None:
                return False
            grid, start_pos, target_pos, perception_left, perception_top, CELL_SIZE, target_tile = result
            pixel_path = self.find_pixel_path(grid, start_pos, target_pos)
            
            if pixel_path:
                # Convert path to instructions
                self.pixel_instructions = self.pixel_path_to_instructions(pixel_path)
                
                if self.pixel_instructions:
                    self.pixel_instruction_index = 0
                    self.pixel_astar_mode = True
                    print(f"Pixel A* activated: {len(self.pixel_instructions)} instructions to execute")
                    return True
                else:
                    print("Pixel A*: No instructions generated from path")
            else:
                print("Pixel A*: No path found, marking tile blocked and recomputing global A*")
                self.temporarily_block_tile(target_tile, duration_ms=2000, reason="pixel A* no-path")
                self.recompute_path()
        except Exception as e:
            print(f"Pixel A* activation failed: {e}")
        
        return False

    def run_logique_floue(self, perception):
        """Run fuzzy logic controller to get obstacle avoidance direction.
        
        Returns:
            (next_direction_angle, has_obstacle) tuple
        """
        instruction, has_obstacle = self.logique_floue.run(
            self.last_direction, 
            self.direction_a_star, 
            self.player, 
            perception
        )
        
        next_direction = self.last_direction
        
        # Subtract the fuzzy adjustment from current direction
        next_direction -= instruction
        
        # Normalize angle to 0-360
        if next_direction >= 360:
            next_direction -= 360
        if next_direction < 0:
            next_direction += 360
        
        return next_direction, has_obstacle

    def get_next_instruction(self):
        """Return the next direction for the player as an angle, or None if nothing to do."""

        # Check if in pixel A* mode first
        if self.pixel_astar_mode:
            if self.pixel_instruction_index < len(self.pixel_instructions):
                # Get next pixel instruction
                instruction = self.pixel_instructions[self.pixel_instruction_index]
                self.pixel_instruction_index += 1
                print(f"Pixel A* [{self.pixel_instruction_index}/{len(self.pixel_instructions)}]: {instruction}°")
                return instruction
            else:
                # Finished all pixel instructions, exit mode
                print("Pixel A* complete, returning to normal navigation")
                self.pixel_astar_mode = False
                self.pixel_instructions = []
                self.pixel_instruction_index = 0
                # Fall through to normal navigation

        # HOMING mode: fine navigation to collect nearby items
        if self.mode == 'HOMING':
            target = self.get_nearest_item_in_perception()

            if target is None:
                # No item visible anymore, return to PATH
                print("No target found, switching to PATH")
                self.mode = 'PATH'
                self.current_target = None
                self.mark_tile_current_tile_completed()
                self.recompute_path()
                return None

            intended_direction = self.compute_direction_to_target(target)

            if intended_direction is None:
                # Close enough to the item, return to PATH
                print("Arrived at target, switching to PATH")
                self.mode = 'PATH'
                self.current_target = None
                self.mark_tile_current_tile_completed()
                self.recompute_path()
                return None

            # In this AI, everything downstream expects angles.
            self.direction_a_star = self.direction_to_angle(intended_direction)

            # Reuse the same obstacle handling as PATH mode.
            perception = self.maze.make_perception_list(self.player, None)
            obstacle_list = perception[1]
            wall_list = perception[0]

            final_direction_angle = self.direction_a_star

            # Match PATH-mode obstacle handling: trust fuzzy output when something is in perception.
            if len(obstacle_list) > 0 or len(wall_list) > 0:
                logique_direction, has_obstacle = self.run_logique_floue(perception)
                final_direction_angle = logique_direction
                self.fuzzy_use_counter += 1

                # Safety: if fuzzy takes too long, force A* to push through
                if self.fuzzy_use_counter >= self.max_fuzzy_frames:
                    print("Fuzzy taking too long (HOMING), forcing A* push")
                    final_direction_angle = self.direction_a_star
                    self.fuzzy_use_counter = 0
            else:
                final_direction_angle = self.direction_a_star
                self.fuzzy_use_counter = 0

            self.last_direction = final_direction_angle
            return final_direction_angle

        # PATH mode: optionally switch to HOMING when near the end of the path
        if self.path and len(self.path) > 0:
            current_tile = self.player_tile()
            if current_tile == self.path[-1] or (len(self.path) >= 2 and current_tile == self.path[-2]):
                nearest_item = self.get_nearest_item_in_perception()
                if nearest_item:
                    self.mode = 'HOMING'
                    self.current_target = nearest_item
                    return self.get_next_instruction()

        # Position-based path following
        if not self.path or self.path_index >= len(self.path):
            # No path or reached end of path
            self.recompute_path()
            print("I got nothin")
            return None
        
        # Get current target tile from path
        target_row, target_col = self.path[self.path_index]
        current_row, current_col = self.player_tile()
        
        # Check if we've reached the current waypoint
        if (current_row, current_col) == (target_row, target_col):
            # Reached waypoint, advance to next one
            self.path_index += 1
            
            # Check if we've completed the entire path
            if self.path_index >= len(self.path):
                # Mark target as completed if chaining is enabled
                if ENABLE_CHAIN_TARGETS and self.path:
                    self.completed_targets.add(self.path[-1])
                self.recompute_path()
                return None
            
            # Get next target
            target_row, target_col = self.path[self.path_index]
        
        # Calculate direction to current target tile
        instr = self.get_direction_to_tile(target_row, target_col)
        
        if instr is None:
            # Shouldn't happen, but advance just in case
            self.path_index += 1
            return self.get_next_instruction()
        
        # Convert A* direction to angle
        self.direction_a_star = self.direction_to_angle(instr)
        
        # Check for obstacles and run fuzzy logic if needed
        perception = self.maze.make_perception_list(self.player, None)
        obstacle_list = perception[1]  # obstacles
        wall_list = perception[0]  # walls
        
        final_direction_angle = self.direction_a_star

        # Apply fuzzy logic if there are OBSTACLES or nearby WALLS
        if len(obstacle_list) > 0 or len(wall_list) > 0:
            # Run fuzzy logic to get obstacle avoidance direction
            logique_direction, has_obstacle = self.run_logique_floue(perception)
            
            # Trust fuzzy logic completely - no overrides
            final_direction_angle = logique_direction

            self.fuzzy_use_counter += 1
            
            # Safety: if fuzzy takes too long, check if we're actually stuck
            if self.fuzzy_use_counter >= self.max_fuzzy_frames:
                final_direction_angle = self.direction_a_star
                self.fuzzy_use_counter = 0

                if self.is_player_stuck():
                    # Truly stuck (not moving) - recompute path to find alternate route
                    print(f"Player stuck (stationary for {self.stuck_check_interval} frames), recomputing path")
                    self.recompute_path()
                
                else:
                    # Still moving - just force A* direction without recomputing
                    # This prevents path oscillation while maintaining progress
                    print(f"Fuzzy timeout but still moving, forcing A* push (no recompute)")
                    final_direction_angle = self.direction_a_star
                    self.fuzzy_use_counter = 0  # Reset counter to allow fuzzy to try again
        else:
            # No obstacles, just follow A* path (collision system handles walls)
            final_direction_angle = self.direction_a_star
            self.fuzzy_use_counter = 0
        
        # Update last direction
        self.last_direction = final_direction_angle
        
        if len(obstacle_list) > 0 or len(wall_list) > 0:
            print(f"A* dir: {instr}, Fuzzy: {logique_direction:.1f}°, Last: {self.last_direction:.1f}°, Obstacles: {len(obstacle_list)}, Walls: {len(wall_list)}, Final: {final_direction_angle:.1f}°")
            #print(f"A* dir: {instr}, Fuzzy: {logique_direction:.1f}°, Last: {self.last_direction:.1f}°, Obstacles: {len(obstacle_list)}, Walls: {len(wall_list)}{side_info}, Final: {final_direction_angle:.1f}°")
        
        # Return the angle directly (not converted to cardinal direction)
        return final_direction_angle
