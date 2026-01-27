import pygame
from A_star import A_star
from FuzzyObstacleController import LogiqueFlou
from Constants import PERCEPTION_RADIUS


# Test-only option: when enabled, the AI will chain through
# all reachable targets one by one. Set to False or comment out
# the related code when you no longer need this behaviour.
ENABLE_CHAIN_TARGETS = True


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
        
        # Pixel A* avoidance mode
        self.pixel_astar_mode = False  # Whether we're in pixel A* mode
        self.pixel_instructions = []  # List of angles to execute
        self.pixel_instruction_index = 0  # Current instruction index

    # ---------------- Pixel <-> tile conversions ----------------

    def player_tile(self):
        """Return the tile indices (row, col) where the player currently is."""
        row = int(self.player.y / self.maze.tile_size_y)
        col = int(self.player.x / self.maze.tile_size_x)
        return row, col

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
        astar = A_star(self.maze.maze)

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
            tuple: (grid, start_pos, target_pos, perception_left, perception_top, CELL_SIZE)
        """
        import math
        CELL_SIZE = 1  # 1 pixel per grid cell

        # IMPORTANT: match Maze.make_perception_list exactly.
        # pygame.Rect truncates its args to ints, so we build the same rect here
        # and derive grid origin/size from it.
        perception_distance = PERCEPTION_RADIUS * max(self.maze.tile_size_x, self.maze.tile_size_y)
        perception_left = self.player.x + 0.5 * (self.player.size_x - perception_distance)
        perception_top = self.player.y + 0.5 * (self.player.size_y - perception_distance)
        perception_rect = pygame.Rect(perception_left, perception_top, perception_distance, perception_distance)

        grid_width = max(1, int(perception_rect.width // CELL_SIZE))
        grid_height = max(1, int(perception_rect.height // CELL_SIZE))

        # Create empty grid (all walkable initially)
        grid = [[True for _ in range(grid_width)] for _ in range(grid_height)]

        player_rect = self.player.get_rect()

        def pix_to_col(pixel_x):
            return int(math.floor((pixel_x - perception_rect.left) / CELL_SIZE))

        def pix_to_row(pixel_y):
            return int(math.floor((pixel_y - perception_rect.top) / CELL_SIZE))

        def clamp_pixel_into_perception(pixel_x, pixel_y):
            clamped_x = max(perception_rect.left, min(perception_rect.right - 1, int(pixel_x)))
            clamped_y = max(perception_rect.top, min(perception_rect.bottom - 1, int(pixel_y)))
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

        perception = self.maze.make_perception_list(self.player, None)
        # Combine walls and obstacles
        all_blockers = perception[0] + perception[1]  # walls + obstacles

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

        # Start from the far edge in the intended direction and scan inward until we find a
        # walkable cell on the same "ray" (same row/col) as the player.
        start_r, start_c = start_pos
        if step_row == 1:  # down
            edge_candidate = (grid_height - 1, start_c)
            scan_positions = ((r, start_c) for r in range(grid_height - 1, -1, -1))
        elif step_row == -1:  # up
            edge_candidate = (0, start_c)
            scan_positions = ((r, start_c) for r in range(0, grid_height))
        elif step_col == 1:  # right
            edge_candidate = (start_r, grid_width - 1)
            scan_positions = ((start_r, c) for c in range(grid_width - 1, -1, -1))
        else:  # left
            edge_candidate = (start_r, 0)
            scan_positions = ((start_r, c) for c in range(0, grid_width))

        target_pos = None
        for rr, cc in scan_positions:
            if grid[rr][cc]:
                target_pos = (rr, cc)
                break

        # If the entire ray is blocked, fall back to the closest walkable around the edge.
        if target_pos is None:
            target_pos = nearest_walkable(edge_candidate, max_radius=64)
            if target_pos is None:
                print(f"Pixel A*: No walkable target cell near forward edge {edge_candidate} in local perception grid")
                return None

        # Convert the chosen grid target back to a representative pixel (for debug only).
        target_pixel_x = perception_rect.left + target_pos[1] * CELL_SIZE
        target_pixel_y = perception_rect.top + target_pos[0] * CELL_SIZE
        
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
        
        return (grid, start_pos, target_pos, perception_rect.left, perception_rect.top, CELL_SIZE)

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
            grid, start_pos, target_pos, perception_left, perception_top, CELL_SIZE = result
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
                print("Pixel A*: No path found, falling back to fuzzy logic")
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
        import numpy as np
        perception = self.maze.make_perception_list(self.player, None)
        obstacle_list = perception[1]  # obstacles
        wall_list = perception[0]  # walls
        
        final_direction_angle = self.direction_a_star

        # Apply fuzzy logic if there are OBSTACLES or nearby WALLS
        if len(obstacle_list) > 0 or len(wall_list) > 0:
            # Run fuzzy logic to get obstacle avoidance direction
            logique_direction, has_obstacle = self.run_logique_floue(perception)
            
            # Check if fuzzy is turning into a blocked side (when obstacles present)
            if has_obstacle and len(obstacle_list) > 0:
                # Calculate clearance on both sides
                player_pos = self.player.get_rect().center
                player_width = self.player.size_x
                player_height = self.player.size_y
                
                # Determine min clearance needed
                if 45 <= self.last_direction < 135 or 225 <= self.last_direction < 315:
                    min_clearance = player_width * 1.5
                else:
                    min_clearance = player_height * 1.5
                
                left_clearance = float('inf')
                right_clearance = float('inf')
                
                # Check all obstacles and walls
                all_blockers = obstacle_list + wall_list
                for blocker in all_blockers:
                    dx = blocker.center[0] - player_pos[0]
                    dy = -(blocker.center[1] - player_pos[1])
                    
                    angle_to_blocker = np.degrees(np.arctan2(dy, dx))
                    if angle_to_blocker < 0:
                        angle_to_blocker += 360
                    
                    rel_angle = self.last_direction - angle_to_blocker
                    if rel_angle < -180:
                        rel_angle += 360
                    if rel_angle > 180:
                        rel_angle -= 360
                    
                    distance = np.sqrt(dx**2 + dy**2)
                    
                    if 0 < rel_angle < 90:
                        right_clearance = min(right_clearance, distance)
                    elif -90 < rel_angle < 0:
                        left_clearance = min(left_clearance, distance)
                
                # Determine which direction fuzzy is turning
                fuzzy_turn = logique_direction - self.last_direction
                if fuzzy_turn > 180:
                    fuzzy_turn -= 360
                if fuzzy_turn < -180:
                    fuzzy_turn += 360
                
                # If fuzzy turning left but left is blocked and right is clear, override
                if fuzzy_turn > 10 and left_clearance < min_clearance and right_clearance >= min_clearance:
                    print(f"Override: Fuzzy wants LEFT but blocked (L:{left_clearance:.1f} R:{right_clearance:.1f}), forcing RIGHT")
                    print("Making it go right then")
                    logique_direction = self.direction_a_star - 30
                    if logique_direction < 0:
                        logique_direction += 360
                # If fuzzy turning right but right is blocked and left is clear, override
                elif fuzzy_turn < -10 and right_clearance < min_clearance and left_clearance >= min_clearance:
                   
                    print(f"Override: Fuzzy wants RIGHT but blocked (L:{left_clearance:.1f} R:{right_clearance:.1f}), forcing LEFT")
                    print("Making it go left then")
                    logique_direction = self.direction_a_star + 30
                    if logique_direction >= 360:
                        logique_direction -= 360
            
            final_direction_angle = logique_direction
            self.fuzzy_use_counter += 1
            
            # Safety: if fuzzy takes too long, force A* to push through
            if self.fuzzy_use_counter >= self.max_fuzzy_frames:
                print(f"Fuzzy taking too long, forcing A* push")
                final_direction_angle = self.direction_a_star
                self.fuzzy_use_counter = 0
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
