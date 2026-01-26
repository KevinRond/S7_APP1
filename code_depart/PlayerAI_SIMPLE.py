from A_star import A_star
from FuzzyObstacleController import LogiqueFlou


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
        
        # Stuck detection
        self.last_position = None
        self.stuck_counter = 0
        self.stuck_threshold = 5  # Number of frames before considering stuck
        self.try_opposite_side = False  # Flag to try opposite direction when stuck

    # ---------------- Pixel <-> tile conversions ----------------

    def player_tile(self):
        """Return the tile indices (row, col) where the player currently is."""
        row = int(self.player.y / self.maze.tile_size_y)
        col = int(self.player.x / self.maze.tile_size_x)
        return row, col
    
    def pixel_to_tile_center_pos(self, pixel_pos):
        """Convert pixel position to sub-tile center position (3x3 grid within each tile)."""
        return int(pixel_pos[0] // (self.maze.tile_size_x / 3)), int(pixel_pos[1] // (self.maze.tile_size_y / 3))
    
    def tile_pos_to_center_pos(self, tile_pos):
        """Convert tile position (row, col) to center position in 3x3 sub-grid.
        Returns (center_col, center_row) in sub-tile coordinates."""
        row, col = tile_pos
        return ((col * 3) + 1), ((row * 3) + 1)  # Note: returns (x, y) format

    # ---------------- Path and instructions ----------------

    def get_direction_to_center(self, target_row, target_col):
        """Calculate direction from current player CENTER position to target tile CENTER.
        
        Uses 3x3 sub-grid for precise center-based navigation.
        Returns:
            Direction string ('UP', 'DOWN', 'LEFT', 'RIGHT') or None if at target center.
        """
        # Get player's current center position in sub-tile coordinates
        player_center = self.player.get_rect().center
        curr_center_x, curr_center_y = self.pixel_to_tile_center_pos(player_center)
        
        # Get target tile's center position in sub-tile coordinates
        target_center_x, target_center_y = self.tile_pos_to_center_pos((target_row, target_col))
        
        # Calculate direction to center
        dx = target_center_x - curr_center_x
        dy = target_center_y - curr_center_y
        
        # Prioritize vertical movement if both needed (Manhattan style)
        if dy < 0:
            return 'UP'
        elif dy > 0:
            return 'DOWN'
        elif dx < 0:
            return 'LEFT'
        elif dx > 0:
            return 'RIGHT'
        else:
            return None  # Already at target center

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

        # Check if player is stuck (same position for multiple frames)
        current_pos = (self.player.x, self.player.y)
        if self.last_position == current_pos:
            self.stuck_counter += 1
            if self.stuck_counter >= self.stuck_threshold:
                # Player is stuck, try opposite direction next time
                if not self.try_opposite_side:
                    self.try_opposite_side = True
                    print(f"STUCK DETECTED! Trying opposite side...")
        else:
            # Player moved, reset stuck detection
            if self.stuck_counter > 0:
                print(f"Unstuck! Resetting...")
            self.stuck_counter = 0
            self.try_opposite_side = False
        
        self.last_position = current_pos

        # Position-based path following
        if not self.path or self.path_index >= len(self.path):
            # No path or reached end of path
            self.recompute_path()
            print("I got nothin")
            return None
        
        # Get current target tile from path
        target_row, target_col = self.path[self.path_index]
        
        # Check if we've reached the CENTER of the current waypoint (using 3x3 sub-grid)
        player_center = self.player.get_rect().center
        curr_center_pos = self.pixel_to_tile_center_pos(player_center)
        target_center_pos = self.tile_pos_to_center_pos((target_row, target_col))
        
        if curr_center_pos == target_center_pos:
            # Reached waypoint center, advance to next one
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
        
        # Calculate direction to current target tile CENTER
        instr = self.get_direction_to_center(target_row, target_col)
        
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
        
        # Only apply fuzzy logic if there are OBSTACLES (not just walls)
        if len(obstacle_list) > 0:
            # Run fuzzy logic to get obstacle avoidance direction
            logique_direction, has_obstacle = self.run_logique_floue(perception)
            
            if has_obstacle:
                # Check if fuzzy direction has stabilized
                difference = abs(logique_direction - self.last_direction)
                
                # Normalize difference to 0-180 range
                if difference > 180:
                    difference = 360 - difference
                
                # If fuzzy direction is stable (converged), use A* to make progress
                # Otherwise use fuzzy to dodge the obstacle
                if difference < 0.01:
                    final_direction_angle = self.direction_a_star
                    self.fuzzy_use_counter = 0  # Reset counter when converged
                else:
                    # Fuzzy is still adjusting
                    self.fuzzy_use_counter += 1
                    
                    # If fuzzy has been active too long without progress, force A* to push through
                    if self.fuzzy_use_counter >= self.max_fuzzy_frames:
                        final_direction_angle = self.direction_a_star
                        self.fuzzy_use_counter = 0  # Reset counter
                        print(f"Fuzzy oscillating too long! Forcing A* direction to push through obstacle.")
                    else:
                        final_direction_angle = logique_direction
            else:
                # No obstacles detected, use A* direction
                final_direction_angle = self.direction_a_star
                self.fuzzy_use_counter = 0
        else:
            # No obstacles, just follow A* path (collision system handles walls)
            final_direction_angle = self.direction_a_star
            self.fuzzy_use_counter = 0
        
        # Update last direction
        self.last_direction = final_direction_angle
        
        if len(obstacle_list) > 0:
            print(f"A* dir: {instr}, Fuzzy: {logique_direction:.1f}°, Last: {self.last_direction:.1f}°, Obstacles: {len(obstacle_list)}, Final angle: {final_direction_angle:.1f}°")
        
        # Return the angle directly (not converted to cardinal direction)
        return final_direction_angle
