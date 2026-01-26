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
        
        # Only apply fuzzy logic if there are obstacles OR walls nearby
        if len(obstacle_list) > 0 or len(wall_list) > 0:
            # Run fuzzy logic to get obstacle avoidance direction
            logique_direction, has_obstacle = self.run_logique_floue(perception)
            
            if has_obstacle:
                # Check if fuzzy direction has stabilized (similar to example code)
                difference = abs(logique_direction - self.last_direction)
                
                # Normalize difference to 0-180 range
                if difference > 180:
                    difference = 360 - difference
                
                # If fuzzy direction is stable (very small change from last direction)
                if difference < 0.1:
                    # Fuzzy has converged, use A* direction to make progress
                    final_direction_angle = self.direction_a_star
                else:
                    # Fuzzy is still adjusting, use its suggestion
                    final_direction_angle = logique_direction
            else:
                # No obstacles detected by fuzzy, use A* direction
                final_direction_angle = self.direction_a_star
        else:
            # No obstacles in perception, use A* direction
            final_direction_angle = self.direction_a_star
        
        # Update last direction
        self.last_direction = final_direction_angle
        
        if len(obstacle_list) > 0 or len(wall_list) > 0:
            print(f"A* dir: {instr}, Fuzzy: {logique_direction:.1f}°, Last: {self.last_direction:.1f}°, Obstacles: {len(obstacle_list)}, Final angle: {final_direction_angle:.1f}°")
        
        # Return the angle directly (not converted to cardinal direction)
        return final_direction_angle
