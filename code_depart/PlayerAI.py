from A_star import A_star


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

    def get_next_instruction(self):
        """Return the next direction for the player, or None if nothing to do."""

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
        
        print(f"instr:", instr)
        return instr
