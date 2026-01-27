import pygame
from A_star import A_star
from FuzzyObstacleController import LogiqueFlou
from PixelAstar import PixelAstar
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
        
        # Pixel A* controller
        self.pixel_astar = PixelAstar(maze, player)
        
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

    def activate_pixel_astar_mode(self):
        """Activate pixel A* avoidance mode when obstacle collision detected.
        
        Builds perception grid, finds pixel path, converts to instructions,
        and switches to pixel A* mode for execution.
        """
        try:
            # Use PixelAstar class to compute path
            instructions, target_tile = self.pixel_astar.compute_pixel_path(self.path, self.path_index)
            
            if instructions:
                self.pixel_instructions = instructions
                self.pixel_instruction_index = 0
                self.pixel_astar_mode = True
                print(f"Pixel A* activated: {len(self.pixel_instructions)} instructions to execute")
                return True
            else:
                print("Pixel A*: No path found, marking tile blocked and recomputing global A*")
                if target_tile:
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
