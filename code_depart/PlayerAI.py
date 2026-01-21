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
        self.instructions = []    # list of 'UP'/'DOWN'/'LEFT'/'RIGHT'
        self.instr_index = 0      # index into path instructions
        self.mode = 'PATH'        # 'PATH' or 'RECENTER'
        self.center_instructions = []  # recenter sequence when in RECENTER mode
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

    def path_to_instructions(self, path):
        """Convert a list of tiles [(r,c), ...] to move instructions.

        Each step between two tiles is expanded into several identical
        instructions so that the player approximately moves one full tile.
        """
        instructions = []
        if not path or len(path) < 2:
            return instructions

        # Approximate number of pixel steps to traverse one tile
        # (use horizontal tile size; vertical is similar in this game).
        steps_per_tile = max(1, int(self.maze.tile_size_x / self.player.speed))

        for (r1, c1), (r2, c2) in zip(path, path[1:]):
            dr, dc = r2 - r1, c2 - c1
            if dr == -1 and dc == 0:
                direction = 'UP'
            elif dr == 1 and dc == 0:
                direction = 'DOWN'
            elif dr == 0 and dc == -1:
                direction = 'LEFT'
            elif dr == 0 and dc == 1:
                direction = 'RIGHT'
            else:
                continue

            for _ in range(steps_per_tile):
                instructions.append(direction)

        return instructions

    def recompute_path(self):
        """Recompute a normal A* path from the player's current tile.

        If ENABLE_CHAIN_TARGETS is True, already-completed targets are
        ignored so the player will move on to the next closest one.
        """
        start_tile = self.player_tile()
        astar = A_star(self.maze.maze)

        if ENABLE_CHAIN_TARGETS and self.completed_targets:
            self.path = astar.find_path_from(start_tile, excluded_targets=self.completed_targets)
        else:
            self.path = astar.find_path_from(start_tile)
        # print("A* start:", start_tile, "path length:", len(self.path))
        self.instructions = self.path_to_instructions(self.path)
        # print("instructions:", self.instructions[:10])  # preview
        self.instr_index = 0
        self.mode = 'PATH'
        self.center_instructions = []

    def start_recenter(self):
        """Enter recenter mode: prepare a small sequence of moves to reach tile center.

        This does not move the player immediately; get_next_instruction will
        emit these moves one per frame until the player is centered, then
        switch back to PATH mode and recompute an A* path.
        """
        if self.mode == 'RECENTER':
            return

        row = int(self.player.y / self.maze.tile_size_y)
        col = int(self.player.x / self.maze.tile_size_x)
        tile_center_x = (col + 0.5) * self.maze.tile_size_x
        tile_center_y = (row + 0.5) * self.maze.tile_size_y

        half_w = self.player.size_x / 2
        half_h = self.player.size_y / 2
        cur_center_x = self.player.x + half_w
        cur_center_y = self.player.y + half_h

        instructions = []

        # Horizontal correction
        dx = tile_center_x - cur_center_x
        if dx > 0:
            steps = int(abs(dx) / self.player.speed)
            for _ in range(steps):
                instructions.append('RIGHT')
        elif dx < 0:
            steps = int(abs(dx) / self.player.speed)
            for _ in range(steps):
                instructions.append('LEFT')

        # Vertical correction
        dy = tile_center_y - cur_center_y
        if dy > 0:
            steps = int(abs(dy) / self.player.speed)
            for _ in range(steps):
                instructions.append('DOWN')
        elif dy < 0:
            steps = int(abs(dy) / self.player.speed)
            for _ in range(steps):
                instructions.append('UP')

        self.center_instructions = instructions
        self.mode = 'RECENTER'

    def get_next_instruction(self):
        """Return the next direction for the player, or None if nothing to do."""

        # If we are recentering, consume recenter instructions first
        if self.mode == 'RECENTER':
            if self.center_instructions:
                instr = self.center_instructions.pop(0)
                # When recentering is complete, switch back to PATH and recompute
                if not self.center_instructions:
                    self.mode = 'PATH'
                    self.recompute_path()
                return instr
            else:
                # No more recenter instructions
                self.mode = 'PATH'

        # Normal path-following mode

        # If we've finished the current instruction list and chaining is
        # enabled, mark the last target as completed and recompute a new path
        # to the next closest one (if any).
        if ENABLE_CHAIN_TARGETS and self.instr_index >= len(self.instructions):
            if self.path:
                self.completed_targets.add(self.path[-1])
            self.recompute_path()

        if self.instr_index < len(self.instructions):
            instr = self.instructions[self.instr_index]
            self.instr_index += 1
            return instr

        # No instructions left and no new path found
        return None
