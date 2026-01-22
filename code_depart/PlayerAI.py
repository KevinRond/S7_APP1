from A_star import A_star
from FuzzyObstacleController import FuzzyObstacleController


# Test-only option: when enabled, the AI will chain through
# all reachable targets one by one. Set to False or comment out
# the related code when you no longer need this behaviour.
ENABLE_CHAIN_TARGETS = True

# HOMING mode: fine-tuned navigation to collect items
HOMING_ACTIVATION_DISTANCE = 60  # pixels - distance pour activer HOMING
PRECISION_THRESHOLD = 8  # pixels - précision finale pour considérer qu'on est arrivé

# Fuzzy logic for obstacle avoidance
OBSTACLE_DANGER_RADIUS = 40  # pixels - rayon de danger autour des obstacles
OBSTACLE_CRITICAL_RADIUS = 20  # pixels - rayon critique (très dangereux)
SPEED_REDUCTION_FACTOR = 0.6  # facteur de réduction de vitesse près des obstacles


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
        self.mode = 'PATH'        # 'PATH', 'RECENTER', or 'HOMING'
        self.center_instructions = []  # recenter sequence when in RECENTER mode
        # Positions (row, col) of targets already visited when chaining
        # multiple goals (used only if ENABLE_CHAIN_TARGETS is True).
        self.completed_targets = set()
        self.original_speed = player.speed  # Sauvegarder la vitesse originale
        self.current_target = None  # L'item ciblé en mode HOMING
        
        # Fuzzy controller for obstacle avoidance
        self.fuzzy_controller = FuzzyObstacleController()

    # ---------------- Pixel <-> tile conversions ----------------

    def player_tile(self):
        """Return the tile indices (row, col) where the player currently is."""
        row = int(self.player.y / self.maze.tile_size_y)
        col = int(self.player.x / self.maze.tile_size_x)
        return row, col

    # ---------------- Item detection ----------------

    def get_nearest_item_in_perception(self):
        """Trouve l'item (coin ou treasure) le plus proche dans le rayon de perception.
        
        Retourne le pygame.Rect de l'item le plus proche, ou None.
        """
        perception = self.maze.make_perception_list(self.player, None)
        item_list = perception[2]  # coins + treasures
        
        if not item_list:
            return None
        
        player_cx = self.player.x + self.player.size_x / 2
        player_cy = self.player.y + self.player.size_y / 2
        
        # Trouver l'item le plus proche (distance euclidienne)
        def distance_to_item(item):
            dx = item.centerx - player_cx
            dy = item.centery - player_cy
            return (dx**2 + dy**2)**0.5
        
        nearest = min(item_list, key=distance_to_item)
        return nearest

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

    # ---------------- Fuzzy logic for obstacle avoidance ----------------

    def compute_obstacle_danger(self, intended_direction):
        """Calcule le niveau de danger des obstacles avec logique floue scikit-fuzzy.
        
        Args:
            intended_direction: 'UP', 'DOWN', 'LEFT', 'RIGHT' - direction voulue
        
        Returns:
            dict avec:
            - 'danger_score': float 0-1 (0=safe, 1=très dangereux)
            - 'suggested_direction': direction alternative ou None
            - 'speed_multiplier': float (0.5-1.0) pour ajuster la vitesse
        """
        perception = self.maze.make_perception_list(self.player, None)
        obstacle_list = perception[1]  # Liste des obstacles
        
        if not obstacle_list:
            return {'danger_score': 0.0, 'suggested_direction': None, 'speed_multiplier': 1.0}
        
        player_cx = self.player.x + self.player.size_x / 2
        player_cy = self.player.y + self.player.size_y / 2
        
        # Calculer le vecteur de mouvement prévu
        move_vector = {
            'UP': (0, -1),
            'DOWN': (0, 1),
            'LEFT': (-1, 0),
            'RIGHT': (1, 0)
        }.get(intended_direction, (0, 0))
        
        max_danger = 0.0
        max_speed_mult = 1.0
        dangerous_obstacles = []
        
        for obstacle in obstacle_list:
            dx = obstacle.centerx - player_cx
            dy = obstacle.centery - player_cy
            distance = (dx**2 + dy**2)**0.5
            
            # Calculer l'alignement avec la direction de mouvement
            # Produit scalaire normalisé: -1 (opposé) à 1 (aligné)
            if distance > 0:
                obstacle_direction = (dx / distance, dy / distance)
                alignment = obstacle_direction[0] * move_vector[0] + \
                           obstacle_direction[1] * move_vector[1]
            else:
                alignment = 1.0  # On est sur l'obstacle!
            
            # *** UTILISER LE CONTRÔLEUR FUZZY ***
            fuzzy_result = self.fuzzy_controller.compute(
                min(distance, 100),  # Clamp à 100px max
                alignment            # -1 à 1
            )
            
            danger = fuzzy_result['danger_score']
            speed = fuzzy_result['speed_multiplier']
            
            if danger > 0.1:  # Seuil de significativité
                dangerous_obstacles.append({
                    'obstacle': obstacle,
                    'distance': distance,
                    'danger': danger,
                    'dx': dx,
                    'dy': dy,
                    'alignment': alignment
                })
            
            # Garder le pire danger
            if danger > max_danger:
                max_danger = danger
                max_speed_mult = speed
        
        # Déterminer une direction alternative si nécessaire
        suggested_direction = None
        if max_danger > 0.5 and dangerous_obstacles:  # Seuil de danger significatif
            # Trouver une direction perpendiculaire pour contourner
            # Moyenne pondérée par le danger
            total_weight = sum(obs['danger'] for obs in dangerous_obstacles)
            avg_dx = sum(obs['dx'] * obs['danger'] for obs in dangerous_obstacles) / total_weight
            avg_dy = sum(obs['dy'] * obs['danger'] for obs in dangerous_obstacles) / total_weight
            
            # Logique de contournement
            if intended_direction in ['UP', 'DOWN']:
                # Mouvement vertical, contourner horizontalement
                if abs(avg_dx) > 5:  # Obstacle à gauche ou droite
                    suggested_direction = 'LEFT' if avg_dx > 0 else 'RIGHT'
            else:  # LEFT ou RIGHT
                # Mouvement horizontal, contourner verticalement
                if abs(avg_dy) > 5:  # Obstacle en haut ou en bas
                    suggested_direction = 'UP' if avg_dy > 0 else 'DOWN'
        
        return {
            'danger_score': max_danger,
            'suggested_direction': suggested_direction,
            'speed_multiplier': max_speed_mult
        }

    def apply_speed_adjustment(self, speed_multiplier):
        """Ajuste la vitesse du joueur selon le multiplicateur."""
        self.player.speed = int(self.original_speed * speed_multiplier)
        # Minimum 1 pour éviter que le joueur ne bouge plus
        if self.player.speed < 1:
            self.player.speed = 1

    # ---------------- HOMING mode: precise navigation to items ----------------

    def compute_direction_to_target(self, target_rect):
        """Calcule la direction simple vers un item (sans logique floue).
        
        Returns:
            Direction ('UP', 'DOWN', 'LEFT', 'RIGHT') ou None si arrivé.
        """
        player_cx = self.player.x + self.player.size_x / 2
        player_cy = self.player.y + self.player.size_y / 2
        
        dx = target_rect.centerx - player_cx
        dy = target_rect.centery - player_cy
        
        # Prioriser le mouvement le plus important (Manhattan)
        if abs(dx) > PRECISION_THRESHOLD and abs(dx) >= abs(dy):
            return 'RIGHT' if dx > 0 else 'LEFT'
        elif abs(dy) > PRECISION_THRESHOLD:
            return 'DOWN' if dy > 0 else 'UP'
        else:
            return None  # Arrivé à destination

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

        # HOMING mode: precise navigation to nearby items with obstacle avoidance
        if self.mode == 'HOMING':
            # Vérifier si l'item cible existe encore
            target = self.get_nearest_item_in_perception()
            
            if target is None:
                # Plus d'item visible, retour au mode PATH
                self.mode = 'PATH'
                self.current_target = None
                self.apply_speed_adjustment(1.0)  # Restaurer vitesse normale
                return self.get_next_instruction()
            
            # Calculer la direction vers l'item
            intended_direction = self.compute_direction_to_target(target)
            
            if intended_direction is None:
                # Arrivé à l'item, retour au mode PATH
                self.mode = 'PATH'
                self.current_target = None
                self.apply_speed_adjustment(1.0)
                return self.get_next_instruction()
            
            # *** LOGIQUE FLOUE: Évaluer les obstacles ***
            obstacle_info = self.compute_obstacle_danger(intended_direction)
            
            # Ajuster la vitesse selon le danger
            self.apply_speed_adjustment(obstacle_info['speed_multiplier'])
            
            # Si danger élevé et direction alternative suggérée, l'utiliser
            if obstacle_info['danger_score'] > 0.5 and obstacle_info['suggested_direction']:
                return obstacle_info['suggested_direction']
            else:
                return intended_direction

        # Normal path-following mode (PATH)

        # Vérifier si un item est proche pour passer en mode HOMING
        nearest_item = self.get_nearest_item_in_perception()
        if nearest_item:
            player_cx = self.player.x + self.player.size_x / 2
            player_cy = self.player.y + self.player.size_y / 2
            dx = nearest_item.centerx - player_cx
            dy = nearest_item.centery - player_cy
            distance = (dx**2 + dy**2)**0.5
            
            # Activer HOMING si assez proche
            if distance < HOMING_ACTIVATION_DISTANCE:
                self.mode = 'HOMING'
                self.current_target = nearest_item
                return self.get_next_instruction()

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
            
            # *** LOGIQUE FLOUE: Vérifier les obstacles même en mode PATH ***
            obstacle_info = self.compute_obstacle_danger(instr)
            self.apply_speed_adjustment(obstacle_info['speed_multiplier'])
            
            # Si danger critique, utiliser direction alternative
            if obstacle_info['danger_score'] > 0.7 and obstacle_info['suggested_direction']:
                return obstacle_info['suggested_direction']
            
            return instr

        # No instructions left and no new path found
        return None
