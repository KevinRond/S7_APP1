from A_star import A_star
from FuzzyObstacleController import FuzzyObstacleController


# Test-only option: when enabled, the AI will chain through
# all reachable targets one by one. Set to False or comment out
# the related code when you no longer need this behaviour.
ENABLE_CHAIN_TARGETS = True

# HOMING mode: fine-tuned navigation to collect items
HOMING_ACTIVATION_DISTANCE = 60  # pixels - distance pour activer HOMING
PRECISION_THRESHOLD = 8  # pixels - précision finale pour considérer qu'on est arrivé




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
        self.mode = 'PATH'        # 'PATH', 'RECENTER', 'HOMING', or 'SQUEEZE'
        self.center_instructions = []  # recenter sequence when in RECENTER mode
        # Positions (row, col) of targets already visited when chaining
        # multiple goals (used only if ENABLE_CHAIN_TARGETS is True).
        self.completed_targets = set()
        self.original_speed = player.speed  # Sauvegarder la vitesse originale
        self.current_target = None  # L'item ciblé en mode HOMING
        
        # Fuzzy controller for obstacle avoidance
        self.fuzzy_controller = FuzzyObstacleController()
        
        # Squeeze mode state
        self.squeeze_instructions = []
        self.intended_direction = None
        self.blocking_obstacle = None

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

    # ---------------- Squeeze mode: navigation when stuck ----------------

    def is_obstacle_blocking_player(self, direction):
        """Vérifie si un obstacle bloque le joueur dans une direction donnée.
        
        Args:
            direction: 'UP', 'DOWN', 'LEFT', 'RIGHT' - direction où le joueur veut aller
        
        Returns:
            True si un obstacle bloque cette direction, False sinon
        """
        perception = self.maze.make_perception_list(self.player, None)
        obstacle_list = perception[1]
        
        if not obstacle_list:
            return False
        
        # Trouver les obstacles bloquants dans cette direction
        blocking_obstacles = [obs for obs in obstacle_list 
                            if self.is_obstacle_blocking_path(obs, direction)]
        
        return len(blocking_obstacles) > 0

    def is_obstacle_blocking_path(self, obstacle, intended_direction):
        """Vérifie si un obstacle bloque le chemin prévu en utilisant les hitboxes exactes."""
        # Demi-largeurs pour la détection de collision
        player_half_width = self.player.size_x / 2
        player_half_height = self.player.size_y / 2
        obstacle_half_width = obstacle.width / 2
        obstacle_half_height = obstacle.height / 2
        
        player_cx = self.player.x + player_half_width
        player_cy = self.player.y + player_half_height
        
        dx = obstacle.centerx - player_cx
        dy = obstacle.centery - player_cy
        
        # Collision corridor = somme des demi-largeurs/hauteurs
        horizontal_corridor = player_half_width + obstacle_half_width
        vertical_corridor = player_half_height + obstacle_half_height
        
        # Vérifier si l'obstacle est devant dans la direction voulue ET dans le corridor de collision
        if intended_direction == 'UP' and dy < 0 and abs(dx) < horizontal_corridor:
            return True
        elif intended_direction == 'DOWN' and dy > 0 and abs(dx) < horizontal_corridor:
            return True
        elif intended_direction == 'LEFT' and dx < 0 and abs(dy) < vertical_corridor:
            return True
        elif intended_direction == 'RIGHT' and dx > 0 and abs(dy) < vertical_corridor:
            return True
        
        return False

    def measure_gap(self, obstacle, wall_list, direction):
        """Mesure l'espace entre l'obstacle et le mur le plus proche dans une direction."""
        if direction == 'LEFT':
            obstacle_edge = obstacle.left
            walls_to_left = [w for w in wall_list if w.right <= obstacle_edge and 
                           abs(w.centery - obstacle.centery) < self.maze.tile_size_y * 2]
            if walls_to_left:
                nearest_wall = max(walls_to_left, key=lambda w: w.right)
                return obstacle_edge - nearest_wall.right
            return 999  # Pas de mur = grand espace
        
        elif direction == 'RIGHT':
            obstacle_edge = obstacle.right
            walls_to_right = [w for w in wall_list if w.left >= obstacle_edge and 
                            abs(w.centery - obstacle.centery) < self.maze.tile_size_y * 2]
            if walls_to_right:
                nearest_wall = min(walls_to_right, key=lambda w: w.left)
                return nearest_wall.left - obstacle_edge
            return 999
        
        elif direction == 'UP':
            obstacle_edge = obstacle.top
            walls_above = [w for w in wall_list if w.bottom <= obstacle_edge and 
                         abs(w.centerx - obstacle.centerx) < self.maze.tile_size_x * 2]
            if walls_above:
                nearest_wall = max(walls_above, key=lambda w: w.bottom)
                return obstacle_edge - nearest_wall.bottom
            return 999
        
        elif direction == 'DOWN':
            obstacle_edge = obstacle.bottom
            walls_below = [w for w in wall_list if w.top >= obstacle_edge and 
                         abs(w.centerx - obstacle.centerx) < self.maze.tile_size_x * 2]
            if walls_below:
                nearest_wall = min(walls_below, key=lambda w: w.top)
                return nearest_wall.top - obstacle_edge
            return 999
        
        return 0

    def create_squeeze_instructions(self, obstacle, squeeze_direction, forward_direction):
        """Crée une séquence de mouvements pour se décaler et dégager l'obstacle.
        
        Déplace le joueur latéralement juste assez pour que l'obstacle ne bloque plus
        le chemin dans la direction voulue. Ensuite les instructions normales reprennent.
        """
        instructions = []
        player_cx = self.player.x + self.player.size_x / 2
        player_cy = self.player.y + self.player.size_y / 2
        
        # Petite marge de sécurité

        
        if squeeze_direction in ['LEFT', 'RIGHT']:
            # Bouger latéralement jusqu'à ce que l'obstacle ne soit plus dans le corridor vertical
            # Distance = distance au centre de l'obstacle + sa demi-largeur + demi-largeur joueur + marge
            clearance_needed = abs(obstacle.centerx - player_cx) + obstacle.width/2 + self.player.size_x/2
            steps = max(1, int(clearance_needed / self.player.speed))
            
            for _ in range(steps):
                instructions.append(squeeze_direction)
        
        else:  # UP or DOWN
            # Bouger latéralement jusqu'à ce que l'obstacle ne soit plus dans le corridor horizontal
            clearance_needed = abs(obstacle.centery - player_cy) + obstacle.height/2 + self.player.size_y/2
            steps = max(1, int(clearance_needed / self.player.speed))
            
            for _ in range(steps):
                instructions.append(squeeze_direction)
        
        return instructions

    def find_squeeze_path(self, intended_direction):
        """Trouve le meilleur passage pour contourner un obstacle bloquant."""
        perception = self.maze.make_perception_list(self.player, None)
        obstacle_list = perception[1]
        wall_list = perception[0]
        
        if not obstacle_list:
            return []
        #TODO: tu peux srm juste prendre l onstacle le plus proche
        # Trouver l'obstacle bloquant
        blocking_obstacles = [obs for obs in obstacle_list 
                            if self.is_obstacle_blocking_path(obs, intended_direction)]
        
        if not blocking_obstacles:
            return []
        
        # Prendre l'obstacle le plus proche
        player_cx = self.player.x + self.player.size_x / 2
        player_cy = self.player.y + self.player.size_y / 2
        obstacle = min(blocking_obstacles, 
                      key=lambda o: (o.centerx - player_cx)**2 + (o.centery - player_cy)**2)
        
        self.blocking_obstacle = obstacle
        
        # Déterminer les directions perpendiculaires
        min_clearance_x = self.player.size_x
        min_clearance_y = self.player.size_y
        
        if intended_direction in ['UP', 'DOWN']:
            # Mesurer les passages à gauche et à droite
            left_gap = self.measure_gap(obstacle, wall_list, 'LEFT')
            right_gap = self.measure_gap(obstacle, wall_list, 'RIGHT')
            
            # Choisir le passage le plus large si le joueur peut y passer
            if left_gap >= min_clearance_x and (right_gap < min_clearance_x or left_gap >= right_gap):
                return self.create_squeeze_instructions(obstacle, 'LEFT', intended_direction)
            elif right_gap >= min_clearance_x:
                return self.create_squeeze_instructions(obstacle, 'RIGHT', intended_direction)
        else:  # LEFT or RIGHT
            # Mesurer les passages en haut et en bas
            up_gap = self.measure_gap(obstacle, wall_list, 'UP')
            down_gap = self.measure_gap(obstacle, wall_list, 'DOWN')
            
            if up_gap >= min_clearance_y and (down_gap < min_clearance_y or up_gap >= down_gap):
                return self.create_squeeze_instructions(obstacle, 'UP', intended_direction)
            elif down_gap >= min_clearance_y:
                return self.create_squeeze_instructions(obstacle, 'DOWN', intended_direction)
        
        return []

    def is_obstacle_cleared(self, obstacle, intended_direction):
        """Vérifie si on a dépassé l'obstacle en utilisant les dimensions exactes."""
        if obstacle is None:
            return True
        
        player_cx = self.player.x + self.player.size_x / 2
        player_cy = self.player.y + self.player.size_y / 2
        
        dx = obstacle.centerx - player_cx
        dy = obstacle.centery - player_cy
        
        # Distance de clearance = demi-dimensions combinées + petite marge
        clearance_x = (self.player.size_x + obstacle.width) / 2 + 2
        clearance_y = (self.player.size_y + obstacle.height) / 2 + 2
        
        # L'obstacle est derrière nous dans la direction voulue
        if intended_direction == 'UP' and dy > clearance_y:
            return True
        elif intended_direction == 'DOWN' and dy < -clearance_y:
            return True
        elif intended_direction == 'LEFT' and dx > clearance_x:
            return True
        elif intended_direction == 'RIGHT' and dx < -clearance_x:
            return True
        
        return False

    def activate_squeeze_mode(self, blocked_direction):
        """Active le mode squeeze pour contourner un obstacle bloquant.
        
        Args:
            blocked_direction: 'UP', 'DOWN', 'LEFT', 'RIGHT' - direction bloquée
        
        Returns:
            True si squeeze mode activé avec succès, False sinon
        """
        self.intended_direction = blocked_direction
        squeeze_path = self.find_squeeze_path(blocked_direction)
        
        if squeeze_path:
            self.mode = 'SQUEEZE'
            self.squeeze_instructions = squeeze_path
            return True
        else:
            # Impossible de contourner, replanifier
            self.recompute_path()
            return False

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
        
    def mark_tile_completed(self, x, y):
        """Marque la tuile actuelle du joueur comme cible complétée."""
        row = int(y / self.maze.tile_size_y)
        col = int(x / self.maze.tile_size_x)
        print("current_tile marked as completed:", (row, col))
        self.completed_targets.add((row, col))

    def get_next_instruction(self):
        """Return the next direction for the player, or None if nothing to do."""

        # SQUEEZE mode: navigating around a blocking obstacle
        if self.mode == 'SQUEEZE':
            if self.squeeze_instructions:
                instr = self.squeeze_instructions.pop(0)
                
                # Vérifier si on a dépassé l'obstacle
                if not self.squeeze_instructions and self.is_obstacle_cleared(self.blocking_obstacle, self.intended_direction):
                    # Obstacle dépassé, retour au mode normal
                    self.mode = 'PATH'
                    self.blocking_obstacle = None
                    self.apply_speed_adjustment(1.0)
                
                return instr
            else:
                # Plus d'instructions de squeeze, retour au mode PATH
                self.mode = 'PATH'
                self.blocking_obstacle = None
                self.apply_speed_adjustment(1.0)
                return self.recompute_path()

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
            print("HOMING")
            # Vérifier si l'item cible existe encore
            target = self.get_nearest_item_in_perception()
            
            if target is None:
                # Plus d'item visible, retour au mode PATH
                print("No target found, switching to PATH")
                self.mode = 'PATH'
                self.current_target = None
                self.apply_speed_adjustment(1.0)  # Restaurer vitesse normale
                self.mark_tile_current_tile_completed()
                return self.recompute_path()
            
            # Calculer la direction vers l'item
            intended_direction = self.compute_direction_to_target(target)
            
            if intended_direction is None:
                # Arrivé à l'item, retour au mode PATH
                print("No intended direction found, switching to PATH")
                self.mode = 'PATH'
                self.current_target = None
                self.apply_speed_adjustment(1.0)
                self.mark_tile_current_tile_completed()
                return self.recompute_path()
            
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
        
        self.intended_direction = instr  # Sauvegarder pour squeeze mode
        
        # *** LOGIQUE FLOUE: Vérifier les obstacles même en mode PATH ***
        obstacle_info = self.compute_obstacle_danger(instr)
        self.apply_speed_adjustment(obstacle_info['speed_multiplier'])
        
        # Si danger critique, utiliser direction alternative
        if obstacle_info['danger_score'] > 0.7 and obstacle_info['suggested_direction']:
            return obstacle_info['suggested_direction']
        
        print(f"instr:", instr)
        return instr
