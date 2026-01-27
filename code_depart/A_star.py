import heapq

from Constants import START, EXIT, COIN, TREASURE, OBSTACLE, MONSTER, DOOR


# Liste des types d'objets que l'on peut cibler avec A*.
# Tu peux facilement modifier cette constante (ajouter/enlever un type).
TARGET_TILES = [EXIT, COIN, TREASURE]

# If True, the EXIT is only targeted when there are no other targets
# (coins/treasures) left on the map.
GO_TO_EXIT_LAST = False

# Nombre de cibles les plus proches (en distance de Manhattan) pour
# lesquelles on calcule un vrai chemin A* avant de choisir la meilleure.
# Modifie simplement cette constante pour tester d'autres valeurs (3, 5, ...).
NEAREST_TARGETS = 3


class A_star:
    def __init__(self, maze_grid, blocked_tiles=None):
        self.grid = maze_grid
        self.rows = len(maze_grid)
        self.cols = len(maze_grid[0])
        self.blocked_tiles = set(blocked_tiles) if blocked_tiles else set()

    def is_walkable(self, i, j):
        if i < 0 or j < 0 or i >= self.rows or j >= self.cols:
            return False
        if (i, j) in self.blocked_tiles:
            return False
        cell = self.grid[i][j]
        # On bloque seulement les murs.
        # Les monstres, pièces, trésors, portes et la sortie restent atteignables
        # (les portes sont traitées comme "ouvertes" pour l'algorithme de cheminement).
        return cell not in ('1')

    def manhattan(self, a, b):
        # Distance de Manhattan entre deux cases (ligne,colonne)
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def heuristic_to_goals(self, node, goals):
        # Heuristique = distance de Manhattan jusqu'au but le plus proche
        return min(self.manhattan(node, g) for g in goals)

    def neighbors(self, node):
        i, j = node
        for di, dj in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            ni, nj = i + di, j + dj
            if self.is_walkable(ni, nj):
                yield (ni, nj)

    def get_target_positions(self):
        """Renvoie la liste des positions (i, j) des tuiles cibles dans la grille."""
        targets = []
        for i, row in enumerate(self.grid):
            for j, cell in enumerate(row):
                if cell in TARGET_TILES:
                    targets.append((i, j))
        return targets

    def reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path   # list of (i, j)

    def _find_path_to_single_goal(self, start, goal):
        """A* standard vers une unique cible 'goal'.

        Retourne le chemin sous forme de liste [(i,j), ...] ou [] si aucun
        chemin n'existe.
        """
        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.manhattan(start, goal)}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.neighbors(current):
                tentative_g = g_score[current] + 1
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.manhattan(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []

    def find_path_from(self, start, excluded_targets=None):
        """Trouve un chemin A* depuis 'start' jusqu'à l'objet atteignable le plus proche.

        start : tuple (ligne, colonne) de la position de départ (par ex. position du joueur).
        excluded_targets : ensemble ou liste de positions (i, j) à ignorer comme cibles.
        Retourne une liste de cases [(i,j), ...] jusqu'à la cible la plus proche,
        ou [] s'il n'y a aucun chemin vers une cible.
        """
        targets = self.get_target_positions()

        # Optional behaviour: only allow EXIT as a target once all other targets are gone.
        if GO_TO_EXIT_LAST:
            non_exit_targets = [(i, j) for (i, j) in targets if self.grid[i][j] != EXIT]
            if non_exit_targets:
                targets = non_exit_targets

        # Optionnel : retirer certaines cibles (par ex. déjà visitées).
        if excluded_targets:
            excluded = set(excluded_targets)
            targets = [t for t in targets if t not in excluded]
        if not targets:
            return []

        # Si la position de départ est déjà sur une tuile cible, le chemin est trivial.
        if start in targets:
            return [start]

        # On trie toutes les cibles par distance de Manhattan depuis la case de départ.
        targets_sorted = sorted(targets, key=lambda t: self.manhattan(start, t))

        # On commence par tester les NEAREST_TARGETS plus proches, comme demandé.
        primary_candidates = targets_sorted[:NEAREST_TARGETS]
        secondary_candidates = targets_sorted[NEAREST_TARGETS:]

        best_path = None

        # 1) Essayer les cibles les plus proches en Manhattan.
        for goal in primary_candidates:
            path = self._find_path_to_single_goal(start, goal)
            if path and (best_path is None or len(path) < len(best_path)):
                best_path = path

        if best_path is not None:
            return best_path

        # 2) Si aucune des NEAREST_TARGETS n'est atteignable, on tente les autres
        #    cibles restantes, toujours triées par distance de Manhattan.
        for goal in secondary_candidates:
            path = self._find_path_to_single_goal(start, goal)
            if path and (best_path is None or len(path) < len(best_path)):
                best_path = path

        return best_path or []  # aucun chemin vers une cible