import heapq

from Constants import START, EXIT, COIN, TREASURE, OBSTACLE, MONSTER, DOOR


# Liste des types d'objets que l'on peut cibler avec A*.
# Tu peux facilement modifier cette constante (ajouter/enlever un type).
TARGET_TILES = [EXIT]


class A_star:
    def __init__(self, maze_grid):
        self.grid = maze_grid
        self.rows = len(maze_grid)
        self.cols = len(maze_grid[0])

    def is_walkable(self, i, j):
        if i < 0 or j < 0 or i >= self.rows or j >= self.cols:
            return False
        cell = self.grid[i][j]
        # On bloque seulement les murs et les obstacles.
        # Les monstres, pièces, trésors, portes et la sortie restent atteignables
        # (les portes sont traitées comme "ouvertes" pour l'algorithme de cheminement).
        return cell not in ('1', OBSTACLE)

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

    def find_path_from(self, start):
        """Trouve un chemin A* depuis 'start' jusqu'à l'objet atteignable le plus proche.

        start : tuple (ligne, colonne) de la position de départ (par ex. position du joueur).
        Retourne une liste de cases [(i,j), ...] jusqu'à la cible la plus proche,
        ou [] s'il n'y a aucun chemin vers une cible.
        """
        targets = self.get_target_positions()
        if not targets:
            return []

        # Si la position de départ est déjà sur une tuile cible, le chemin est trivial.
        if start in targets:
            return [start]

        goals = set(targets)

        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: self.heuristic_to_goals(start, goals)}

        while open_set:
            _, current = heapq.heappop(open_set)
            if current in goals:
                return self.reconstruct_path(came_from, current)

            for neighbor in self.neighbors(current):
                tentative_g = g_score[current] + 1
                if tentative_g < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score[neighbor] = tentative_g + self.heuristic_to_goals(neighbor, goals)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return []  # aucun chemin vers une cible