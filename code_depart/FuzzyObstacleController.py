import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl


class FuzzyObstacleController:
    """Fuzzy logic controller for obstacle avoidance using scikit-fuzzy."""
    
    def __init__(self):
        # Input variables
        # Distance: 0-100 pixels (normalized range for obstacle perception)
        self.obstacle_distance = ctrl.Antecedent(np.linspace(0, 100, 1000), 'obstacle_distance')
        
        # Path alignment: -1 to 1 (-1=opposite direction, 0=perpendicular, 1=same direction)
        self.path_alignment = ctrl.Antecedent(np.linspace(-1, 1, 1000), 'path_alignment')
        
        # Output variables
        # Danger score: 0 to 1 (0=safe, 1=very dangerous)
        self.danger_score = ctrl.Consequent(np.linspace(0, 1, 1000), 'danger_score', defuzzify_method='centroid')
        
        # Speed multiplier: 0.5 to 1.0 (how much to slow down)
        self.speed_multiplier = ctrl.Consequent(np.linspace(0.5, 1.0, 1000), 'speed_multiplier', defuzzify_method='centroid')
        
        # Accumulation method
        self.danger_score.accumulation_method = np.fmax
        self.speed_multiplier.accumulation_method = np.fmax
        
        # ============ Membership functions ============
        
        # Obstacle distance (in pixels)
        # Critical: 0-5px, Close: 5-15px, Medium: 15-30px, Far: 30+px
        self.obstacle_distance['critical'] = fuzz.trapmf(self.obstacle_distance.universe, [0, 0, 3, 8])
        self.obstacle_distance['close'] = fuzz.trimf(self.obstacle_distance.universe, [5, 10, 18])
        self.obstacle_distance['medium'] = fuzz.trimf(self.obstacle_distance.universe, [15, 22, 35])
        self.obstacle_distance['far'] = fuzz.trapmf(self.obstacle_distance.universe, [30, 40, 100, 100])
        
        # Path alignment (-1 to 1)
        # Opposite: going away, Perpendicular: crossing, Aligned: heading toward
        self.path_alignment['opposite'] = fuzz.trapmf(self.path_alignment.universe, [-1.0, -1.0, -0.5, -0.1])
        self.path_alignment['perpendicular'] = fuzz.trimf(self.path_alignment.universe, [-0.3, 0.0, 0.3])
        self.path_alignment['aligned'] = fuzz.trapmf(self.path_alignment.universe, [0.1, 0.5, 1.0, 1.0])
        
        # Danger score (0 to 1)
        self.danger_score['safe'] = fuzz.trapmf(self.danger_score.universe, [0.0, 0.0, 0.2, 0.4])
        self.danger_score['moderate'] = fuzz.trimf(self.danger_score.universe, [0.3, 0.5, 0.7])
        self.danger_score['high'] = fuzz.trapmf(self.danger_score.universe, [0.6, 0.8, 1.0, 1.0])
        
        # Speed multiplier (0.5 to 1.0)
        self.speed_multiplier['slow'] = fuzz.trapmf(self.speed_multiplier.universe, [0.5, 0.5, 0.6, 0.7])
        self.speed_multiplier['moderate'] = fuzz.trimf(self.speed_multiplier.universe, [0.65, 0.75, 0.85])
        self.speed_multiplier['fast'] = fuzz.trapmf(self.speed_multiplier.universe, [0.8, 0.9, 1.0, 1.0])
        
        # ============ Fuzzy rules ============
        self.rules = []
        
        # Critical distance rules - always dangerous if very close
        self.rules.append(ctrl.Rule(self.obstacle_distance['critical'] & self.path_alignment['aligned'], 
                                    (self.danger_score['high'], self.speed_multiplier['slow'])))
        self.rules.append(ctrl.Rule(self.obstacle_distance['critical'] & self.path_alignment['perpendicular'], 
                                    (self.danger_score['high'], self.speed_multiplier['slow'])))
        self.rules.append(ctrl.Rule(self.obstacle_distance['critical'] & self.path_alignment['opposite'], 
                                    (self.danger_score['moderate'], self.speed_multiplier['moderate'])))
        
        # Close distance rules - danger depends on alignment
        self.rules.append(ctrl.Rule(self.obstacle_distance['close'] & self.path_alignment['aligned'], 
                                    (self.danger_score['high'], self.speed_multiplier['slow'])))
        self.rules.append(ctrl.Rule(self.obstacle_distance['close'] & self.path_alignment['perpendicular'], 
                                    (self.danger_score['moderate'], self.speed_multiplier['moderate'])))
        self.rules.append(ctrl.Rule(self.obstacle_distance['close'] & self.path_alignment['opposite'], 
                                    (self.danger_score['safe'], self.speed_multiplier['fast'])))
        
        # Medium distance rules
        self.rules.append(ctrl.Rule(self.obstacle_distance['medium'] & self.path_alignment['aligned'], 
                                    (self.danger_score['moderate'], self.speed_multiplier['moderate'])))
        self.rules.append(ctrl.Rule(self.obstacle_distance['medium'] & self.path_alignment['perpendicular'], 
                                    (self.danger_score['safe'], self.speed_multiplier['fast'])))
        self.rules.append(ctrl.Rule(self.obstacle_distance['medium'] & self.path_alignment['opposite'], 
                                    (self.danger_score['safe'], self.speed_multiplier['fast'])))
        
        # Far distance rules - always safe
        self.rules.append(ctrl.Rule(self.obstacle_distance['far'], 
                                    (self.danger_score['safe'], self.speed_multiplier['fast'])))
        
        # Set rule conjunction and disjunction methods
        for rule in self.rules:
            rule.and_func = np.fmin
            rule.or_func = np.fmax
        
        # Create control system
        self.control_system = ctrl.ControlSystem(self.rules)
        self.sim = ctrl.ControlSystemSimulation(self.control_system)
    
    def compute(self, obstacle_dist, path_alignment):
        """
        Compute fuzzy outputs for obstacle avoidance.
        
        Args:
            obstacle_dist: Distance to nearest obstacle in pixels (0-100)
            path_alignment: Alignment with movement direction (-1 to 1)
                           -1 = moving away, 0 = perpendicular, 1 = moving toward
        
        Returns:
            dict with 'danger_score' (0-1) and 'speed_multiplier' (0.5-1.0)
        """
        # Clamp inputs to valid ranges
        obstacle_dist = float(np.clip(obstacle_dist, 0, 100))
        path_alignment = float(np.clip(path_alignment, -1, 1))
        
        # Set inputs
        self.sim.input['obstacle_distance'] = obstacle_dist
        self.sim.input['path_alignment'] = path_alignment
        
        # Compute output
        try:
            self.sim.compute()
            danger = self.sim.output['danger_score']
            speed = self.sim.output['speed_multiplier']
        except Exception as e:
            # Fallback if computation fails
            print(f"Fuzzy computation error: {e}")
            danger = 0.0
            speed = 1.0
        
        return {
            'danger_score': danger,
            'speed_multiplier': speed
        }
