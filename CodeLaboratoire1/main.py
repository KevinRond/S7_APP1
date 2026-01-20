# Université de Sherbrooke
# Code préparé par Audrey Corbeil Therrien
# Laboratoire 1 - Interaction avec prolog

from swiplserver import PrologMQI

from a_star import A_Star
from node import Node
from prolog_helpers import get_straight_line_distance, get_successors

if __name__ == '__main__':
    with PrologMQI() as mqi:
        with PrologMQI() as mqi_file:
            with mqi_file.create_thread() as prolog_thread:
                prolog_thread.query("[prolog/roumanie]")
                arad_successors = get_successors(prolog_thread, "arad")
                arad_h_score = get_straight_line_distance(prolog_thread, "arad")
                initial_node = Node("arad", arad_successors, None, 0, arad_h_score)
                
                a_star = A_Star(prolog_thread, initial_node, "bucharest")
                a_star.find_path()
