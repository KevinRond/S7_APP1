from typing import List
from swiplserver import PrologThread

from node import Node


class A_Star:
    prolog_thread = None
    current_node = None
    open_set: List[Node] = []
    closed_set = []

    def __init__(self, prolog_thread: PrologThread, start_node: Node):
        self.prolog_thread = prolog_thread
        self.current_node = start_node
        self.open_set.append(start_node)

    def find_next_current(self):
        scores_by_node = {}
        for node in self.open_set:
            scores_by_node[node.f_score] = node

        lowest_score = 1000
        for score in scores_by_node.keys():
            if score < lowest_score:
                lowest_score = score

        next_node = scores_by_node[lowest_score]
        return next_node

    def find_path(self):
        while (self.current_node.current_position != "bucharest"):
            self.current_node = self.find_next_current()

    
    def __repr__(self):
        return f"""
        A_Star Instance:\n
        open_set='{self.open_set}',\n
        closed_set='{self.closed_set}',\n
        """