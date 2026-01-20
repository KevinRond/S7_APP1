from typing import List
from swiplserver import PrologThread

from prolog_helpers import get_distance_between, get_straight_line_distance, get_successors
from node import Node


class A_Star:
    prolog_thread = None
    current_node: Node = None
    open_set: List[Node] = []
    closed_set: List[Node] = []
    destination = None

    def __init__(self, prolog_thread: PrologThread, start_node: Node, destination):
        self.prolog_thread = prolog_thread
        self.open_set.append(start_node)
        self.destination = destination

    def find_path(self):
        while self.open_set:
            if self.current_node is not None: print(f"current node: {self.current_node.current_position}")
            print(f"open set: {self.open_set}")
            print(f"closed set: {self.closed_set}")
            self.current_node = self.find_next_current()

            if self.current_node.current_position == self.destination:
                print(f"Path found! Total distance was {self.current_node.g_score} ")
                return

            self.open_set.remove(self.current_node)
            self.closed_set.append(self.current_node)


            successors = self.current_node.get_successors()
            for successor in successors:
                new_node = self.create_node(self.current_node.current_position, successor)
                self.open_set.append(new_node)


    def find_next_current(self):
        scores_by_node = {}
        for node in self.open_set:
            scores_by_node[node.f_score] = node

        # if self.current_node is not None: 
            # print(f"finding next node when {self.current_node.current_position} is current node")
            # print(scores_by_node)

        lowest_score = 1000
        for score in scores_by_node.keys():
            if score < lowest_score:
                lowest_score = score

        next_node = scores_by_node[lowest_score]
        return next_node

    def create_node(self, current_city, city_name):
        node_successors = get_successors(self.prolog_thread, city_name)
        node_g_score = get_distance_between(self.prolog_thread, current_city, city_name) + self.current_node.g_score
        node_h_score = get_straight_line_distance(self.prolog_thread, city_name)
        node = Node(city_name, node_successors, current_city, node_g_score, node_h_score)
        # if (city_name == 'fagaras'): print(f"aaaaaaaaaaah {node}")
        return node
    
    def __repr__(self):
        return f"""
        A_Star Instance:\n
        open_set='{self.open_set}',\n
        closed_set='{self.closed_set}',\n
        """