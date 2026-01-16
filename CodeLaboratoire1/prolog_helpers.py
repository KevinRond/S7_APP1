
from swiplserver import PrologThread


def get_straight_line_distance(prolog_thread: PrologThread, city):
    query = prolog_thread.query(f"h({city}, H)")
    return query[0]['H']

def get_distance_between(prolog_thread: PrologThread, currentPosition, destination):
    query = prolog_thread.query(f"d({currentPosition}, {destination}, D)")
    return query[0]['D']

def get_successors(prolog_thread: PrologThread, city):
    query = prolog_thread.query(f"s({city}, S)")
    return query[0]['S']