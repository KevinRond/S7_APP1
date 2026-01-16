class Node:
    current_position = None
    successors = []
    came_from = None
    g_score = None
    h_score = None
    f_score = None

    def __init__(self, current_position, successors, came_from, g_score, h_score):
        self.current_position = current_position
        self.successors = successors
        self.came_from = came_from
        self.g_score = g_score
        self.h_score = h_score
        self.f_score = g_score + h_score

    def get_current_position(self):
        return self.current_position
    
    def get_successors(self):
        return self.successors
    
    def get_f_score(self):
        return self.f_score
    
    # Use this if you want just the name
    def __repr__(self):
        return self.current_position.capitalize()
    
    # Use this if you want full node info
    # def __repr__(self):
    #     return f"""
    #     Node Instance:\n
    #     current_position='{self.current_position}',\n
    #     successors='{self.successors}',\n
    #     came_from='{self.came_from}',\n
    #     g_score='{self.g_score}',\n
    #     h_score='{self.h_score}',\n
    #     f_score='{self.f_score}',
    #     """