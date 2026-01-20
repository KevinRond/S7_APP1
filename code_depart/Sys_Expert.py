class Systeme_Expert:
    door = []

    def __init__(self, thread):
        self.thread = thread

    def setDoor(self, door):
        # print(f"door object: {door}")
        self.door = door

    def getSolution(self):
        query = self.thread.query(f"solve({self.door}, Result)")
        return query[0]['Result']