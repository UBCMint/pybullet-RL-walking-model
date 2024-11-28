import random



class Headset:
    def __init__(self):
        self.num = 0
        self.prev_action = 1
        self.actions = [0, 1]
    

    def random_action(self):
        int = random.randint(0, 1)
        action = self.actions[int]
        self.prev_action = action
        return action



    