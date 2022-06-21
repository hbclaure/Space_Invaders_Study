import random

class RandomShoot:
    def update(self, state):
        #TODO: make it non-random
        action = {
            'left': False, #if random.randint(0,1) == 0 else True,
            'right': False, #if random.randint(0,1) == 0 else True,
            'shoot': False if random.randint(0,1) == 0 else True
        }
        return action
