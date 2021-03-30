class CooperativeLate:
    def update(self, state):
        s = state

        attack_left = (enemies_right_sprites.length == 0)

        action = {
            'left': False, #if random.randint(0,1) == 0 else True,
            'right': False, #if random.randint(0,1) == 0 else True,
            'shoot': False if random.randint(0,1) == 0 else True
        }
        return action