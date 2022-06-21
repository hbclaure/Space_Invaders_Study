from game_files.agents.uncooperative import Uncooperative

class CooperativeEarly(Uncooperative):
    def __init__(self):
        super().__init__()
        self.min_x = 0
        
    def check_attack_left(self, state):
        s = state
        enemies_left_positions = s['enemies_left_positions']
        enemies_right_positions = s['enemies_right_positions']
        num_left_enemies = len(enemies_left_positions)
        num_right_enemies = len(enemies_right_positions)

        if num_right_enemies > 22 and num_left_enemies > 22:
            attack_left = False
        elif num_left_enemies > 15:
            attack_left = True
        elif num_right_enemies > 9 and num_left_enemies > 7:
            attack_left = False
        elif num_left_enemies > 0:
            attack_left = True
        else:
            attack_left = False
        return attack_left
        