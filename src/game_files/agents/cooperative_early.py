from game_files.agents.uncooperative import Uncooperative

GAME_ENEMIES = 9*4*3*2 # across * row per color * color * sides

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
        total_enemies = num_left_enemies + num_right_enemies

        if total_enemies > GAME_ENEMIES*.8: #num_right_enemies > 72 and num_left_enemies > 72:
            attack_left = False
        elif total_enemies > GAME_ENEMIES*.65: #num_left_enemies > 62:
            attack_left = True
        elif num_left_enemies > (GAME_ENEMIES/2)*(4.0/12) and num_right_enemies > (GAME_ENEMIES/2)*(3.0/12):#num_right_enemies > 50 and num_left_enemies > 45:
            attack_left = False
        elif num_left_enemies > (GAME_ENEMIES/2)*(2.0/12) and num_right_enemies > (GAME_ENEMIES/2)*(1.5/12): #num_left_enemies > 35:
            attack_left = True
        elif num_left_enemies > (GAME_ENEMIES/2)*(1.0/12) and num_right_enemies > (GAME_ENEMIES/2)*(.5/12): #num_right_enemies > 15 and num_left_enemies > 15:
            attack_left = False
        elif num_left_enemies > 0:
            attack_left = True
        else:
            attack_left = False
        return attack_left
        