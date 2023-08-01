import random
from game_files.agents.uncooperative import Uncooperative
from game_files.agents.training_ai import AiBehavior

class Training(AiBehavior):
    def __init__(self):
        super().__init__()
        # self.support = random.randint(1,2)
        # self.min_x = 0
        
    def run_ai(self, state):
        # print("IN ALTERNATING SUPPPORT------")
        # s = state
        # # print(s)
        # enemies_left_positions = s['enemies_left_positions']
        # # enemies_right_positions = s['enemies_right_positions']
        # num_left_enemies = len(enemies_left_positions)
        # # num_right_enemies = len(enemies_right_positions)
        # # change_team = s['enemies_mult_5']

        # # if change_team == True:
        # #     self.support = random.randint(1,2)
        
        # if num_left_enemies % 5 == 0 and self.support_changed == False:
        #     self.support = random.randint(1,2)
        #     self.support_changed = True
        # elif (num_left_enemies + 1) % 5 == 0:
        #     self.support_changed = False

        self.support = 0

        return True


        