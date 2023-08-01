# import random
from game_files.agents.uncooperative import Uncooperative

class TargetedAssistance(Uncooperative):
    def __init__(self):
        super().__init__()
        self.support = self.start_support
        # self.min_x = 0
        
    def run_ai(self, state):
        print("IN ALTERNATING SUPPPORT------")
        s = state
        enemies_left_positions = s['enemies_left_positions']
        print('HRERE')
        # enemies_right_positions = s['enemies_right_positions']
        num_left_enemies = len(enemies_left_positions)
        # num_right_enemies = len(enemies_right_positions)
        p1_score = s['player1_score']
        p2_score = s['player2_score']
        # if p1_score < 30 and p2_score < 30:
        #     self.ai_shoots = False
        # else:
        self.ai_shoots = True
        if p1_score < p2_score:
            self.support = 1
        elif p2_score < p1_score:
            self.support = 2
        return True


        