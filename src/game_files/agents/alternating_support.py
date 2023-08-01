# import random
from game_files.agents.uncooperative import Uncooperative

class AlternatingSupport(Uncooperative):
    def __init__(self):
        super().__init__()
        # self.support = random.randint(1,2)
        # self.min_x = 0
        
    def run_ai(self, state):
        # print("IN ALTERNATING SUPPPORT------")
        s = state
        enemies_left_positions = s['enemies_left_positions']
        # enemies_right_positions = s['enemies_right_positions']
        num_left_enemies = len(enemies_left_positions)

        num_enemies_shot = s['num_enemies_shot']
        num_enemies_shot = num_enemies_shot + 1
        # num_right_enemies = len(enemies_right_positions)
        # print("num_enemies_left", num_left_enemies)

        val = num_enemies_shot / 10
        if round(val) - val >= 0:
            # print("IN IF")
            # support starting support player
            # s['ai_supporting'] = self.support
            self.support = self.start_support
        elif round(val) - val < 0:
            # print("IN ELSE")
            # support other player
            if self.start_support == 1:
                # s['ai_supporting'] = 2
                self.support = 2
            elif self.start_support == 2:
                # s['ai_supporting'] = 1
                self.support = 1
        # print("--- support ", self.support)
        return True

