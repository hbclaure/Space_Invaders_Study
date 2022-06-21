from game_files.agents.uncooperative import Uncooperative
from game_files.agents.cooperative_early import CooperativeEarly
from game_files.agents.cooperative_late import CooperativeLate

class Apology(CooperativeLate):
    def __init__(self):
        super().__init__()
        self.min_x = 0
        self.signal_down_count = 0
        self.zombied = False
        self.zombie_go_left = True

    def update(self, state):
        s = state
        enemies_left_positions = s['enemies_left_positions']
        enemies_right_positions = s['enemies_right_positions']
        num_left_enemies = len(enemies_left_positions)
        num_right_enemies = len(enemies_right_positions)

        if num_right_enemies == 18 and self.zombied == False:
            self.strategy = self.zombie
            self.zombied = True
        
        if s['signal_down'] and self.strategy == self.zombie:
            self.signal_down_count += 1
            if self.signal_down_count == 1:
                self.strategy = self.active
        
        if s['signal_down']:
            print('DOWN_SIGNAL: ', s['signal_down'])
        
        return self.strategy(state)
    
    def zombie(self, state):
        # gather state information
        s = state

        ship_x = s['ai_position']

        enemies_left_positions = s['enemies_left_positions']
        enemies_right_positions = s['enemies_right_positions']

        bullets_left_positions = s['bullets_left_positions']
        bullets_right_positions = s['bullets_right_positions']

        ai_shoots = s['can_shoot']

        ai_last_shot = s['ai_last_shot_time']

        player_avg_frequency = self.FREQUENCY_BOUND
        if s['player_avg_frequency']:
            player_avg_frequency = min(self.FREQUENCY_BOUND, int(s['player_avg_frequency']))

        num_left_enemies = len(enemies_left_positions)
        num_right_enemies = len(enemies_right_positions)

        attack_left = True

        left = False
        right = False
        shoot = False
        hit = False

        # find bullet that should determine movement
        nearest_bullet = [0,0]

        if ship_x >= self.MIDDLE:
            bullets_to_search = bullets_right_positions
        else:
            bullets_to_search = bullets_left_positions
        
        if ship_x >= self.MIDDLE - 25 and ship_x <= self.MIDDLE +25:
            bullets_to_search = bullets_left_positions + bullets_right_positions

        problem_bullets = []
        x_diff_prev = self.CANVAS

        for bullet in bullets_to_search:
            x_diff = abs(bullet[0] - ship_x)
            if bullet[1] < self.SHIP_Y and bullet[1] > self.VERTICAL_BUFFER and x_diff < self.HIT_RANGE * 2:
                problem_bullets.append(bullet)
                if x_diff < x_diff_prev:
                    nearest_bullet = bullet
                    x_diff_prev = x_diff

        # find nearest enemy
        nearest_enemy = [200,0]
        nearest_x_diff = self.CANVAS

        # check if ai_agent is in danger of being hit by bullet
        if nearest_bullet[0] <= ship_x + self.HIT_RANGE and nearest_bullet[0] >= ship_x - self.HIT_RANGE:
            hit = True

        if hit:
            if nearest_bullet[0] >= self.CANVAS-75:
                left = True
            elif nearest_bullet[0] <= self.min_x + 55:
                right = True
            elif nearest_bullet[0] > ship_x:
                left = True
            elif nearest_bullet[0] <= ship_x:
                right = True


        # For if we want AI ship going left when doing zombie mode
        if 200 < ship_x and self.zombie_go_left:
            if not(nearest_bullet[0] < ship_x and self.SHIP_Y - nearest_bullet[1] < 200 and ship_x - nearest_bullet[0] <= self.SECOND_HIT_RANGE):
                left = True
        if 200 >= ship_x:
            self.zombie_go_left = False

        # if no more enemies to worry about, do nothing
        if num_right_enemies==0 and not(attack_left):
            left = False
            right = False
        shoot = False
        # pass back action
        action = {
            'left': left,
            'right': right,
            'shoot': shoot
        }
        return action