class Uncooperative:
    def __init__(self):
        self.min_x = 425

    def check_attack_left(self, state):
        return False


    def update(self, state):
        SHIP_Y = 540
        CANVAS = 800

        s = state

        ship_x = s['ai_position']

        enemies_left_positions = s['enemies_left_positions']
        enemies_right_positions = s['enemies_right_positions']

        bullets_left_positions = s['bullets_left_positions']
        bullets_right_positions = s['bullets_right_positions']

        ai_shoots = s['can_shoot']

        num_left_enemies = len(enemies_left_positions)
        num_right_enemies = len(enemies_right_positions)

        attack_left = self.check_attack_left(state)

        left = False
        right = False
        shoot = False
        hit = False

        nearest_bullet = [0,0]

        if ship_x >= 400:
            bullets_to_search = bullets_right_positions
        else:
            bullets_to_search = bullets_left_positions

        for bullet in bullets_to_search:
            if bullet[1] < SHIP_Y + 15 and bullet[1] > nearest_bullet[1]:
                nearest_bullet = bullet

        nearest_enemy = [0,0]
        nearest_x_diff = CANVAS

        if attack_left:
            enemies_to_search = enemies_left_positions
        else:
            enemies_to_search = enemies_right_positions

        for enemy in enemies_to_search:
            check_distance = abs(enemy[0] - ship_x)
            if check_distance < nearest_x_diff:
                nearest_enemy = enemy
                nearest_x_diff = check_distance

        

        if not(ai_shoots):
            #print("cant shoot")
            if nearest_bullet[0] <= ship_x +30 and nearest_bullet[0] >= ship_x - 30:
                hit = True
            if hit:
                if nearest_bullet[0] >= CANVAS-75:
                    left = True
                elif nearest_bullet[0] <= self.min_x + 55:
                    right = True
                elif nearest_bullet[0] > ship_x:
                    left = True
                elif nearest_bullet[0] <= ship_x:
                    right = True
        else:
            if nearest_x_diff <= 24:
                shoot = True
            if nearest_enemy[0] < ship_x:
                if not(nearest_bullet[0] < ship_x and nearest_bullet[1] < SHIP_Y + 240 and ship_x - nearest_bullet[0] <= 45):
                    left = True
            if nearest_enemy[0] > ship_x:
                if not(nearest_bullet[0] > ship_x and nearest_bullet[1] < SHIP_Y + 240 and nearest_bullet[0] - ship_x <= 45):
                    right = True

        if num_right_enemies==0 and not(attack_left):
            left = False
            right = False

        action = {
            'left': left,
            'right': right,
            'shoot': shoot
        }
        return action