class CooperativeEarly:
    def update(self, state):
        s = state

        ship_x = s['ai_position']
        ship_min_x = s['ai_ship_min_x']
        ship_y = s['ai_pos_y']

        enemies_left_positions = s['enemies_left_positions']
        enemies_right_positions = s['enemies_right_positions']

        bullets_left_positions = s['bullets_left_positions']
        bullets_right_positions = s['bullets_right_positions']

        ai_shoots = s['can_shoot']

        num_left_enemies = len(enemies_left_positions)
        num_right_enemies = len(enemies_right_positions)

        canvas_width = s['canvas_width']

        attack_left = False
        left = False
        right = False
        shoot = False
        hit = False

        if num_right_enemies > 17:
            attack_left = False
        elif num_left_enemies > 12:
            attack_left = True
        elif num_right_enemies > 9:
            attack_left = False
        elif num_left_enemies > 0:
            attack_left = True
        else:
            attack_left = False

        nearest_bullet = [0,0]

        if ship_x >= 400:
            bullets_to_search = bullets_right_positions
        else:
            bullets_to_search = bullets_left_positions

        for bullet in bullets_to_search:
            if bullet[1] < ship_y + 15 and bullet[1] > nearest_bullet[1]:
                nearest_bullet = bullet

        nearest_enemy = [0,0]
        nearest_x_diff = canvas_width

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
            print("cant shoot")
            if nearest_bullet[0] <= ship_x +30 and nearest_bullet[0] >= ship_x - 30:
                hit = True
            if hit and nearest_bullet[0] >= canvas_width-75:
                left = True
            elif hit and nearest_bullet[0] <= ship_min_x + 55:
                right = True
            elif hit and nearest_bullet[0] > ship_x:
                left = True
            elif hit and nearest_bullet[0] <= ship_x:
                right = True
        else:
            print("can shoot")
            if nearest_x_diff <= 24:
                shoot = True
                print("yes shoot")
            if nearest_enemy[0] < ship_x:
                print(" if left")
                if not(nearest_bullet[0] < ship_x and nearest_bullet[1] < ship_y + 200 and ship_x - nearest_bullet[0] <= 35):
                    left = True
                    print("yes left")
            if nearest_enemy[0] > ship_x:
                print("if right")
                print(nearest_bullet[0], "gt", ship_x, "*", nearest_bullet[1], "lt", ship_y + 200, "*", ship_x - nearest_bullet[0],"lte", 35)
                if not(nearest_bullet[0] > ship_x and nearest_bullet[1] < ship_y + 200 and nearest_bullet[0] - ship_x <= 35):
                    right = True
                    print("yes right")





        action = {
            'left': left,
            'right': right,
            'shoot': shoot
        }
        return action