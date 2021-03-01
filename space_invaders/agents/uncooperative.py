class Uncooperative:
    def update(self, state):
        #TODO: make it non-random
        s = state

        ai_y = 540
        ai_x = s['ai_position']
        ai_shoots = s['can_shoot']

        shoot = False
        right = False
        left = False
        hit_range = False

        nearest_enemy_diff = s['nearest_enemy']['x'] - ai_x

        if abs(nearest_enemy_diff) <= 24:
            shoot = True
        
        if not ai_shoots:
            if (s['nearest_bullet']['x'] <= ai_x + 30 and s['nearest_bullet']['x'] >= ai_x - 30):
                hit_range = True
            
            if hit_range:
                # if s['nearest_bullet']['x'] <= ai_x + 55:
                #     right = True
                if s['nearest_bullet']['x'] > ai_x:
                    left = True
                else:
                    right = True

        if nearest_enemy_diff > 0:
            if (not (s['nearest_bullet']['x'] > ai_x and
                s['nearest_bullet']['y'] < ai_y + 240 and
                s['nearest_bullet']['x'] - ai_x <= 45)):
                right = True
        elif nearest_enemy_diff < 0:
            if (not (s['nearest_bullet']['x'] < ai_x and
                s['nearest_bullet']['y'] < ai_y + 240 and
                ai_x - s['nearest_bullet']['x'] <= 45)):
                left = True
        
        
    #     //if (!ai_shoots) {
    # //    if (nearest_bullet.x <= ai_ship.sprite.x + 30 && nearest_bullet.x >= ai_ship.sprite.x - 30) {
    # //        hit = true;
    # //    }

    # //    if (hit && nearest_bullet.x >= this.sys.canvas.width - 75) {
    # //        left_final = true;
    # //    }
    # //    else if (hit && nearest_bullet.x <= ai_ship.min_x + 55) {
    # //        right_final = true;
    # //    }
    # //    else if (hit && nearest_bullet.x > ai_ship.sprite.x) {
    # //        left_final = true;
    # //    }
    # //    else if (hit && nearest_bullet.x <= ai_ship.sprite.x) {
    # //        right_final = true;
    # //    }

    #     if (nearest_enemy.x < ai_ship.sprite.x) {
    # //        // make sure it doesn't drive into bullets
    # //        if (!(nearest_bullet.x < ai_ship.sprite.x && 
    # //            nearest_bullet.y < ai_ship.sprite.y + 200 &&
    # //            ai_ship.sprite.x - nearest_bullet.x <= 35)) {
    # //            left_final = true;
    # //        }
    # //    }
    # //    if (nearest_enemy.x > ai_ship.sprite.x) {
    # //        if (!(nearest_bullet.x > ai_ship.sprite.x && 
    # //            nearest_bullet.y < ai_ship.sprite.y + 200 && 
    # //            nearest_bullet.x - ai_ship.sprite.x <= 35)) {
    # //            right_final = true;
    # //        }
    # //    }
        
        action = {
            'left': left,
            'right': right,
            'shoot': shoot,
        }
        return action