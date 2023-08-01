import time
import random

class AiBehavior:
    def __init__(self):
        # self.min_x = 425
        self.min_x = 0
        self.min_x_shutter = 400
        self.strategy = self.active
        
        self.CANVAS = 800 # width of canvas
        self.MIDDLE = 400 # middle of x axis
        self.SHOOTING_RANGE = 24 # hit enemy if within range
        self.HIT_RANGE = 35 # range for dodging bullets
        self.SECOND_HIT_RANGE = 50 # range check for moving towards enemies

        self.SHIP_Y = 540 # y value of ai_ship
        self.VERTICAL_BUFFER = 440 # vertical buffer for bullets (higher numbers means closer to ship)

        self.AI_RELATIVE_SPEED = 0.6 # relatively how much shorter time between bullets is for ai_agent
        self.FREQUENCY_BOUND = 4000 # ai max time between shots

        self.attack_left = False

        self.start_support = random.randint(1,2)
        self.support = 0
        self.support_changed = False
        self.ai_shoots = False

        self.last_shot_time = 0
        self.shooting_interval = .2  #.2 for very fast , 2 for very slow
        #self.shoots_player = False
        self.counter_right = 0
        self.counter_left = 0
        


    def update(self, state):
        return self.strategy(state)

    def run_ai(self, state):
        """ Check whether ship should target left (human player's) enemies """
        return False

    def active(self, state):
        # print("UNCOOPERATIVE")
        s = state

        ship_x = s['ai_position']

        enemies_left_positions = s['enemies_left_positions']
        enemies_left_shot = s['enemies_left_shot']
        enemies_right_positions = s['enemies_right_positions']

        bullets_left_positions = s['bullets_left_positions']
        bullets_right_positions = s['bullets_right_positions']

        self.ai_shoots = s['can_shoot']
        self.shoots_player = s['can_shoot_shutter']

        ai_last_shot = s['ai_last_shot_time']
        shutter_last_shot = s['player2_last_shot_time']



        player_avg_frequency = self.FREQUENCY_BOUND
        if s['player_avg_frequency']:
            player_avg_frequency = min(self.FREQUENCY_BOUND, int(s['player_avg_frequency']))

        num_left_enemies = len(enemies_left_positions)
        num_right_enemies = len(enemies_right_positions)


        shutter_ai_position= s['player2_position']



        self.attack_left = self.run_ai(state)



        if self.support == 1:
            enemies_to_search = enemies_left_positions
        else:
            enemies_to_search = enemies_right_positions
        #print('self support', self.support)

        left = False
        right = False
        shoot = False
        hit = False

        player_left = False
        player_right = False
        player_shoot = False
        player_hit = False

    






################## SHUTTER AGENT######

         # find bullet that should determine movement
        nearest_bullet = [0,0]
        enemies_to_search = enemies_right_positions


        if shutter_ai_position >= self.MIDDLE:
            bullets_to_search = bullets_right_positions
        else:
            bullets_to_search = bullets_left_positions
        
        if shutter_ai_position >= self.MIDDLE - 25 and shutter_ai_position <= self.MIDDLE +25:
            bullets_to_search = bullets_left_positions + bullets_right_positions

        
        problem_bullets = []
        x_diff_prev = self.CANVAS
        #print('bullets to search', bullets_to_search)

        for bullet in bullets_to_search:
            x_diff = abs(bullet[0] - shutter_ai_position)
            #print(bullet[1] < self.SHIP_Y,bullet[1] > self.VERTICAL_BUFFER ,x_diff < self.HIT_RANGE * 2 )
            if bullet[1] < self.SHIP_Y and bullet[1] > self.VERTICAL_BUFFER and x_diff < self.HIT_RANGE * 2:
                #print('here')
                problem_bullets.append(bullet)
                if x_diff < x_diff_prev:
                    nearest_bullet = bullet
                    x_diff_prev = x_diff

        # find nearest enemy
        nearest_enemy = [0,0]
        nearest_x_diff = self.CANVAS

        # if self.attack_left:
        #     enemies_to_search = enemies_left_positions
        # else:
        #     enemies_to_search = enemies_right_positions
        
        
        #enemies_to_search = enemies_left_positions

        for enemy in enemies_to_search:
            # print("ENEMENY", enemy)
            check_distance = abs(enemy[0] - shutter_ai_position)
            index = enemies_to_search.index(enemy)
            if check_distance < nearest_x_diff and enemies_left_shot[index] == False:
                nearest_enemy = enemy
                nearest_x_diff = check_distance

        # set restriction on frequency based on player's recent average
        #if round(time.time() * 1000) - ai_last_shot < player_avg_frequency * self.AI_RELATIVE_SPEED:
        #    self.ai_shoots = False 

        # Set restriction on shooting frequency based on custom shooting frequency
        # Check if enough time has passed since t, he last shot
        #print('ai',round(time.time() * 1000),ai_last_shot,player_avg_frequency,self.AI_RELATIVE_SPEED)
        if round(time.time() * 1000) - shutter_last_shot < player_avg_frequency * self.AI_RELATIVE_SPEED:

            self.shoots_player = False 

       # print('ai can shoot',self.ai_shoots)

        # current_time = time.time()
        # if current_time - self.last_shot_time > self.shooting_interval:
        #     print('shoot is true' )
        #     self.ai_shoots = True
        #     self.last_shot_time = current_time  # Update the last shot time
        # else:
        #     print('cant shoot')
        #     self.ai_shoots = False

        # check if ai_agent is in danger of being hit by bullet

        #print('nearest bullet[0', nearest_bullet[0], 'nearest bullet',nearest_bullet, 'shutter position', shutter_ai_position)
        #print('1',nearest_bullet[0] <= shutter_ai_position + self.HIT_RANGE, '2',nearest_bullet[0] >= shutter_ai_position - self.HIT_RANGE)
        if nearest_bullet[0] <= shutter_ai_position + self.HIT_RANGE and nearest_bullet[0] >= shutter_ai_position - self.HIT_RANGE:
            

            player_hit = True
        #print('nearest bullet',nearest_bullet)
        approached_enemy = False


        if not(self.shoots_player):
            #print("cant shoot")
            # if can't shoot, figure out which way to move
            print('1')
            if player_hit:
                print('az')
                if nearest_bullet[0] >= self.CANVAS-75:
                    print('2')
                    player_left = True
                elif nearest_bullet[0] <= self.min_x_shutter +55:
                    print('3')
                    player_right = True
                elif nearest_bullet[0] > shutter_ai_position:
                    print('4')
                    player_left = True
                elif nearest_bullet[0] < shutter_ai_position:
                    print('5')
                    player_right = True


            print('time',s['timer'])

            # else:
            #     if self.counter_right >0 and self.counter_right <=5:
            #         player_right = True
            #         self.counter_right +=1 

            #     elif self.counter_left >0 and self.counter_left <=5:
                    


            #     if random.choice([True, False]):
            #         player_right = True
            #         self.counter_right +=1 
            #         if self.counter == 5:
            #             self.counter = 0
            #     else:
            #         player_left = True
                    
            #         self.counter +=1 
            #         if self.counter == 5:
            #             self.counter = 0
            

        else:
            #IF AI CAN SHOOT
            print('ca')
            if nearest_x_diff <= self.SHOOTING_RANGE:
                print('6')
                player_shoot = True

            if nearest_enemy[0] < shutter_ai_position:
                print('7')
                if not(nearest_bullet[0] < shutter_ai_position and self.SHIP_Y - nearest_bullet[1] < 200 and shutter_ai_position - nearest_bullet[0] <= self.SECOND_HIT_RANGE):
                    #print("TARGET LEFT")
                    print('8')
                    player_left = True
                    approached_enemy = True
            elif nearest_enemy[0] > shutter_ai_position:
                print('9')
                if not(nearest_bullet[0] > shutter_ai_position and self.SHIP_Y - nearest_bullet[1] < 200 and nearest_bullet[0] - shutter_ai_position <= self.SECOND_HIT_RANGE):
                    #print("TARGET RIGHT")
                    print('10')
                    player_right = True
                    approached_enemy = True
                    
            if not approached_enemy and player_hit:
                print('11')
                if x_diff_prev >5:
                    # if not going to hit bullet, don't shoot so you can move
                    player_shoot = False
                    print('12')
                # figure out which way to move so you don't get hit
                if nearest_bullet[0] >= self.CANVAS-75:
                    print('13')
                    player_left = True
                elif nearest_bullet[0] <= self.min_x_shutter + 55:
                    print('14')
                    player_right = True
                elif nearest_bullet[0] > shutter_ai_position:
                    print('15')
                    player_left = True
                elif nearest_bullet[0] <= shutter_ai_position:
                    print('16')
                    player_right = True



        print(player_left,player_right)

#
        # pass back action

        #print(player_left,player_right)
        #print('end')
        action = {
            'left': left,
            'right': right,
            'shoot': shoot,
            'support': self.support,

            'player_left':player_left,
            'player_right': player_right,
            'player_shoot': player_shoot,

        }
        return action

        
