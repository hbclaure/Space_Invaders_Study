#!/usr/bin/env python

import rosbag
import os
import json
import csv  


def split_bag(bag,games,file):
    with rosbag.Bag(f'/data/bitmap/NaoSpaceInvaders_Summer2022/game_bags/{file.split(".")[0]}_g1.bag', 'w') as outbag1:
        with rosbag.Bag(f'/data/bitmap/NaoSpaceInvaders_Summer2022/game_bags/{file.split(".")[0]}_g2.bag', 'w') as outbag2:
            with rosbag.Bag(f'/data/bitmap/NaoSpaceInvaders_Summer2022/game_bags/{file.split(".")[0]}_g3.bag', 'w') as outbag3:
                with rosbag.Bag(f'/data/bitmap/NaoSpaceInvaders_Summer2022/game_bags/{file.split(".")[0]}_g4.bag', 'w') as outbag4:
                    with rosbag.Bag(f'/data/bitmap/NaoSpaceInvaders_Summer2022/game_bags/{file.split(".")[0]}_g5.bag', 'w') as outbag5:
                        with rosbag.Bag(f'/data/bitmap/NaoSpaceInvaders_Summer2022/game_bags/{file.split(".")[0]}_g6.bag', 'w') as outbag6:
                            for topic, msg, t in bag.read_messages():
                                if t.to_sec() >= games[f"g1_start"] and t.to_sec() <= games[f"g1_end"]:
                                    outbag1.write(topic,msg,t)
                                elif t.to_sec() >= games[f"g2_start"] and t.to_sec() <= games[f"g2_end"]:
                                    outbag2.write(topic,msg,t)
                                elif t.to_sec() >= games[f"g3_start"] and t.to_sec() <= games[f"g3_end"]:
                                    outbag3.write(topic,msg,t)
                                elif t.to_sec() >= games[f"g4_start"] and t.to_sec() <= games[f"g4_end"]:
                                    outbag4.write(topic,msg,t)
                                elif t.to_sec() >= games[f"g5_start"] and t.to_sec() <= games[f"g5_end"]:
                                    outbag5.write(topic,msg,t)
                                elif t.to_sec() >= games[f"g6_start"] and t.to_sec() <= games[f"g6_end"]:
                                    outbag6.write(topic,msg,t)

def main():
    #load alread done
    with open('/data/bitmap/NaoSpaceInvaders_Summer2022/game_bags_complete/game_bags_complete.csv', newline='') as f:
        reader = csv.reader(f)
        already_done = [row[0] for row in reader]

    path_name = "/data/bitmap/NaoSpaceInvaders_Summer2022/participantbags"
    folder = os.listdir(path_name)
    for file in folder:
        if file in already_done:
            pass
            print(f"Already split {file}")
        else:
            print(f"Parsing {file}...")
            rosbag_file = f"{path_name}/{file}"
            bag = rosbag.Bag(rosbag_file)
            # reset
            i = 0
            games = {"g1_start":0,"g1_end":0,"g2_start":0,"g2_end":0,"g3_start":0,"g3_end":0, "g4_start":0,"g4_end":0, "g5_start":0,"g5_end":0, "g6_start":0,"g6_end":0}
            for topic, msg, t in bag.read_messages(topics=['/space_invaders/game/game_mode',
                                                        '/space_invaders/game/nao_action']):
                if i == 0:
                    if topic == '/space_invaders/game/game_mode' and msg.data == "3":
                        games["g1_start"] = t.to_sec()
                        i+=1
                elif i == 1:
                    if topic == '/space_invaders/game/nao_action':
                        try:
                            if (msg.data.split(" ")[1] == "no_nap_i" or msg.data.split(" ")[1] == "no_nap_we"):
                                games["g1_end"] = t.to_sec()
                                i+=1
                        except IndexError:
                            if (msg.data == "no_nap_i" or msg.data == "no_nap_we"):
                                games["g1_end"] = t.to_sec()
                                i+=1
                elif i == 2:
                    if topic == '/space_invaders/game/game_mode' and msg.data == "3":
                        games["g2_start"] = t.to_sec()
                        i+=1
                elif i == 3:
                    if topic == '/space_invaders/game/nao_action':
                        try:
                            if msg.data.split(" ")[1]== "sleep_off":
                                games["g2_end"] = t.to_sec()
                                i+=1
                        except IndexError:
                            if msg.data == "sleep_off":
                                games["g2_end"] = t.to_sec()
                                i+=1
                elif i == 4:
                    if topic == '/space_invaders/game/game_mode' and msg.data == "1":
                        games["g3_start"] = t.to_sec()
                        i+=1    
                elif i == 5:
                    if topic == '/space_invaders/game/nao_action':
                        try:
                            if (msg.data.split(" ")[1] == "no_nap_i" or msg.data.split(" ")[1] == "no_nap_we"):
                                games["g3_end"] = t.to_sec()
                                i+=1
                        except IndexError:
                            if (msg.data == "no_nap_i" or msg.data == "no_nap_we"):
                                games["g3_end"] = t.to_sec()
                                i+=1
                elif i == 6:
                    if topic == '/space_invaders/game/game_mode' and msg.data == "1":
                        games["g4_start"] = t.to_sec()
                        i+=1
                elif i == 7:
                    if topic == '/space_invaders/game/nao_action':
                        try:
                            if msg.data.split(" ")[1]== "sleep_off":
                                games["g4_end"] = t.to_sec()
                                i+=1
                        except IndexError:
                            if msg.data == "sleep_off":
                                games["g4_end"] = t.to_sec()
                                i+=1
                elif i == 8:
                    if topic == '/space_invaders/game/game_mode' and msg.data == "2":
                        games["g5_start"] = t.to_sec()
                        i+=1
                elif i == 9:
                    if topic == '/space_invaders/game/nao_action':
                        try:
                            if (msg.data.split(" ")[1] == "no_nap_i" or msg.data.split(" ")[1] == "no_nap_we"):
                                games["g5_end"] = t.to_sec()
                                i+=1
                        except IndexError:
                            if (msg.data == "no_nap_i" or msg.data == "no_nap_we"):
                                games["g5_end"] = t.to_sec()
                                i+=1
                elif i == 10:
                    if topic == '/space_invaders/game/game_mode' and msg.data == "2":
                        games["g6_start"] = t.to_sec()
                        i+=1
                elif i == 11:
                    if topic == '/space_invaders/game/nao_action':
                        try:
                            if msg.data.split(" ")[1]== "sleep_off":
                                games["g6_end"] = t.to_sec()
                                i+=1
                        except IndexError:
                            if msg.data == "sleep_off":
                                games["g6_end"] = t.to_sec()
                                i+=1
            if games["g6_end"] != 0:
                split_bag(bag,games,file)
                with open(f'/data/bitmap/NaoSpaceInvaders_Summer2022/game_bags_complete/{file}',"a") as f:
                    json.dump(games,f)
                with open('/data/bitmap/NaoSpaceInvaders_Summer2022/game_bags_complete/game_bags_complete.csv', 'a', encoding='UTF8') as f:
                    writer = csv.writer(f)
                        # write the header
                    writer.writerow([file])
                print(f"{file} split!")
            else:
                print(f"*** Error with {file}...")
                with open(f'/data/bitmap/NaoSpaceInvaders_Summer2022/bag_issues/{file}',"a") as f:
                    json.dump(games,f)


            bag.close()

if __name__ == "__main__":
    main()