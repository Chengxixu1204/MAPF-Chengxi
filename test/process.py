import csv




map_config = dict()
map_config["lak303d"] = (500, 50, 800, "lak303d")
map_config["ost003d"] = (400, 100, 800, "ost003d")
map_config["maze"] = (50, 20, 130, "maze-32-32-2")
map_config["random"] = (150, 10, 200, "random-32-32-20")
map_config["room"] = (50, 25, 150, "room-32-32-4")
map_config["warehouse"] = (100, 100, 500, "warehouse-10-20-10-2-1")

mode_list = [0, 3, 4]

agent_num = map_config[map_name][0]
while agent_num <= map_config[map_name][2]:
    
    with open('eggs.csv', newline='') as csvfile:
    for mode in mode_list:
        csv_file_name = map_config[3] + "-%d-%d-random.csv"%(agent_num, mode)
    agent_num += map_config[map_name][1]
