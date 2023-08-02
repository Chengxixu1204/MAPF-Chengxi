import csv 
from statistics import mean

mapname, num_agents = 'lak303d', [500]#],550,600,650,700,750,800]
#mapname, num_agents = 'maze-32-32-2', [50, 70, 90, 110,130]
#mapname, num_agents = 'ost003d', [400, 500, 600, 700,800]
#mapname, num_agents = 'room-32-32-4', [50, 75, 100,125,150]
#mapname, num_agents = 'warehouse-10-20-10-2-1', [100, 200, 300, 400]

path = mapname+"-%d-%d-random.csv-LNS.csv"
mode = [0,4,3,5]

if True:
    for k in num_agents:
        print(f"#agents = {k}")
        commonly_solved = None
        for m in mode:
            file_name = path % (k, m)
            instance_name = []
            with open(file_name, mode='r') as csv_file:
                csv_reader = csv.DictReader(csv_file)
                for i, row in enumerate(csv_reader):
                    if i == 0: continue
                    if not (eval(row['solution cost']) > 0):
                        continue
                    instance_name.append(row['instance name'])
                    
                print(f"mode {m}: solved {len(instance_name)}")
                if commonly_solved == None:
                    commonly_solved = set(instance_name)
                else:
                    commonly_solved = commonly_solved.intersection(set(instance_name))
                
        for m in mode:
            solution = []
            lb = []
            ratio = []
            file_name = path % (k, m)
            
            with open(file_name, mode='r') as csv_file:
                csv_reader = csv.DictReader(csv_file)
                for i, row in enumerate(csv_reader):
                    if i == 0: continue
                    if not (row['instance name'] in commonly_solved): continue
                    if eval(row['solution cost']) > 0:
                        solution.append(eval(row['solution cost']))
                        lb.append(eval(row['lower bound']))
                        ratio.append(solution[-1]/lb[-1])
            print("#agent = %d mode = %d" % (k, m))
            print(len(solution))
            if len(solution) == 0:
                print("NA")
            else:
                print(mean(ratio))
if False:
    pshuffle = ["0.0", "-0.75", "0.75", "0.5", "-0.5"]
    num_agents = 150
    mapname = "random-32-32-20"
    path = 'test_partial/'+mapname+"/"+str(num_agents)+"/"+mapname+"-"+str(num_agents)+"-0-"+"%s-random-%d.csv-LNS.csv"
    for ps in pshuffle:
        solution = []
        lb = []
        ratio = []
        for i in range(1, 26):
            file_name = path % (ps, i)
            with open(file_name, mode='r') as csv_file:
                csv_reader = csv.DictReader(csv_file)
                for j, row in enumerate(csv_reader):
                    #if j == 0: continue
                    if eval(row['solution cost']) > 0:
                        solution.append(eval(row['solution cost']))
                        lb.append(eval(row['lower bound']))
                        ratio.append(solution[-1]/lb[-1])
        print("#ps = %s" % (ps))
        print(len(solution))
        if len(solution) == 0:
            print("NA")
        else:
            print(mean(ratio))
