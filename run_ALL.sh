#name=empty-32-32
#name=room-32-32-4
#name=maze-32-32-2
#name=ost003d
#name=lak303d
#name=warehouse-10-20-10-2-1

name=random-32-32-20
map=instances/mapf-map/$name.map
scen=instances/scen-random/$name-random

tl=50000

# 5 recompute mdd
# 6 no recompute mdd

k=150
mode=5
i=9
output="test_for_distribution/$name-$k-$mode-random.csv"

echo "./lns -m $map -a $scen-$i.scen -o $output -k $k -t $tl --agtselection=$mode"
./lns -m $map -a $scen-$i.scen -o $output -k $k -t $tl --agtselection=$mode

