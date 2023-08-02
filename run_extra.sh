#name=empty-32-32
#name=room-32-32-4
#name=maze-32-32-2
#name=ost003d
#name=lak303d
#name=warehouse-10-20-10-2-1

name=warehouse-10-20-10-2-1
map=instances/mapf-map/$name.map
scen=instances/scen-random/$name-random

tl=3000

# 5 recompute mdd
# 6 no recompute mdd

for k in $(seq 100 50 200)
do
  mode=5
  output=warehouse_test_unitlength/$name-$k-$mode-random.csv
  for i in $(seq 1 1 25)
  do
    echo    ./lns -m $map -a $scen-$i.scen -o $output -k $k -t $tl --agtselection=$mode
    ./lns -m $map -a $scen-$i.scen -o $output -k $k -t $tl --agtselection=$mode
  done
done