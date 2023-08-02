
#name=empty-32-32
#name=room-32-32-4
#name=warehouse-10-20-10-2-1
#name=ost003d
name=maze-32-32-2
map=instances/mapf-map/$name.map
scen=instances/scen-random/$name-random

k=300
tl=36000

for k in $(seq 50 20 130)
do
  mode=0
  output=test/$name-$k-$mode-random.csv
  for i in $(seq 1 1 25)
  do
    echo    ./lns -m map -a scen-$i.scen -o $output -k $k -t $tl --agtselection=$mode
    ./lns -m $map -a $scen-$i.scen -o $output -k $k -t $tl --agtselection=$mode
  done

  mode=3
  output=test/$name-$k-$mode-random.csv
  for i in $(seq 1 1 25)
  do
    echo    ./lns -m map -a scen-$i.scen -o $output -k $k -t $tl --agtselection=$mode
    ./lns -m $map -a $scen-$i.scen -o $output -k $k -t $tl --agtselection=$mode
  done
done