
#name=empty-32-32
#name=room-32-32-4
#name=lak303d
map=instances/mapf-map/$name.map
scen=instances/scen-random/$name-random

k=300
tl=36000

mode=0
k=800
pshuffle=0.5
output=test/test_partial/$name-$k-$mode-$pshuffle-random.csv
for i in $(seq 1 1 25)
do
  for j in $(pseed 1 1 10)
  echo    ./lns -m map -a scen-$i.scen -o $output -k $k -t $tl --agtselection=$mode
  ./lns -m $map -a $scen-$i.scen -o $output -k $k -t $tl --agtselection=$mode
done




for k in $(seq 850 50 900)
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