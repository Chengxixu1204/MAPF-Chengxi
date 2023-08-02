
#name=empty-32-32
#name=room-32-32-4
#name=lak303d
name=random-32-32-20
map=instances/mapf-map/$name.map
scen=instances/scen-random/$name-random

tl=3600

mode=0
k=150
pshuffle=0.75
mkdir test/test_partial/$name
mkdir test/test_partial/$name/$k
for i in $(seq 1 1 25)
do
  output=test/test_partial/$name/$k/$name-$k-$mode-$pshuffle-random-$i.csv
  for pseed in $(seq 1 1 100)
  do
    echo    ./lns -m $map -a $scen-$i.scen -o $output -k $k -t $tl --agtselection=$mode --pshuffle=$pshuffle --pseed=$pseed
    #./lns -m $map -a $scen-$i.scen -o $output -k $k -t $tl --agtselection=$mode --pshuffle=$pshuffle --pseed=$pseed
  done
done
#./lns -m instances/mapf-map/random-32-32-20.map -a instances/scen-random/random-32-32-20-random-25.scen -o test/test_partial/random-32-32-20/150/random-32-32-20-150-0-0.75-random-25.csv -k 150 -t 3600 --agtselection=0 --pshuffle=0.75 --pseed=100
#./lns -m instances/mapf-map/random-32-32-20.map -a instances/scen-random/random-32-32-20-random-25.scen -o test/tmp.csv -k 150 -t 3600 --agtselection=5
#./lns -m instances/mapf-map/warehouse-10-20-10-2-1.map -a instances/scen-random/warehouse-10-20-10-2-1-random-25.scen -o test/tmp.csv -k 10 -t 3600 --agtselection=5

