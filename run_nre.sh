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

k=150  # Set the agent count to 125

for repeat in $(seq 2 1 5)  # Repeat 5 times
do
  mode=5  # Your mode
  output_folder="random_test_unit_uniform"  # Your output folder
  output_prefix="$name-$k-$mode-random"  # Prefix for the output file
  
  for i in $(seq 1 1 25)  # Loop over scenarios 1 to 25
  do
    output="$output_folder/$output_prefix-$repeat.csv"  # Create a unique output file for each run
    echo    "./lns -m $map -a $scen-$i.scen -o $output -k $k -t $tl --agtselection=$mode"
    ./lns -m $map -a $scen-$i.scen -o $output -k $k -t $tl --agtselection=$mode
  done
done