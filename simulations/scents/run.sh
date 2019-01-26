#!/usr/bin/env bash

INET_DIR="$HOME/Downloads/dev_tools/inet"

echo "inet path: $INET_DIR"

# ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## #
# # Chart1 scenarios
declare -a chart1_arr=("Scenario1_Independent" "Scenario2_RandomSelect" "Scenario3_Optimized")

for scenario in "${chart1_arr[@]}"; do
    echo "Running test set: [" $scenario "]"

    RUNS=$(../../src/blendsim -m -n ..:../../src:$INET_DIR/src:$INET_DIR/examples:$INET_DIR/tutorials:$INET_DIR/showcases --image-path=$INET_DIR/images -l $INET_DIR/src/INET omnetpp.ini -c $scenario -s -q runnumbers)
    
    echo "Runs found:" $RUNS

    for i in $RUNS; do
     	echo "Running test:" $i
	../../src/blendsim -m -n ..:../../src:$INET_DIR/src:$INET_DIR/examples:$INET_DIR/tutorials:$INET_DIR/showcases --image-path=$INET_DIR/images -l $INET_DIR/src/INET omnetpp.ini -c $scenario -r $i &>/dev/null
    done
done
# ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## #


# ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## #
# Chart2 scenario
scenario="Scenario4"
echo "Running test set: [" $scenario "]"

RUNS=$(../../src/blendsim -m -n ..:../../src:$INET_DIR/src:$INET_DIR/examples:$INET_DIR/tutorials:$INET_DIR/showcases --image-path=$INET_DIR/images -l $INET_DIR/src/INET omnetpp.ini -c $scenario -r '$RR == 100 && $ON < 100 && $repetition == 2' -s -q runnumbers)
    
echo "Runs found:" $RUNS

for i in $RUNS; do
    echo "Running test:" $i
    ../../src/blendsim -m -n ..:../../src:$INET_DIR/src:$INET_DIR/examples:$INET_DIR/tutorials:$INET_DIR/showcases --image-path=$INET_DIR/images -l $INET_DIR/src/INET omnetpp.ini -c $scenario -r $i &>/dev/null
done
# ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## #


# ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## #
# Chart 4 scenario
scenario="Scenario6"
echo "Running test set: [" $scenario "]"

RUNS=$(../../src/blendsim -m -n ..:../../src:$INET_DIR/src:$INET_DIR/examples:$INET_DIR/tutorials:$INET_DIR/showcases --image-path=$INET_DIR/images -l $INET_DIR/src/INET omnetpp.ini -c $scenario -s -q runnumbers)
    
echo "Runs found:" $RUNS

for i in $RUNS; do
    echo "Running test:" $i
    ../../src/blendsim -m -n ..:../../src:$INET_DIR/src:$INET_DIR/examples:$INET_DIR/tutorials:$INET_DIR/showcases --image-path=$INET_DIR/images -l $INET_DIR/src/INET omnetpp.ini -c $scenario -r $i &>/dev/null
done
# ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## #


# ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## #
# Chart 5 scenarios

#declare -a chart5_arr=("Scenario7_SCENTS" "Scenario7_INDEPENDENT")
declare -a chart5_arr=("Scenario7_SCENTS")

for scenario in "${chart5_arr[@]}"; do
    echo "Running test set: [" $scenario "]"

    RUNS=$(../../src/blendsim -m -n ..:../../src:$INET_DIR/src:$INET_DIR/examples:$INET_DIR/tutorials:$INET_DIR/showcases --image-path=$INET_DIR/images -l $INET_DIR/src/INET omnetpp.ini -c $scenario -r '$repetition > 2' -s -q runnumbers)
    
    echo "Runs found:" $RUNS

    for i in $RUNS; do
     	echo "Running test:" $i
	../../src/blendsim -m -n ..:../../src:$INET_DIR/src:$INET_DIR/examples:$INET_DIR/tutorials:$INET_DIR/showcases --image-path=$INET_DIR/images -l $INET_DIR/src/INET omnetpp.ini -c $scenario -r $i &>/dev/null
    done
done

# ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## ## #
