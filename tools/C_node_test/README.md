# C_node_test

This is a test node, testing the dora-rs/dora#502 problem

node_A: publishes an array of length 3 at a frequency of 10Hz, and the number range in the array is 0-255

node_B: publishes a number with a frequency of 0-255 at a frequency of 100Hz

node_C: receives and displays the data sent by node_A and node_B

Tested a few times and 502 problem didnot happen for current main branch.

## Compile 

```
mkdir build
cd build
cmake ..
make
```

## Start Test

```
dora start dataflow.yml --name test
```

