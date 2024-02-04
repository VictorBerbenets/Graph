# Implementation of Donald Knuth's graph  
### About
This program checks whether a given graph is bipartite and,  
if so, paints the graph in two colors - blue and red.  
Otherwise an error message is displayed.
## Requirements
**cmake** version must be 3.15 or higher
## How to build
```bash
git clone git@github.com:VictorBerbenets/Graph.git
cd Graph/
cmake -S ./ -B build/ -DCMAKE_BUILD_TYPE=Release
cd build/
cmake --build .
```
## To Run the program do
```bash
./graph
```
The programm will be waiting for input data in stdin in such way:  
```bash
<v1> -- <v2>, load  
```
**v1, v2** - vertices defining an edge  
**load** - edge atribut  
At the end, the program will output the painted graph. 
## How to run tests:
```bash
bash ../tests/end2end/test_runner.sh <tests_number>
```
**tests_number** - the number of tests you want to generate.  
