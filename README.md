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
<v1> -- <v2>, load1  
<v3> -- <v4>, load2  
    ... 
```
**v1, v2, ...** - vertices defining an edges  
**load1, load2 ...** - edge atributs  
At the end, the program will output the painted graph. 
## How to run tests:
#### In that case you can generate end2end tests do:
```bash
./tests/end2end <tests_number>
```
***tests_number*** - the number of tests you want to generate.  
After running you can see a generated directory - `tests/end2end/resources/`.
There will be the tests themselves and the answers to them.
#### To run end2end tests do:
```bash
bash ../tests/end2end/test_runner.sh <test_nu> <tests_directory> <answers_directory>
```
***test_num*** - number of tests you want to run(must be correct number)  
***tests_directory*** - directory with tests.  
***answers_directory*** - directory with the answers.  
By default tests and answers will be launched from the `tests/end2end/resources` directory.
> Note: tests should be named like this: test[i], and the answers to them:
        ans[i], where i is the test number
